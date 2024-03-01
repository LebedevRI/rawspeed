/*
    RawSpeed - RAW file decoder.

    Copyright (C) 2009-2014 Klaus Post
    Copyright (C) 2014-2015 Pedro Côrte-Real
    Copyright (C) 2017-2024 Roman Lebedev

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include "decompressors/OlympusDecompressor.h"
#include "adt/Array2DRef.h"
#include "adt/Bit.h"
#include "adt/Casts.h"
#include "adt/Invariant.h"
#include "adt/Point.h"
#include "bitstreams/BitStreamerMSB.h"
#include "common/Common.h"
#include "common/RawImage.h"
#include "common/SimpleLUT.h"
#include "decoders/RawDecoderException.h"
#include "decompressors/AbstractDecompressor.h"
#include "io/ByteStream.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <utility>

namespace rawspeed {

namespace {

class OlympusDifferenceDecoder final {
  const SimpleLUT<int8_t, 12>& numLZ;

  std::array<int, 3> carry{{}};

public:
  // NOLINTNEXTLINE(google-explicit-constructor)
  OlympusDifferenceDecoder(const SimpleLUT<int8_t, 12>& numLZ_)
      : numLZ(numLZ_) {}

  inline __attribute__((always_inline)) int16_t getDiff(BitStreamerMSB& bits);
};

inline __attribute__((always_inline)) int16_t
OlympusDifferenceDecoder::getDiff(BitStreamerMSB& bits) {
  bits.fill();

  int numLowBitsBias = (carry[2] < 3) ? 2 : 0;
  int numLowBits = numActiveBits(implicit_cast<uint16_t>(carry[0]));
  numLowBits -= numLowBitsBias;
  numLowBits = std::max(numLowBits, 2 + numLowBitsBias);
  assert(numLowBits >= 2);
  assert(numLowBits <= 14);

  int b = bits.peekBitsNoFill(15);
  int sign = (b >> 14) * -1;
  int low = (b >> 12) & 3;
  int numLeadingZeros = numLZ[b & 4095];

  int highBits;
  // Skip bytes used above or read bits
  if (numLeadingZeros == 12) {
    bits.skipBitsNoFill(15);
    int numHighBits = 15 - numLowBits;
    assert(numHighBits >= 1);
    assert(numHighBits <= 13);
    highBits = bits.peekBitsNoFill(numHighBits);
    bits.skipBitsNoFill(1 + numHighBits);
  } else {
    bits.skipBitsNoFill(numLeadingZeros + 1 + 3);
    highBits = numLeadingZeros;
  }

  carry[0] = (highBits << numLowBits) | bits.getBitsNoFill(numLowBits);
  int diff = (carry[0] ^ sign) + carry[1];
  carry[1] = (diff * 3 + carry[1]) >> 5;
  carry[2] = carry[0] > 16 ? 0 : carry[2] + 1;

  diff *= 4;
  diff |= low;

  // This is a 12-bit raw format. Pixel values are in [0, 4095],
  // therefore the largest possible differences are [-4095, +4095],
  // which means (valid) difference is 13-bit signed integer.
  // That being said, for corrupted files, it can be larger.

  return static_cast<int16_t>(diff);
}

class OlympusDecompressorImpl final : public AbstractDecompressor {
  RawImage mRaw;

  // A table to quickly look up the number of leading zeros in a value.
  const SimpleLUT<int8_t, 12> numLZ{
      [](size_t i, [[maybe_unused]] unsigned tableSize) {
        return 12 - numActiveBits(i);
      }};

  static __attribute__((always_inline)) int getPred(Array2DRef<uint16_t> out,
                                                    int row, int col);

  inline __attribute__((always_inline)) void
  decodeDiffGroup(std::array<OlympusDifferenceDecoder, 2>& acarry,
                  BitStreamerMSB& bits, int row, int group) const;

  inline __attribute__((always_inline)) void
  decodeDiffBlock(std::array<OlympusDifferenceDecoder, 2>& acarry,
                  BitStreamerMSB& bits, int row, int firstGroup,
                  int lastGroup) const;

  inline __attribute__((always_inline)) void predictGroup(int row,
                                                          int group) const;

  inline __attribute__((always_inline)) void
  predictBlock(int row, int firstGroup, int lastGroup) const;

  inline __attribute__((always_inline)) void
  decompressBlock(std::array<OlympusDifferenceDecoder, 2>& acarry,
                  BitStreamerMSB& bits, int row, int firstGroup,
                  int lastGroup) const;

  void decompressRow(BitStreamerMSB& bits, int row) const;

public:
  explicit OlympusDecompressorImpl(RawImage img) : mRaw(std::move(img)) {}

  void decompress(ByteStream input) const;
};

/* This is probably the slowest decoder of them all.
 * I cannot see any way to effectively speed up the prediction
 * phase, which is by far the slowest part of this algorithm.
 * Also there is no way to multithread this code, since prediction
 * is based on the output of all previous pixel (bar the first four)
 */

inline __attribute__((always_inline)) int
OlympusDecompressorImpl::getPred(const Array2DRef<uint16_t> out, int row,
                                 int col) {
  auto getLeft = [&]() { return out(row, col - 2); };
  auto getUp = [&]() { return out(row - 2, col); };
  auto getLeftUp = [&]() { return out(row - 2, col - 2); };

  int pred;
  if (row < 2 && col < 2)
    pred = 0;
  else if (row < 2)
    pred = getLeft();
  else if (col < 2)
    pred = getUp();
  else {
    int left = getLeft();
    int up = getUp();
    int leftUp = getLeftUp();

    int leftMinusNw = left - leftUp;
    int upMinusNw = up - leftUp;

    // Check if sign is different, and they are both not zero
    if ((std::signbit(leftMinusNw) != std::signbit(upMinusNw)) &&
        (leftMinusNw != 0 && upMinusNw != 0)) {
      if (std::abs(leftMinusNw) > 32 || std::abs(upMinusNw) > 32)
        pred = left + upMinusNw;
      else
        pred = (left + up) >> 1;
    } else
      pred = std::abs(leftMinusNw) > std::abs(upMinusNw) ? left : up;
  }

  return pred;
}

inline __attribute__((always_inline)) void
OlympusDecompressorImpl::decodeDiffGroup(
    std::array<OlympusDifferenceDecoder, 2>& acarry, BitStreamerMSB& bits,
    int row, int group) const {
  const Array2DRef<uint16_t> out(mRaw->getU16DataAsUncroppedArray2DRef());

  for (int c = 0; c != 2; ++c) {
    const int col = 2 * group + c;
    OlympusDifferenceDecoder& carry = acarry[c];
    int16_t diff = carry.getDiff(bits);
    out(row, col) = static_cast<uint16_t>(diff);
  }
}

inline __attribute__((always_inline)) void
OlympusDecompressorImpl::decodeDiffBlock(
    std::array<OlympusDifferenceDecoder, 2>& acarry, BitStreamerMSB& bits,
    int row, int firstGroup, int lastGroup) const {
  for (int group = firstGroup; group != lastGroup; ++group)
    decodeDiffGroup(acarry, bits, row, group);
}

inline __attribute__((always_inline)) void
OlympusDecompressorImpl::predictGroup(int row, int group) const {
  const Array2DRef<uint16_t> out(mRaw->getU16DataAsUncroppedArray2DRef());

  for (int c = 0; c != 2; ++c) {
    const int col = 2 * group + c;
    int diff = static_cast<int16_t>(out(row, col));
    int pred = getPred(out, row, col);
    out(row, col) = implicit_cast<uint16_t>(pred + diff);
  }
}

inline __attribute__((always_inline)) void
OlympusDecompressorImpl::predictBlock(int row, int firstGroup,
                                      int lastGroup) const {
  for (int group = firstGroup; group != lastGroup; ++group)
    predictGroup(row, group);
}

inline __attribute__((always_inline)) void
OlympusDecompressorImpl::decompressBlock(
    std::array<OlympusDifferenceDecoder, 2>& acarry, BitStreamerMSB& bits,
    int row, int firstGroup, int lastGroup) const {
  decodeDiffBlock(acarry, bits, row, firstGroup, lastGroup);
  predictBlock(row, firstGroup, lastGroup);
}

void OlympusDecompressorImpl::decompressRow(BitStreamerMSB& bits,
                                            int row) const {
  const Array2DRef<uint16_t> out(mRaw->getU16DataAsUncroppedArray2DRef());

  invariant(out.width() > 0);
  invariant(out.width() % 2 == 0);

  std::array<OlympusDifferenceDecoder, 2> acarry{numLZ, numLZ};

  const int numGroups = out.width() / 2;

  // It's best to process pixels in blocks, that are large-ish,
  // but small-enough to fully fit into CPU L1d cache.
  constexpr int blockSize = 1;

  const auto numBlocks =
      implicit_cast<int>(roundUpDivision(numGroups, blockSize));

  for (int block = 0; block != numBlocks; ++block) {
    int firstGroup = blockSize * block;
    int lastGroup = std::min(firstGroup + blockSize, numGroups);
    decompressBlock(acarry, bits, row, firstGroup, lastGroup);
  }
}

void OlympusDecompressorImpl::decompress(ByteStream input) const {
  invariant(mRaw->dim.y > 0);
  invariant(mRaw->dim.x > 0);
  invariant(mRaw->dim.x % 2 == 0);

  input.skipBytes(7);
  BitStreamerMSB bits(input.peekRemainingBuffer().getAsArray1DRef());

  for (int y = 0; y < mRaw->dim.y; y++)
    decompressRow(bits, y);
}

} // namespace

OlympusDecompressor::OlympusDecompressor(RawImage img) : mRaw(std::move(img)) {
  if (mRaw->getCpp() != 1 || mRaw->getDataType() != RawImageType::UINT16 ||
      mRaw->getBpp() != sizeof(uint16_t))
    ThrowRDE("Unexpected component count / data type");

  if (!mRaw->dim.hasPositiveArea() || mRaw->dim.x % 2 != 0 ||
      mRaw->dim.y % 2 != 0 || mRaw->dim.x > 10400 || mRaw->dim.y > 7792)
    ThrowRDE("Unexpected image dimensions found: (%u; %u)", mRaw->dim.x,
             mRaw->dim.y);
}

void OlympusDecompressor::decompress(ByteStream input) const {
  OlympusDecompressorImpl(mRaw).decompress(input);
}

} // namespace rawspeed
