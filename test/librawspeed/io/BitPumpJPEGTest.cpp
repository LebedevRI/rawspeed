/*
    RawSpeed - RAW file decoder.

    Copyright (C) 2018 Roman Lebedev

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

#include "io/BitPumpJPEG.h" // for BitPumpJPEG
#include "common/Common.h"  // for uchar8
#include "io/BitPumpTest.h" // for BitPumpTest
#include "io/BitStream.h"   // for BitStream
#include "io/Buffer.h"      // for Buffer
#include "io/ByteStream.h"  // for ByteStream
#include "io/Endianness.h"  // for getHostEndianness, Endianness::big, Endia...
#include <array>            // for array
#include <gtest/gtest.h>    // for Message, AssertionResult, ASSERT_PRED_FOR...

using rawspeed::BitPumpJPEG;
using rawspeed::Buffer;
using rawspeed::ByteStream;
using rawspeed::DataBuffer;
using rawspeed::Endianness;

namespace rawspeed_test {

template <>
const std::array<rawspeed::uchar8, 4> BitPumpTest<BitPumpJPEG>::ones = {
    /* [Byte0 Byte1 Byte2 Byte3] */
    /* Byte: [Bit0 .. Bit7] */
    0b10100100, 0b01000010, 0b00001000, 0b00011111};

template <>
const std::array<rawspeed::uchar8, 4> BitPumpTest<BitPumpJPEG>::invOnes = {
    0b11010010, 0b00100001, 0b00000100, 0b00001111};

INSTANTIATE_TYPED_TEST_CASE_P(JPEG, BitPumpTest, BitPumpJPEG);

TEST(BitPumpJPEGTest, 0xFF0x00Is0xFFTest) {
  // If 0xFF0x00 byte sequence is found, it is just 0xFF, i.e. 0x00 is ignored.
  static const std::array<rawspeed::uchar8, 2 + 4> data{
      0xFF, 0x00, 0b10100100, 0b01000010, 0b00001000, 0b00011111};

  const Buffer b(data.data(), data.size());

  for (auto e : {Endianness::little, Endianness::big}) {
    const DataBuffer db(b, e);
    const ByteStream bs(db);

    BitPumpJPEG p(bs);

    ASSERT_EQ(p.getBits(8), 0xFF);

    for (int len = 1; len <= 7; len++)
      ASSERT_EQ(p.getBits(len), 1) << "     Where len: " << len;
  }
}

TEST(BitPumpJPEGTest, 0xFF0xXXIsTheEndTest) {
  // If 0xFF0xXX byte sequence is found, where XX != 0, then it is the end.
  for (rawspeed::uchar8 end = 0x01; end < 0xFF; end++) {
    static const std::array<rawspeed::uchar8, 2 + 4> data{0xFF, end,  0xFF,
                                                          0xFF, 0xFF, 0xFF};

    const Buffer b(data.data(), data.size());

    for (auto e : {Endianness::little, Endianness::big}) {
      const DataBuffer db(b, e);
      const ByteStream bs(db);

      BitPumpJPEG p(bs);

      for (int cnt = 0; cnt <= 64 + 32 - 1; cnt++)
        ASSERT_EQ(p.getBits(1), 0);
    }
  }
}

} // namespace rawspeed_test
