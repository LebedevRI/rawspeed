/*
    RawSpeed - RAW file decoder.

    Copyright (C) 2024 Roman Lebedev

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

#include "io/BitStreamerJPEG.h"
#include "adt/Array1DRef.h"
#include "adt/Casts.h"
#include "adt/DefaultInitAllocatorAdaptor.h"
#include "adt/Invariant.h"
#include "adt/Optional.h"
#include "bench/Common.h"
#include "common/Common.h"
#include "io/BitStreamerMSB.h"
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <random>
#include <vector>
#include <benchmark/benchmark.h>

#ifndef NDEBUG
#include <limits>
#endif

namespace rawspeed {

namespace {

// i'th element is the frequency with which
// the byte `i` is found in an average JPEG byte stream.
//
// NOTE: that `a[0xFF]` means you need to refer to the next table!
constexpr std::array<uint64_t, 256> ByteFrequency = {
    37430671, 14132311, 13984933, 12761251, 15441489, 15917873, 14033532,
    14720343, 15330852, 14359336, 16465670, 15011391, 15115707, 15633862,
    15688679, 16849808, 15609394, 18281908, 15998397, 15569330, 17006307,
    19161398, 17085848, 16684239, 15877118, 16756080, 18202399, 17255236,
    18718957, 17213057, 18037193, 20235778, 15603036, 15646190, 18543234,
    18838400, 18256526, 17680266, 17011355, 17696070, 16852012, 16762342,
    19169721, 19859483, 18944292, 18110439, 17600510, 18690491, 14453933,
    17940152, 18183410, 17409100, 16738209, 19943804, 18787970, 17751720,
    17347265, 20394637, 18395264, 17499103, 19061319, 19361768, 20736052,
    22088160, 14690894, 16965679, 14968528, 16506585, 18188943, 18859887,
    18415428, 19955138, 16811981, 18846460, 17484928, 18654803, 16535273,
    17963646, 17546062, 19483908, 15935965, 19412589, 17245314, 17704296,
    18048301, 23388723, 19929128, 21906184, 18643445, 20551023, 19435427,
    19707654, 19520686, 19205539, 19683939, 21785956, 14509978, 17205392,
    18264798, 20193450, 18947924, 19501788, 18176200, 19198067, 15966764,
    18122816, 19218268, 21440898, 19245733, 19215315, 17907790, 19021123,
    15827043, 18973606, 20233163, 20367551, 17151769, 19668756, 18554930,
    17254756, 17067414, 20387741, 19405590, 19668835, 20327445, 25203205,
    20918424, 23220775, 13339541, 13420639, 16352894, 15989776, 14704472,
    16125700, 16720195, 17822469, 17812784, 17207645, 19111593, 18754421,
    17515179, 19816542, 20222074, 21406958, 16128757, 19357388, 19935517,
    19143192, 16922138, 20127725, 19970834, 19608663, 16511615, 18834660,
    18485337, 19277887, 19043270, 18686678, 20367485, 22287927, 15052425,
    15835782, 18156821, 19531813, 17407175, 18452761, 17504494, 19338126,
    17911263, 18183973, 21709325, 21965187, 20243198, 21030734, 21131023,
    22778300, 17256270, 20529013, 20262540, 19965143, 17355681, 20694967,
    19680487, 19172007, 17449952, 20204714, 18429885, 18310909, 18439569,
    19748953, 25089818, 22097152, 12897032, 15590523, 15836310, 18037553,
    17154124, 19046730, 18920612, 21699659, 18669348, 20234800, 19580543,
    20915178, 18800093, 19806009, 20160527, 23176542, 15454569, 18369799,
    18606228, 19132365, 18297527, 20579792, 21351414, 22028029, 19137664,
    19687607, 18620857, 19145016, 18143657, 17540079, 18456299, 25020540,
    13939516, 16684341, 17946927, 20428670, 19967527, 20984363, 20425768,
    24167106, 17843968, 19616616, 19668881, 21952056, 19576231, 18552022,
    17766458, 24468667, 14798841, 19376094, 20728004, 24226516, 20309956,
    21938120, 19565819, 24969400, 17097824, 24536621, 22854688, 24953682,
    21311845, 22454503, 23018858, 22961508};

// i'th element is the frequency with which a sequence `0xFF00` consecutively
// repeated `i` times is found in an average JPEG byte stream.
constexpr std::array<uint64_t, 4> NumConsecutive0xFF00Frequency = {0, 22878475,
                                                                   82708, 325};

struct JPEGStuffedByteStreamGenerator final {
  std::vector<uint8_t,
              DefaultInitAllocatorAdaptor<uint8_t, std::allocator<uint8_t>>>
      dataStorage;
  int64_t numBytesGenerated;

  [[nodiscard]] Array1DRef<const uint8_t> getInput() const {
    return {dataStorage.data(), implicit_cast<int>(dataStorage.size())};
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpragmas"
#pragma GCC diagnostic ignored "-Wunknown-warning-option"
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma GCC diagnostic ignored "-Wframe-larger-than="
#pragma GCC diagnostic ignored "-Wstack-usage="
  __attribute__((noinline)) explicit JPEGStuffedByteStreamGenerator(
      const int64_t numBytesMax) {
    invariant(numBytesMax > 0);
    const auto expectedOverhead = roundUpDivision(numBytesMax, 100); // <=1%
    dataStorage.reserve(implicit_cast<size_t>(numBytesMax + expectedOverhead));

    // Here we only need to differentiate between a normal byte,
    // and an 0xFF00 sequence, so clump together non-0xFF frequencies.
    // This makes distribution sampling -40% faster.
    constexpr uint64_t TotalWeight = std::accumulate(
        ByteFrequency.begin(), ByteFrequency.end(), uint64_t(0));
    constexpr uint64_t ControlSequenceStartWeight = ByteFrequency.back();

    std::bernoulli_distribution controlSequenceStartDistribution(
        implicit_cast<double>(ControlSequenceStartWeight) /
        implicit_cast<double>(TotalWeight));
    std::discrete_distribution<uint8_t> numConsecutive0xFF00Distribution(
        NumConsecutive0xFF00Frequency.begin(),
        NumConsecutive0xFF00Frequency.end());

    std::random_device rd;
    std::mt19937_64 gen(rd());

    for (numBytesGenerated = 0; numBytesGenerated < numBytesMax;) {
      bool isNormalByte = !controlSequenceStartDistribution(gen);
      if (isNormalByte) {
        dataStorage.emplace_back(0x00);
        ++numBytesGenerated;
      } else {
        const int len = numConsecutive0xFF00Distribution(gen);
        invariant(len > 0);
        for (int i = 0; i != len; ++i) {
          dataStorage.emplace_back(0xFF);
          dataStorage.emplace_back(0x00); // This is a no-op stuffing byte.
        }
        numBytesGenerated += len;
      }
    }
    invariant(numBytesGenerated >= numBytesMax);
  }
#pragma GCC diagnostic pop
};

struct JPEGUnstuffedByteStreamGenerator final {
  std::vector<uint8_t,
              DefaultInitAllocatorAdaptor<uint8_t, std::allocator<uint8_t>>>
      dataStorage;
  int64_t numBytesGenerated;

  [[nodiscard]] Array1DRef<const uint8_t> getInput() const {
    return {dataStorage.data(), implicit_cast<int>(dataStorage.size())};
  }

  __attribute__((noinline)) explicit JPEGUnstuffedByteStreamGenerator(
      const int64_t numBytesMax)
      : numBytesGenerated(numBytesMax) {
    invariant(numBytesGenerated > 0);
    dataStorage.resize(implicit_cast<size_t>(numBytesGenerated), 0x00);
  }
};

template <typename T> void BM(benchmark::State& state, bool Stuffed) {
  int64_t numBytes = state.range(0);
  assert(numBytes > 0);
  assert(numBytes <= std::numeric_limits<int>::max());

  Optional<JPEGStuffedByteStreamGenerator> genStuffed;
  Optional<JPEGUnstuffedByteStreamGenerator> genUnstuffed;
  Optional<Array1DRef<const uint8_t>> input;
  if (Stuffed) {
    genStuffed.emplace(numBytes);
    numBytes = genStuffed->numBytesGenerated;
    input = genStuffed->getInput();
  } else {
    genUnstuffed.emplace(numBytes);
    numBytes = genUnstuffed->numBytesGenerated;
    input = genUnstuffed->getInput();
  }
  benchmark::DoNotOptimize(input->begin());

  for (auto _ : state) {
    T bs(*input);

    constexpr int MaxGetBits = 32;
    int processedBytes = 0;
    for (processedBytes = 0; processedBytes != numBytes;
         processedBytes += MaxGetBits / 8) {
      uint32_t bits = bs.getBits(MaxGetBits);
      benchmark::DoNotOptimize(bits);
    }
    invariant(numBytes == processedBytes);
  }

  state.SetComplexityN(numBytes);
  state.counters.insert({
      {"Throughput",
       benchmark::Counter(sizeof(uint8_t) * state.complexity_length_n(),
                          benchmark::Counter::Flags::kIsIterationInvariantRate,
                          benchmark::Counter::kIs1024)},
      {"Latency",
       benchmark::Counter(sizeof(uint8_t) * state.complexity_length_n(),
                          benchmark::Counter::Flags::kIsIterationInvariantRate |
                              benchmark::Counter::Flags::kInvert,
                          benchmark::Counter::kIs1000)},
  });
}

void CustomArguments(benchmark::internal::Benchmark* b) {
  b->Unit(benchmark::kMicrosecond);
  b->RangeMultiplier(2);

  static constexpr int L1dByteSize = 32U * (1U << 10U);
  static constexpr int L2dByteSize = 512U * (1U << 10U);
  static constexpr int MaxBytesOptimal = L2dByteSize * (1U << 5);

  if (benchmarkDryRun()) {
    b->Arg(L1dByteSize);
    return;
  }

  // NOLINTNEXTLINE(readability-simplify-boolean-expr)
  if constexpr ((true)) {
    b->Arg(MaxBytesOptimal);
  } else {
    b->Range(8, MaxBytesOptimal * (1U << 2));
    b->Complexity(benchmark::oN);
  }
}

#ifndef BENCHMARK_TEMPLATE1_CAPTURE
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define BENCHMARK_TEMPLATE1_CAPTURE(func, a, test_case_name, ...)              \
  BENCHMARK_PRIVATE_DECLARE(func) =                                            \
      (::benchmark::internal::RegisterBenchmarkInternal(                       \
          new ::benchmark::internal::FunctionBenchmark(                        \
              #func "<" #a ">"                                                 \
                    "/" #test_case_name,                                       \
              [](::benchmark::State& st) { func<a>(st, __VA_ARGS__); })))
#endif // BENCHMARK_TEMPLATE1_CAPTURE

BENCHMARK_TEMPLATE1_CAPTURE(BM, BitStreamerJPEG, Stuffed, true)
    ->Apply(CustomArguments);
BENCHMARK_TEMPLATE1_CAPTURE(BM, BitStreamerJPEG, Unstuffed, false)
    ->Apply(CustomArguments);
BENCHMARK_TEMPLATE1_CAPTURE(BM, BitStreamerMSB, Unstuffed, false)
    ->Apply(CustomArguments);

} // namespace

} // namespace rawspeed

BENCHMARK_MAIN();
