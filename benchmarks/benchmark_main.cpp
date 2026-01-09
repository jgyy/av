#include <benchmark/benchmark.h>
#include "av/foundation/math.hpp"

using namespace av;

// Benchmark vector operations
static void BM_Vector3Normalize(benchmark::State& state) {
    Vec3 v(3.0f, 4.0f, 5.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(normalize(v));
    }
}

static void BM_Vector3DotProduct(benchmark::State& state) {
    Vec3 v1(1.0f, 2.0f, 3.0f);
    Vec3 v2(4.0f, 5.0f, 6.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(dot(v1, v2));
    }
}

static void BM_Vector3CrossProduct(benchmark::State& state) {
    Vec3 v1(1.0f, 2.0f, 3.0f);
    Vec3 v2(4.0f, 5.0f, 6.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(cross(v1, v2));
    }
}

BENCHMARK(BM_Vector3Normalize);
BENCHMARK(BM_Vector3DotProduct);
BENCHMARK(BM_Vector3CrossProduct);

// Benchmark other operations
BENCHMARK_MAIN();
