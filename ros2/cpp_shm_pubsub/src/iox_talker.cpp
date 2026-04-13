// Pure iceoryx talker — bypasses CycloneDDS entirely.
// True zero-copy: write directly into shared memory, no serialization.

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <random>
#include <thread>
#include <chrono>
#include <vector>

#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

static uint64_t clock_monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1'000'000'000ULL + (uint64_t)ts.tv_nsec;
}

// Fixed-size payloads matching our benchmark sizes.
struct Bench8     { uint64_t data[1]; };
struct Bench40K   { uint64_t data[5120]; };
struct Bench400K  { uint64_t data[51200]; };
struct Bench4M    { uint64_t data[512000]; };

template <typename T>
void run_bracket(const char *service, const char *instance,
                 size_t n_elements, int samples) {
  iox::popo::Publisher<T> publisher(iox::capro::ServiceDescription{
      iox::capro::IdString_t(iox::cxx::TruncateToCapacity, "Benchmark"),
      iox::capro::IdString_t(iox::cxx::TruncateToCapacity, service),
      iox::capro::IdString_t(iox::cxx::TruncateToCapacity, instance)});

  // Pre-fill random data.
  std::mt19937_64 rng(42);
  std::vector<uint64_t> prefilled(n_elements);
  for (size_t j = 0; j < n_elements; ++j)
    prefilled[j] = rng();

  // Warmup.
  for (int w = 0; w < 10; ++w) {
    publisher.loan()
        .and_then([&](auto &sample) {
          std::memset(sample->data, 0, n_elements * sizeof(uint64_t));
          sample->data[0] = clock_monotonic_ns();
          sample.publish();
        })
        .or_else([](auto &e) {
          fprintf(stderr, "warmup loan failed: %d\n", (int)e);
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  printf("bracket %s: %zu bytes, %d samples\n", service, n_elements * 8, samples);

  for (int i = 0; i < samples; ++i) {
    publisher.loan()
        .and_then([&](auto &sample) {
          std::memcpy(sample->data, prefilled.data(),
                      n_elements * sizeof(uint64_t));
          sample->data[0] = clock_monotonic_ns();
          sample.publish();
        })
        .or_else([](auto &e) {
          fprintf(stderr, "loan failed: %d\n", (int)e);
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

int main() {
  iox::runtime::PoshRuntime::initRuntime("iox_benchmark_talker");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  run_bracket<Bench8>("8B", "latency", 1, 1000);
  run_bracket<Bench40K>("40KB", "latency", 5120, 1000);
  run_bracket<Bench400K>("400KB", "latency", 51200, 1000);
  run_bracket<Bench4M>("4MB", "latency", 512000, 1000);

  std::this_thread::sleep_for(std::chrono::seconds(3));
  return 0;
}
