// Pure iceoryx listener — bypasses CycloneDDS entirely.
// True zero-copy: reads directly from shared memory.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

static uint64_t clock_monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1'000'000'000ULL + (uint64_t)ts.tv_nsec;
}

struct Bench8     { uint64_t data[1]; };
struct Bench40K   { uint64_t data[5120]; };
struct Bench400K  { uint64_t data[51200]; };
struct Bench4M    { uint64_t data[512000]; };

static uint64_t percentile_us(std::vector<uint64_t> &sorted, double pct) {
  if (sorted.empty()) return 0;
  size_t idx = (size_t)(pct / 100.0 * (double)(sorted.size() - 1) + 0.5);
  if (idx >= sorted.size()) idx = sorted.size() - 1;
  return sorted[idx];
}

static void record(const char *date, size_t size_bytes,
                   std::vector<uint64_t> &lat) {
  std::sort(lat.begin(), lat.end());
  uint64_t n = lat.size();
  uint64_t sum = 0;
  for (auto v : lat) sum += v;
  uint64_t avg = sum / n;
  uint64_t p50 = percentile_us(lat, 50);
  uint64_t p90 = percentile_us(lat, 90);
  uint64_t p99 = percentile_us(lat, 99);

  const char *name_env = std::getenv("NAME");
  const char *name = name_env ? name_env : "iceoryx direct";
  const char *csv_env = std::getenv("CSV_TIME_FILE");
  const char *csv = csv_env ? csv_env : "time.csv";

  FILE *f = fopen(csv, "a");
  fprintf(f, "%s,C++,COMPUTER_PERF,%s,%zu,%lu,%lu,%lu,%lu,%lu\n",
          date, name, size_bytes, avg, p50, p90, p99, n);
  fclose(f);

  printf("size=%zu  avg=%luus  p50=%luus  p90=%luus  p99=%luus  n=%lu\n",
         size_bytes, avg, p50, p90, p99, n);
}

template <typename T>
void poll_bracket(const char *service, size_t size_bytes, int expect,
                  const char *date, int warmup_skip,
                  std::vector<uint64_t> &latencies) {
  iox::popo::Subscriber<T> subscriber(iox::capro::ServiceDescription{
      iox::capro::IdString_t(iox::cxx::TruncateToCapacity, "Benchmark"),
      iox::capro::IdString_t(iox::cxx::TruncateToCapacity, service),
      iox::capro::IdString_t(iox::cxx::TruncateToCapacity, "latency")});
  latencies.clear();
  int skipped = 0;
  int received = 0;
  int idle = 0;

  while (received < expect) {
    bool got = false;
    subscriber.take().and_then([&](const auto &sample) {
      uint64_t t_received = clock_monotonic_ns();
      uint64_t t_send = sample->data[0];
      got = true;
      idle = 0;
      if (skipped < warmup_skip) {
        ++skipped;
        return;
      }
      latencies.push_back((t_received - t_send) / 1000);
      ++received;
    });
    if (!got) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      ++idle;
      if (idle > 100000) break; // 10s idle → give up
    }
  }

  if (!latencies.empty()) {
    record(date, size_bytes, latencies);
  }
}

int main() {
  iox::runtime::PoshRuntime::initRuntime("iox_benchmark_listener");

  // Timestamp for CSV.
  time_t now = time(nullptr);
  char date[64];
  strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", localtime(&now));

  std::vector<uint64_t> lat;
  lat.reserve(1100);

  poll_bracket<Bench8>("8B", 8, 1000, date, 10, lat);
  poll_bracket<Bench40K>("40KB", 40960, 1000, date, 10, lat);
  poll_bracket<Bench400K>("400KB", 409600, 1000, date, 10, lat);
  poll_bracket<Bench4M>("4MB", 4096000, 1000, date, 10, lat);

  return 0;
}
