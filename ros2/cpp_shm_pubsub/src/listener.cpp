// ROS 2 C++ zero-copy listener — CycloneDDS + iceoryx shared memory.
// Uses const-ref callbacks to avoid SharedPtr copies from SHM.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "cpp_shm_pubsub/msg/bench8.hpp"
#include "cpp_shm_pubsub/msg/bench40_k.hpp"
#include "cpp_shm_pubsub/msg/bench400_k.hpp"
#include "cpp_shm_pubsub/msg/bench4_m.hpp"

static uint64_t clock_monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

static uint64_t percentile_us(std::vector<uint64_t> &sorted, double pct) {
  if (sorted.empty()) return 0;
  size_t idx = static_cast<size_t>(
      std::round((pct / 100.0) * static_cast<double>(sorted.size() - 1)));
  if (idx >= sorted.size()) idx = sorted.size() - 1;
  return sorted[idx];
}

static void record_results(const std::string &date, size_t size_bytes,
                           std::vector<uint64_t> &latencies_us) {
  std::sort(latencies_us.begin(), latencies_us.end());
  uint64_t n = latencies_us.size();
  uint64_t sum = 0;
  for (auto v : latencies_us) sum += v;
  uint64_t avg = sum / n;
  uint64_t p50 = percentile_us(latencies_us, 50.0);
  uint64_t p90 = percentile_us(latencies_us, 90.0);
  uint64_t p99 = percentile_us(latencies_us, 99.0);

  const char *csv_env = std::getenv("CSV_TIME_FILE");
  std::string csv_path = csv_env ? csv_env : "time.csv";
  const char *name_env = std::getenv("NAME");
  std::string name = name_env ? name_env : "ROS 2 C++ SHM";

  std::ofstream f(csv_path, std::ios::app);
  f << date << ",C++,COMPUTER_PERF," << name << "," << size_bytes
    << "," << avg << "," << p50 << "," << p90 << "," << p99 << "," << n << "\n";

  std::cout << "size=" << size_bytes << "  avg=" << avg << "us  p50=" << p50
            << "us  p90=" << p90 << "us  p99=" << p99 << "us  n=" << n
            << std::endl;
}

class Listener : public rclcpp::Node {
public:
  Listener() : Node("shm_benchmark_listener") {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    date_ = buf;

    // Use const-ref callbacks — avoids any SharedPtr allocation/copy.
    auto qos = rclcpp::QoS(1).best_effort();
    sub8_ = create_subscription<cpp_shm_pubsub::msg::Bench8>(
        "bench8", qos,
        [this](cpp_shm_pubsub::msg::Bench8::UniquePtr msg) {
          on_msg(msg->data[0], 1 * 8);
        });
    sub40k_ = create_subscription<cpp_shm_pubsub::msg::Bench40K>(
        "bench40k", qos,
        [this](cpp_shm_pubsub::msg::Bench40K::UniquePtr msg) {
          on_msg(msg->data[0], 5120 * 8);
        });
    sub400k_ = create_subscription<cpp_shm_pubsub::msg::Bench400K>(
        "bench400k", qos,
        [this](cpp_shm_pubsub::msg::Bench400K::UniquePtr msg) {
          on_msg(msg->data[0], 51200 * 8);
        });
    sub4m_ = create_subscription<cpp_shm_pubsub::msg::Bench4M>(
        "bench4m", qos,
        [this](cpp_shm_pubsub::msg::Bench4M::UniquePtr msg) {
          on_msg(msg->data[0], 512000 * 8);
        });
  }

  ~Listener() { flush_all(); }

private:
  void on_msg(uint64_t t_send, size_t size_bytes) {
    uint64_t t_received = clock_monotonic_ns();
    uint64_t latency_us = (t_received - t_send) / 1000;
    auto &count = warmup_count_[size_bytes];
    if (count < 10) {
      ++count;
      return;
    }
    auto &lat = latencies_[size_bytes];
    lat.push_back(latency_us);
    // Flush results as soon as we hit 1000 samples for this bracket.
    if (lat.size() == 1000) {
      record_results(date_, size_bytes, lat);
    }
  }

  void flush_all() {
    std::vector<size_t> sizes;
    for (auto &kv : latencies_) sizes.push_back(kv.first);
    std::sort(sizes.begin(), sizes.end());
    for (auto sz : sizes) {
      auto &lat = latencies_[sz];
      if (!lat.empty())
        record_results(date_, sz, lat);
    }
  }

  std::map<size_t, std::vector<uint64_t>> latencies_;
  std::map<size_t, int> warmup_count_;
  std::string date_;

  rclcpp::Subscription<cpp_shm_pubsub::msg::Bench8>::SharedPtr sub8_;
  rclcpp::Subscription<cpp_shm_pubsub::msg::Bench40K>::SharedPtr sub40k_;
  rclcpp::Subscription<cpp_shm_pubsub::msg::Bench400K>::SharedPtr sub400k_;
  rclcpp::Subscription<cpp_shm_pubsub::msg::Bench4M>::SharedPtr sub4m_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
