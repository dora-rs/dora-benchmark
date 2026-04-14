// ROS 2 C++ listener — mirrors dora-rs/rs-latency/sink.
//
// Subscribes to /topic, measures latency per size bracket, and writes
// results to time.csv with the same schema as dora-rs/rs-latency/timer.csv.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

static uint64_t clock_monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

static uint64_t percentile_us(std::vector<uint64_t> &sorted, double pct) {
  if (sorted.empty())
    return 0;
  size_t idx = static_cast<size_t>(
      std::round((pct / 100.0) * static_cast<double>(sorted.size() - 1)));
  if (idx >= sorted.size())
    idx = sorted.size() - 1;
  return sorted[idx];
}

static void record_results(const std::string &date, size_t size_bytes,
                           std::vector<uint64_t> &latencies_us) {
  std::sort(latencies_us.begin(), latencies_us.end());
  uint64_t n = latencies_us.size();
  uint64_t sum = 0;
  for (auto v : latencies_us)
    sum += v;
  uint64_t avg = sum / n;
  uint64_t p50 = percentile_us(latencies_us, 50.0);
  uint64_t p90 = percentile_us(latencies_us, 90.0);
  uint64_t p99 = percentile_us(latencies_us, 99.0);

  const char *csv_env = std::getenv("CSV_TIME_FILE");
  std::string csv_path = csv_env ? csv_env : "time.csv";

  const char *name_env = std::getenv("NAME");
  std::string name = name_env ? name_env : "ROS 2 C++";

  std::ofstream f(csv_path, std::ios::app);
  f << date << ",C++,COMPUTER_PERF," << name << "," << size_bytes << "," << avg
    << "," << p50 << "," << p90 << "," << p99 << "," << n << "\n";

  std::cout << "size=" << size_bytes << "  avg=" << avg << "us  p50=" << p50
            << "us  p90=" << p90 << "us  p99=" << p99 << "us  n=" << n
            << std::endl;
}

class Listener : public rclcpp::Node {
public:
  Listener()
      : Node("cpp_benchmark_listener"), current_size_(0) {
    // Get a rough ISO timestamp for CSV labelling.
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    date_ = buf;

    subscription_ = this->create_subscription<std_msgs::msg::UInt64MultiArray>(
        "topic", 10,
        [this](const std_msgs::msg::UInt64MultiArray::SharedPtr msg) {
          this->callback(msg);
        });
  }

  ~Listener() {
    if (!latencies_us_.empty()) {
      record_results(date_, current_size_, latencies_us_);
    }
  }

private:
  void callback(const std_msgs::msg::UInt64MultiArray::SharedPtr msg) {
    uint64_t t_received = clock_monotonic_ns();
    size_t size_bytes = msg->data.size() * 8;

    if (size_bytes != current_size_) {
      if (!latencies_us_.empty()) {
        record_results(date_, current_size_, latencies_us_);
      }
      current_size_ = size_bytes;
      latencies_us_.clear();
    }

    uint64_t t_send = msg->data[0];
    uint64_t latency_ns = t_received - t_send;
    latencies_us_.push_back(latency_ns / 1000); // ns -> us
  }

  rclcpp::Subscription<std_msgs::msg::UInt64MultiArray>::SharedPtr
      subscription_;
  size_t current_size_;
  std::vector<uint64_t> latencies_us_;
  std::string date_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
