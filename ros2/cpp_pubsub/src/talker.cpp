// ROS 2 C++ talker — mirrors dora-rs/rs-latency/node.
//
// Publishes UInt64MultiArray messages of varying sizes on /topic.
// First element of each payload is a CLOCK_MONOTONIC timestamp (ns).

#include <chrono>
#include <cstdint>
#include <random>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

// Match dora-rs/rs-latency sizes (number of uint64 elements).
static const std::vector<size_t> SIZES = {
    1, 10 * 512, 100 * 512, 1000 * 512, 10000 * 512};
static const int SAMPLES_PER_SIZE = 1000;
static const auto TICK = std::chrono::milliseconds(20);

static uint64_t clock_monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cpp_benchmark_talker");
  auto publisher =
      node->create_publisher<std_msgs::msg::UInt64MultiArray>("topic", 10);

  // Give subscriber time to connect.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::mt19937_64 rng(42);

  for (auto size : SIZES) {
    for (int i = 0; i < SAMPLES_PER_SIZE; ++i) {
      auto msg = std_msgs::msg::UInt64MultiArray();
      msg.data.resize(size);
      for (size_t j = 0; j < size; ++j) {
        msg.data[j] = rng();
      }
      msg.data[0] = clock_monotonic_ns();
      publisher->publish(msg);
      std::this_thread::sleep_for(TICK);
    }
  }

  // Grace period.
  std::this_thread::sleep_for(std::chrono::seconds(3));
  rclcpp::shutdown();
  return 0;
}
