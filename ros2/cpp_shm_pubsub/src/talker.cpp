// ROS 2 C++ zero-copy talker — CycloneDDS + iceoryx shared memory.
// Pre-fills random data once per bracket, only updates timestamp per send.

#include <chrono>
#include <cstdint>
#include <cstring>
#include <random>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "cpp_shm_pubsub/msg/bench8.hpp"
#include "cpp_shm_pubsub/msg/bench40_k.hpp"
#include "cpp_shm_pubsub/msg/bench400_k.hpp"
#include "cpp_shm_pubsub/msg/bench4_m.hpp"

static const int SAMPLES = 1000;
static const auto TICK = std::chrono::milliseconds(20);

static uint64_t clock_monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

template <typename MsgT>
void run_bracket(rclcpp::Publisher<MsgT> &pub, size_t n_elements,
                 rclcpp::Logger logger) {
  // Pre-fill random data once.
  std::mt19937_64 rng(42);
  std::vector<uint64_t> prefilled(n_elements);
  for (size_t j = 0; j < n_elements; ++j)
    prefilled[j] = rng();

  bool loaned = pub.can_loan_messages();
  RCLCPP_INFO(logger, "size=%zuB loan=%d", n_elements * 8, loaned);

  // Warmup: borrow + touch + publish a few messages to pre-fault SHM pages.
  if (loaned) {
    for (int w = 0; w < 10; ++w) {
      auto loan = pub.borrow_loaned_message();
      auto &data = loan.get().data;
      std::memset(data.data(), 0, n_elements * sizeof(uint64_t));
      data[0] = clock_monotonic_ns();
      pub.publish(std::move(loan));
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // Let warmup messages drain.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  for (int i = 0; i < SAMPLES; ++i) {
    if (loaned) {
      auto loan = pub.borrow_loaned_message();
      auto &data = loan.get().data;
      // memcpy pre-filled data into loaned SHM buffer.
      std::memcpy(data.data(), prefilled.data(), n_elements * sizeof(uint64_t));
      data[0] = clock_monotonic_ns();
      pub.publish(std::move(loan));
    } else {
      MsgT msg;
      std::memcpy(msg.data.data(), prefilled.data(),
                  n_elements * sizeof(uint64_t));
      msg.data[0] = clock_monotonic_ns();
      pub.publish(msg);
    }
    std::this_thread::sleep_for(TICK);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("shm_benchmark_talker");
  auto logger = node->get_logger();

  auto qos = rclcpp::QoS(1).best_effort();
  auto pub8 =
      node->create_publisher<cpp_shm_pubsub::msg::Bench8>("bench8", qos);
  auto pub40k =
      node->create_publisher<cpp_shm_pubsub::msg::Bench40K>("bench40k", qos);
  auto pub400k =
      node->create_publisher<cpp_shm_pubsub::msg::Bench400K>("bench400k", qos);
  auto pub4m =
      node->create_publisher<cpp_shm_pubsub::msg::Bench4M>("bench4m", qos);

  std::this_thread::sleep_for(std::chrono::seconds(2));

  run_bracket(*pub8, 1, logger);
  run_bracket(*pub40k, 5120, logger);
  run_bracket(*pub400k, 51200, logger);
  run_bracket(*pub4m, 512000, logger);

  std::this_thread::sleep_for(std::chrono::seconds(3));
  rclcpp::shutdown();
  return 0;
}
