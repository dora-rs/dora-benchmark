// ROS 2 C++ CUDA IPC talker — GPU-to-GPU latency benchmark.
// Allocates GPU buffers, extracts CUDA IPC handles, and publishes them.
// Supports IPC mode (default) and CPU fallback mode (CUDA_BENCH_MODE=cpu).

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <cuda_runtime_api.h>

#include "rclcpp/rclcpp.hpp"
#include "cuda_interfaces/msg/cuda_ipc.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

// Match Dora cuda-latency sizes: number of int64 elements.
static const std::vector<size_t> SIZES = {512, 5120, 51200, 512000, 5120000};
static const int SAMPLES_PER_SIZE = 100;
static const int WARMUP_SAMPLES = 10;
static const size_t ELEMENT_SIZE = 8;  // sizeof(int64_t)

static uint64_t clock_monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

#define CUDA_CHECK(call)                                                   \
  do {                                                                     \
    cudaError_t err = (call);                                              \
    if (err != cudaSuccess) {                                              \
      fprintf(stderr, "CUDA error at %s:%d: %s\n", __FILE__, __LINE__,    \
              cudaGetErrorString(err));                                     \
      std::exit(1);                                                        \
    }                                                                      \
  } while (0)

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cuda_benchmark_talker");
  auto logger = node->get_logger();

  // Determine mode.
  const char *mode_env = std::getenv("CUDA_BENCH_MODE");
  bool ipc_mode = !(mode_env && std::string(mode_env) == "cpu");

  RCLCPP_INFO(logger, "CUDA benchmark talker — mode=%s", ipc_mode ? "ipc" : "cpu");

  // Initialize CUDA context.
  CUDA_CHECK(cudaSetDevice(0));
  CUDA_CHECK(cudaFree(nullptr));  // Force lazy context init.

  // Publishers.
  auto qos = rclcpp::QoS(10).reliable();
  auto ipc_pub = node->create_publisher<cuda_interfaces::msg::CudaIpc>("cuda_ipc", qos);
  auto cpu_pub = node->create_publisher<std_msgs::msg::UInt8MultiArray>("cuda_cpu_data", qos);

  // ACK subscriber with condition variable.
  std::mutex ack_mtx;
  std::condition_variable ack_cv;
  bool ack_received = false;

  auto ack_sub = node->create_subscription<std_msgs::msg::Empty>(
      "cuda_ack", rclcpp::QoS(10).reliable(),
      [&](std_msgs::msg::Empty::SharedPtr) {
        std::lock_guard<std::mutex> lock(ack_mtx);
        ack_received = true;
        ack_cv.notify_one();
      });

  // Spin in background thread.
  std::thread spin_thread([&]() { rclcpp::spin(node); });

  // Wait for subscriber discovery.
  std::this_thread::sleep_for(std::chrono::seconds(4));

  // RNG for filling buffers.
  std::mt19937_64 rng(42);

  for (size_t size : SIZES) {
    size_t byte_size = size * ELEMENT_SIZE;

    // Pre-generate host data.
    std::vector<int64_t> host_data(size);
    for (size_t j = 0; j < size; ++j) {
      host_data[j] = static_cast<int64_t>(rng());
    }

    RCLCPP_INFO(logger, "size=%zuB (%zu elements), warmup=%d, samples=%d",
                byte_size, size, WARMUP_SAMPLES, SAMPLES_PER_SIZE);

    // Allocate GPU buffer once per size bracket to avoid CUDA VA reuse
    // conflicts with IPC handle mapping on the receiver side.
    void *d_ptr = nullptr;
    CUDA_CHECK(cudaMalloc(&d_ptr, byte_size));

    int total = WARMUP_SAMPLES + SAMPLES_PER_SIZE;
    for (int i = 0; i < total; ++i) {
      // Re-fill buffer each iteration (like Dora benchmark).
      CUDA_CHECK(cudaMemcpy(d_ptr, host_data.data(), byte_size,
                             cudaMemcpyHostToDevice));
      CUDA_CHECK(cudaDeviceSynchronize());

      if (ipc_mode) {
        // Get IPC handle (valid for this allocation's lifetime).
        cudaIpcMemHandle_t handle;
        CUDA_CHECK(cudaIpcGetMemHandle(&handle, d_ptr));

        // Build message.
        cuda_interfaces::msg::CudaIpc msg;
        std::memcpy(msg.ipc_handle.data(), &handle, 64);
        msg.timestamp_ns = clock_monotonic_ns();
        msg.num_elements = size;
        msg.element_size = ELEMENT_SIZE;
        msg.mode = 0;

        ipc_pub->publish(msg);
      } else {
        // CPU fallback: copy device -> host -> publish as bytes.
        std::vector<uint8_t> bytes(byte_size);
        CUDA_CHECK(cudaMemcpy(bytes.data(), d_ptr, byte_size,
                               cudaMemcpyDeviceToHost));

        // Metadata message.
        cuda_interfaces::msg::CudaIpc meta;
        std::memset(meta.ipc_handle.data(), 0, 64);
        meta.timestamp_ns = clock_monotonic_ns();
        meta.num_elements = size;
        meta.element_size = ELEMENT_SIZE;
        meta.mode = 1;
        ipc_pub->publish(meta);

        // Data message.
        std_msgs::msg::UInt8MultiArray data_msg;
        data_msg.data = std::move(bytes);
        cpu_pub->publish(data_msg);
      }

      // Wait for ACK.
      {
        std::unique_lock<std::mutex> lock(ack_mtx);
        ack_cv.wait_for(lock, std::chrono::seconds(5),
                        [&]() { return ack_received; });
        if (!ack_received) {
          RCLCPP_WARN(logger, "ACK timeout at size=%zuB sample=%d", byte_size, i);
        }
        ack_received = false;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Free GPU buffer after the bracket completes.
    CUDA_CHECK(cudaFree(d_ptr));
  }

  RCLCPP_INFO(logger, "Benchmark complete.");
  std::this_thread::sleep_for(std::chrono::seconds(2));
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
