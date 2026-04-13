// ROS 2 C++ CUDA IPC listener — GPU-to-GPU latency benchmark.
// Receives CUDA IPC handles, opens GPU memory, measures latency.
// Supports IPC mode (default) and CPU fallback mode (CUDA_BENCH_MODE=cpu).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <cuda_runtime_api.h>

#include "rclcpp/rclcpp.hpp"
#include "cuda_interfaces/msg/cuda_ipc.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

static const int WARMUP_SAMPLES = 10;

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

static void record_results(const std::string &date, const std::string &name,
                           size_t size_bytes,
                           std::vector<uint64_t> &latencies_us) {
  std::sort(latencies_us.begin(), latencies_us.end());
  uint64_t n = latencies_us.size();
  if (n == 0) return;
  uint64_t sum = 0;
  for (auto v : latencies_us) sum += v;
  uint64_t avg = sum / n;
  uint64_t p50 = percentile_us(latencies_us, 50.0);
  uint64_t p90 = percentile_us(latencies_us, 90.0);
  uint64_t p99 = percentile_us(latencies_us, 99.0);

  const char *csv_env = std::getenv("CSV_TIME_FILE");
  std::string csv_path = csv_env ? csv_env : "time.csv";

  std::ofstream f(csv_path, std::ios::app);
  f << date << ",C++,COMPUTER_PERF," << name << "," << size_bytes
    << "," << avg << "," << p50 << "," << p90 << "," << p99 << "," << n
    << "\n";

  std::cout << "size=" << size_bytes << "  avg=" << avg << "us  p50=" << p50
            << "us  p90=" << p90 << "us  p99=" << p99 << "us  n=" << n
            << std::endl;
}

#define CUDA_CHECK(call)                                                   \
  do {                                                                     \
    cudaError_t err = (call);                                              \
    if (err != cudaSuccess) {                                              \
      fprintf(stderr, "CUDA error at %s:%d: %s\n", __FILE__, __LINE__,    \
              cudaGetErrorString(err));                                     \
    }                                                                      \
  } while (0)

class CudaListener : public rclcpp::Node {
public:
  CudaListener() : Node("cuda_benchmark_listener") {
    // Determine mode.
    const char *mode_env = std::getenv("CUDA_BENCH_MODE");
    bool is_cpu = mode_env && std::string(mode_env) == "cpu";
    name_ = is_cpu ? "ROS 2 C++ CUDA CPU" : "ROS 2 C++ CUDA IPC";

    const char *name_env = std::getenv("NAME");
    if (name_env) name_ = name_env;

    // Timestamp for CSV.
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    date_ = buf;

    // Initialize CUDA context with a real allocation.
    CUDA_CHECK(cudaSetDevice(0));
    void *test_ptr = nullptr;
    CUDA_CHECK(cudaMalloc(&test_ptr, 256));
    if (test_ptr) CUDA_CHECK(cudaFree(test_ptr));

    auto qos = rclcpp::QoS(10).reliable();
    ack_pub_ = create_publisher<std_msgs::msg::Empty>("cuda_ack", qos);

    ipc_sub_ = create_subscription<cuda_interfaces::msg::CudaIpc>(
        "cuda_ipc", qos,
        [this](cuda_interfaces::msg::CudaIpc::SharedPtr msg) {
          on_ipc_msg(msg);
        });

    cpu_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "cuda_cpu_data", qos,
        [this](std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
          pending_cpu_data_ = msg;
        });

    RCLCPP_INFO(get_logger(), "CUDA benchmark listener ready — name=%s",
                name_.c_str());
  }

  ~CudaListener() { flush_all(); }

  void flush_all() {
    if (flushed_) return;
    flushed_ = true;
    std::vector<size_t> sizes;
    for (auto &kv : latencies_) sizes.push_back(kv.first);
    std::sort(sizes.begin(), sizes.end());
    for (auto sz : sizes) {
      auto &lat = latencies_[sz];
      if (!lat.empty())
        record_results(date_, name_, sz, lat);
    }
  }

private:
  void on_ipc_msg(cuda_interfaces::msg::CudaIpc::SharedPtr msg) {
    uint64_t t_send = msg->timestamp_ns;
    size_t num_elements = msg->num_elements;
    size_t elem_size = msg->element_size;
    size_t size_bytes = num_elements * elem_size;

    if (msg->mode == 0) {
      // CUDA IPC mode.
      cudaIpcMemHandle_t handle;
      std::memcpy(&handle, msg->ipc_handle.data(), 64);

      void *d_ptr = nullptr;
      cudaError_t ipc_err = cudaIpcOpenMemHandle(&d_ptr, handle,
                                       cudaIpcMemLazyEnablePeerAccess);
      if (ipc_err != cudaSuccess) {
        // If the same handle is already mapped (sender reuses allocation),
        // the driver returns the existing mapping. If it truly fails,
        // skip this sample.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "cudaIpcOpenMemHandle failed: %s", cudaGetErrorString(ipc_err));
        ack_pub_->publish(std_msgs::msg::Empty());
        return;
      }
      CUDA_CHECK(cudaDeviceSynchronize());

      uint64_t t_received = clock_monotonic_ns();

      CUDA_CHECK(cudaIpcCloseMemHandle(d_ptr));

      // Publish ACK (after close, so sender can safely re-export handle).
      ack_pub_->publish(std_msgs::msg::Empty());

      record_latency(size_bytes, t_send, t_received);
    } else {
      // CPU fallback mode.
      if (!pending_cpu_data_) {
        pending_cpu_meta_ = msg;
        return;
      }

      auto &data = pending_cpu_data_->data;

      void *d_ptr = nullptr;
      CUDA_CHECK(cudaMalloc(&d_ptr, data.size()));
      CUDA_CHECK(cudaMemcpy(d_ptr, data.data(), data.size(),
                             cudaMemcpyHostToDevice));
      CUDA_CHECK(cudaDeviceSynchronize());

      uint64_t t_received = clock_monotonic_ns();

      CUDA_CHECK(cudaFree(d_ptr));
      pending_cpu_data_.reset();

      ack_pub_->publish(std_msgs::msg::Empty());

      record_latency(size_bytes, t_send, t_received);
    }
  }

  void record_latency(size_t size_bytes, uint64_t t_send,
                       uint64_t t_received) {
    uint64_t latency_us = (t_received - t_send) / 1000;

    auto &warmup = warmup_count_[size_bytes];
    if (warmup < WARMUP_SAMPLES) {
      ++warmup;
      return;
    }

    latencies_[size_bytes].push_back(latency_us);
  }

  bool flushed_ = false;
  std::string date_;
  std::string name_;
  std::map<size_t, std::vector<uint64_t>> latencies_;
  std::map<size_t, int> warmup_count_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ack_pub_;
  rclcpp::Subscription<cuda_interfaces::msg::CudaIpc>::SharedPtr ipc_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr cpu_sub_;

  std_msgs::msg::UInt8MultiArray::SharedPtr pending_cpu_data_;
  cuda_interfaces::msg::CudaIpc::SharedPtr pending_cpu_meta_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto listener = std::make_shared<CudaListener>();
  rclcpp::spin(listener);
  listener->flush_all();
  rclcpp::shutdown();
  return 0;
}
