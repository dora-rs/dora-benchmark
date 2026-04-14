// Dora C++ CUDA IPC benchmark receiver — non-Arrow path.
// Uses `event_as_input` to get the raw uint8 payload directly (no Arrow FFI).
// Payload layout: [timestamp_ns (i64)][num_elements (i64)][ipc_handle (64B)]

#include "dora-node-api.h"

#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

static const int WARMUP_SAMPLES = 10;

static uint64_t clock_monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

int main() {
    auto dora_node = init_dora_node();
    auto id = std::string(node_id(dora_node.send_output));

    cudaSetDevice(0);
    cudaFree(nullptr);
    std::cerr << "[" << id << "] CUDA context ready" << std::endl;

    std::vector<uint8_t> ack_payload(1, 0);

    std::map<int64_t, std::vector<uint64_t>> latencies;
    std::map<int64_t, int> warmup;

    while (true) {
        auto event = dora_node.events->next();
        auto ty = event_type(event);
        if (ty == DoraEventType::Stop || ty == DoraEventType::AllInputsClosed)
            break;
        if (ty != DoraEventType::Input)
            continue;

        DoraInput input = event_as_input(std::move(event));
        if (std::string(input.id) != "cuda_data")
            continue;

        const uint8_t *data = input.data.data();
        if (input.data.size() < 80) {
            std::cerr << "unexpected payload size: " << input.data.size() << std::endl;
            continue;
        }

        int64_t t_send;
        int64_t num_elements;
        std::memcpy(&t_send, data, sizeof(int64_t));
        std::memcpy(&num_elements, data + 8, sizeof(int64_t));

        cudaIpcMemHandle_t handle;
        std::memcpy(&handle, data + 16, 64);

        void *d_ptr = nullptr;
        cudaError_t err = cudaIpcOpenMemHandle(
            &d_ptr, handle, cudaIpcMemLazyEnablePeerAccess);
        if (err != cudaSuccess) {
            std::cerr << "cudaIpcOpenMemHandle failed: " << cudaGetErrorString(err)
                      << std::endl;
            send_output(
                dora_node.send_output, "next",
                rust::Slice<const uint8_t>(ack_payload.data(), ack_payload.size()));
            continue;
        }
        cudaDeviceSynchronize();
        uint64_t t_received = clock_monotonic_ns();
        cudaIpcCloseMemHandle(d_ptr);

        send_output(
            dora_node.send_output, "next",
            rust::Slice<const uint8_t>(ack_payload.data(), ack_payload.size()));

        int64_t size_bytes = num_elements * 8;
        auto &wc = warmup[size_bytes];
        if (wc < WARMUP_SAMPLES) {
            wc++;
            continue;
        }
        uint64_t lat_us = (t_received - static_cast<uint64_t>(t_send)) / 1000;
        latencies[size_bytes].push_back(lat_us);
    }

    const char *csv_env = std::getenv("CSV_TIME_FILE");
    std::string csv_path = csv_env ? csv_env : "time.csv";
    std::ofstream f(csv_path, std::ios::app);

    for (auto &[sz, lats] : latencies) {
        std::sort(lats.begin(), lats.end());
        size_t n = lats.size();
        if (n == 0) continue;
        uint64_t sum = 0;
        for (auto v : lats) sum += v;
        uint64_t avg = sum / n;
        uint64_t p50 = lats[n / 2];
        uint64_t p90 = lats[std::min((size_t)(n * 0.9), n - 1)];
        uint64_t p99 = lats[std::min((size_t)(n * 0.99), n - 1)];

        std::cerr << "size=" << sz << "  avg=" << avg << "us  p50=" << p50
                  << "us  p90=" << p90 << "us  p99=" << p99 << "us  n=" << n
                  << std::endl;
        f << "C++,COMPUTER_PERF,dora C++ CUDA IPC," << sz << "," << avg << ","
          << p50 << "," << p90 << "," << p99 << "," << n << "\n";
    }

    std::cerr << "[" << id << "] done" << std::endl;
    return 0;
}
