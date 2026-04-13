// Dora C++ CUDA IPC benchmark receiver.
// Uses raw CUDA runtime API (cudaIpcOpenMemHandle) and dora C++ node API.

#include "dora-node-api.h"

#include <arrow/api.h>
#include <arrow/c/bridge.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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

// Arrow FFI helper from upstream example.
struct ArrowInput {
    std::shared_ptr<arrow::Array> array;
    std::string id;
    rust::cxxbridge1::Box<Metadata> metadata;
};

ArrowInput receive_arrow(rust::cxxbridge1::Box<DoraEvent> event) {
    struct ArrowArray c_array;
    struct ArrowSchema c_schema;
    auto info = event_as_arrow_input_with_info(
        std::move(event),
        reinterpret_cast<uint8_t *>(&c_array),
        reinterpret_cast<uint8_t *>(&c_schema));
    if (!info.error.empty()) {
        return {nullptr, "", std::move(info.metadata)};
    }
    auto result = arrow::ImportArray(&c_array, &c_schema);
    if (!result.ok()) {
        return {nullptr, "", std::move(info.metadata)};
    }
    return {result.MoveValueUnsafe(), std::string(info.id), std::move(info.metadata)};
}

int main() {
    auto dora_node = init_dora_node();
    auto id = std::string(node_id(dora_node.send_output));

    // Init CUDA.
    cudaSetDevice(0);
    cudaFree(nullptr);
    std::cerr << "[" << id << "] CUDA context ready" << std::endl;

    // Build empty array for ACK.
    arrow::Int8Builder builder;
    auto empty_arr = builder.Finish().ValueOrDie();

    std::map<int64_t, std::vector<uint64_t>> latencies;
    std::map<int64_t, int> warmup;

    while (true) {
        auto event = dora_node.events->next();
        auto ty = event_type(event);
        if (ty == DoraEventType::Stop || ty == DoraEventType::AllInputsClosed)
            break;
        if (ty != DoraEventType::Input)
            continue;

        auto input = receive_arrow(std::move(event));
        if (!input.array || input.id != "cuda_data")
            continue;

        int64_t t_send = input.metadata->get_int("time");
        int64_t num_elements = input.metadata->get_int("num_elements");
        int64_t size_bytes = num_elements * 8;

        // Extract 64-byte IPC handle from Int8Array.
        auto int8_array = std::static_pointer_cast<arrow::Int8Array>(input.array);
        const int8_t *raw = int8_array->raw_values();

        cudaIpcMemHandle_t handle;
        std::memcpy(&handle, raw, 64);

        void *d_ptr = nullptr;
        cudaError_t err = cudaIpcOpenMemHandle(&d_ptr, handle,
                                                cudaIpcMemLazyEnablePeerAccess);
        if (err != cudaSuccess) {
            std::cerr << "cudaIpcOpenMemHandle failed: " << cudaGetErrorString(err)
                      << std::endl;
            // Send ACK anyway.
            struct ArrowArray c_a;
            struct ArrowSchema c_s;
            arrow::ExportArray(*empty_arr, &c_a, &c_s);
            send_arrow_output(dora_node.send_output, "next",
                              reinterpret_cast<uint8_t *>(&c_a),
                              reinterpret_cast<uint8_t *>(&c_s),
                              new_metadata());
            continue;
        }
        cudaDeviceSynchronize();
        uint64_t t_received = clock_monotonic_ns();
        cudaIpcCloseMemHandle(d_ptr);

        // Send ACK.
        {
            struct ArrowArray c_a;
            struct ArrowSchema c_s;
            arrow::ExportArray(*empty_arr, &c_a, &c_s);
            send_arrow_output(dora_node.send_output, "next",
                              reinterpret_cast<uint8_t *>(&c_a),
                              reinterpret_cast<uint8_t *>(&c_s),
                              new_metadata());
        }

        // Record latency.
        auto &wc = warmup[size_bytes];
        if (wc < WARMUP_SAMPLES) {
            wc++;
            continue;
        }
        uint64_t lat_us = (t_received - static_cast<uint64_t>(t_send)) / 1000;
        latencies[size_bytes].push_back(lat_us);
    }

    // Write CSV.
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
