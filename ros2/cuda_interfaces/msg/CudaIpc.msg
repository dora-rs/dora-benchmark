# CUDA IPC memory handle (always exactly 64 bytes)
uint8[64] ipc_handle

# Sender timestamp in nanoseconds (CLOCK_MONOTONIC / perf_counter_ns)
uint64 timestamp_ns

# Number of elements in the GPU buffer
uint64 num_elements

# Size of each element in bytes (8 for int64)
uint8 element_size

# 0 = CUDA IPC mode, 1 = CPU fallback mode
uint8 mode
