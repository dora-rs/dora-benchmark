# Robotic Dataflow Benchmark

![Screenshot 2025-01-11 at 11-01-37 dora-rs dora-rs](https://github.com/user-attachments/assets/3285d183-7560-40e1-ac02-30fee0f120cb) ![Screenshot 2025-01-11 at 11-01-43 dora-rs dora-rs](https://github.com/user-attachments/assets/d64370fc-b0ba-46af-be83-19d3c772ada6)


## Getting started

```bash
## If you do not have dora-rs installed
# sudo apt update && sudo apt-get install wget
cargo install dora-cli --locked
pip install dora-rs
## In development, within the dora-node-api-python folder, You can also use:
## maturin develop --release

## To test Python version of dora
cd dora-rs/py-latency
dora up

### For operators
dora start dataflow_op.yml --attach
# Ctrl + C at the end

cat benchmark_data.csv

## To test Rust version of dora
cd dora-rs/rs-latency
cargo build --release --all
dora start dataflow.yml
```

## Getting started ROS2 Python

```bash
docker run --network=host -e DISPLAY=${DISPLAY} -v $(pwd):/dora-benchmark -it osrf/ros:humble-desktop

# Within the docker container
cd dora-benchmark/ros2/py_pubsub
colcon build
. install/setup.bash
ros2 run py_pubsub listener & ros2 run py_pubsub talker
cat time.csv
```

## Getting started ROS2 Rust (rclrs)

A pure-Rust ROS 2 publisher/subscriber using the official
[`rclrs`](https://github.com/ros2-rust/ros2_rust) client library, mirroring
the dora-rs Rust benchmark for a fair Rust-vs-Rust comparison.

```bash
# Build inside a Jazzy ROS 2 container. The script clones rclrs +
# message-package dependencies into a colcon workspace and builds.
docker volume create ros2-ws
docker run --rm --platform linux/amd64 \
    -v ros2-ws:/ros2_ws \
    -v $(pwd):/dora-benchmark \
    osrf/ros:jazzy-desktop \
    bash /dora-benchmark/ros2/rs_pubsub/build.sh

# Run the benchmark
docker run --rm --platform linux/amd64 \
    -v ros2-ws:/ros2_ws \
    -v $(pwd):/dora-benchmark \
    -w /dora-benchmark/ros2/rs_pubsub \
    osrf/ros:jazzy-desktop bash -c '
        set +u
        source /opt/ros/jazzy/setup.sh
        source /ros2_ws/install/setup.sh
        rm -f time.csv
        ros2 run ros2_benchmark_rs listener > /tmp/listener.log 2>&1 &
        sleep 2
        ros2 run ros2_benchmark_rs talker
        sleep 5
        cat time.csv
    '
```

## CUDA GPU-to-GPU Latency

Measures GPU-to-GPU data transfer latency using CUDA IPC handles. Only the
64-byte IPC handle traverses the messaging framework -- bulk data stays on
GPU. All five configurations use identical raw
`cudaIpcGetMemHandle` / `cudaIpcOpenMemHandle` calls, so the differences
reflect messaging-framework overhead.

1000 samples per size, 10 warmup iterations discarded, 5 ms inter-send delay.
Dora benchmarks use the direct node-to-node TCP optimization
([dora-rs/dora#1621](https://github.com/dora-rs/dora/pull/1621)).

### p50 latency (microseconds)

| Size | Dora C++ | Dora Rust | Dora Python | ROS2 C++ | ROS2 Python |
|------|----------|-----------|-------------|----------|-------------|
| 8 B | 236 | **173** | 320 | 256 | 436 |
| 64 B | 201 | **174** | 219 | 289 | 425 |
| 512 B | 248 | **181** | 252 | 369 | 452 |
| 4 KB | **164** | 170 | 277 | 260 | 456 |
| 40 KB | 202 | **141** | 339 | 327 | 320 |
| 400 KB | 202 | 242 | 306 | **177** | 288 |
| 4 MB | 290 | 263 | 318 | **207** | 346 |
| 40 MB | 269 | **265** | 500 | 316 | 510 |

### p90 latency (microseconds)

| Size | Dora C++ | Dora Rust | Dora Python | ROS2 C++ | ROS2 Python |
|------|----------|-----------|-------------|----------|-------------|
| 8 B | 374 | **220** | 423 | 353 | 489 |
| 64 B | 351 | **212** | 307 | 411 | 499 |
| 512 B | 305 | **306** | 325 | 413 | 533 |
| 4 KB | 280 | **226** | 402 | 363 | 515 |
| 40 KB | 362 | **211** | 436 | 398 | 411 |
| 400 KB | 303 | 353 | 365 | **252** | 339 |
| 4 MB | 371 | 399 | 401 | **276** | 395 |
| 40 MB | 310 | **308** | 648 | 516 | 692 |

### p99 latency (microseconds)

| Size | Dora C++ | Dora Rust | Dora Python | ROS2 C++ | ROS2 Python |
|------|----------|-----------|-------------|----------|-------------|
| 8 B | 458 | **280** | 489 | 416 | 534 |
| 64 B | 439 | **260** | 358 | 468 | 582 |
| 512 B | 387 | **376** | 385 | 457 | 601 |
| 4 KB | 347 | **324** | 461 | 421 | 580 |
| 40 KB | 435 | **328** | 484 | 460 | 493 |
| 400 KB | 388 | 434 | 415 | **320** | 383 |
| 4 MB | 470 | 459 | 491 | **343** | 454 |
| 40 MB | 372 | **384** | 725 | 583 | 762 |

### Framework comparison — range analysis

Across all 8 size brackets (p50):

| Framework | min | max | mean |
|-----------|-----|-----|------|
| Dora Rust | **141** | **265** | **201** |
| Dora C++ | 164 | 290 | 226 |
| ROS2 C++ | 177 | 369 | 275 |
| Dora Python | 219 | 500 | 316 |
| ROS2 Python | 288 | 510 | 404 |

**Dora Rust is consistently fastest** for GPU-to-GPU CUDA IPC (mean 201 µs,
25–35% faster than ROS2 C++). Dora C++ comes second. Dora Python beats ROS2
Python by ~90 µs on average.

Range overlap: **4 of 5 frameworks** (everything except ROS2 Python) overlap
in the [219–265] µs window. ROS2 Python is the only outlier on the slow
side — its minimum (288 µs) is already above Dora Rust's maximum (265 µs).

### CPU benchmark (same machine, for reference)

Same machine, same date, but moving actual bulk data through the framework
instead of just a CUDA IPC handle. This is where framework choice *actually*
matters. Values below are average latency in µs per message.

| Size | Dora Rust | Dora Python | ROS2 C++ | ROS2 Python |
|------|-----------|-------------|----------|-------------|
| 8 B | 220 | — | 277 | — |
| 64 B | — | 330 | — | 423 |
| 512 B | — | 269 | — | 441 |
| 4 KB | — | 516 | — | 457 |
| 40 KB | 529 | 500 | **319** | 432 |
| 400 KB | 390 | 603 | **385** | 1134 |
| 4 MB | **800** | 957 | 11724 | 26256 |
| 40 MB | **2505** | 3177 | did not finish | 119902 |

Compare to the CUDA IPC table above: at 40 MB, **Dora Rust CPU is 2505 µs vs
Dora Rust CUDA IPC at 265 µs** — roughly 10× slower when the actual data
travels. Meanwhile ROS2 Python at 40 MB is 119,902 µs (~120 ms!) on the CPU
path — FastDDS with the default CDR serialization hits a wall on large
variable-size arrays. ROS2 C++ couldn't keep up with the default publish
rate at 40 MB.

**Takeaway:** framework differences are dramatic for bulk data on CPU
(1 − 50× between Dora and ROS2 at large sizes) and negligible for GPU-to-GPU
via CUDA IPC (everyone's within ~100 µs). Use CUDA IPC when you can.

### Notes

- **Dora uses Unix domain sockets** for local IPC
  ([dora-rs/dora#1622](https://github.com/dora-rs/dora/pulls?q=unix-domain-sockets)).
- **Dora benchmarks use `dora run <yaml>`** (fresh coordinator/daemon per run)
  instead of long-lived `dora up`. We observed that leftover state in a
  persistent daemon systematically slowed Dora Rust by ~100 µs per size
  bracket. Always use `dora run` for benchmarks.
- **Dora C++ receiver uses `event_as_input`** (raw uint8 payload) instead of
  Arrow C-Data Interface to avoid FFI overhead. Sender packs
  `[timestamp][num_elements][ipc_handle]` into a single 80-byte uint8 array.
- **ROS2 C++ p50 is lowest at 400KB–4MB** — FastDDS has very low per-message
  overhead on the steady-state path, but it has higher cold-start jitter at
  tiny sizes.
- **Noise is real** — single-digit percent variance between runs; exact p50
  ordering shifts between runs, especially at the 8B–512B boundary.

## Getting started Dora CUDA GPU-to-GPU (Python)

```bash
pip install dora-rs torch numpy tqdm
cd dora-rs/cuda-latency
dora up
dora start cuda_bench.yml --attach
cat benchmark_data.csv
```

## Getting started Dora CUDA GPU-to-GPU (Rust)

```bash
cd dora-rs/cuda-latency-rust/receiver
cargo build --release
cd ..
dora up
dora start dataflow.yml --attach
cat time.csv
```

## Getting started Dora CUDA GPU-to-GPU (C++)

Uses prebuilt dora C++ libraries downloaded automatically from GitHub releases.

Requires: Arrow C++ (`libarrow-dev`), CUDA toolkit, CMake 3.17+.

```bash
cd dora-rs/cuda-latency-cpp
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ..
dora up
dora start dataflow.yml --attach
cat time.csv
```

## Getting started ROS2 CUDA GPU-to-GPU (IPC)

Measures GPU-to-GPU latency using CUDA IPC handles over ROS2, comparable to
the Dora CUDA benchmark (`dora-rs/cuda-latency/`). Only the 64-byte IPC handle
traverses ROS2 -- the bulk data stays on the GPU.

Requires: ROS2 Jazzy, NVIDIA GPU, CUDA toolkit.

```bash
cd ros2

# Build all three packages
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cuda_interfaces cuda_cpp_pubsub cuda_py_pubsub
source install/setup.sh

# --- C++ CUDA IPC benchmark ---
install/cuda_cpp_pubsub/lib/cuda_cpp_pubsub/cuda_listener &
install/cuda_cpp_pubsub/lib/cuda_cpp_pubsub/cuda_talker
# Ctrl-C the listener after the talker finishes
cat time.csv

# --- Python CUDA IPC benchmark ---
install/cuda_py_pubsub/lib/cuda_py_pubsub/cuda_listener &
install/cuda_py_pubsub/lib/cuda_py_pubsub/cuda_talker
# Ctrl-C the listener after the talker finishes
cat time.csv

# --- CPU fallback baseline (data goes GPU→CPU→ROS2→CPU→GPU) ---
CUDA_BENCH_MODE=cpu install/cuda_cpp_pubsub/lib/cuda_cpp_pubsub/cuda_listener &
CUDA_BENCH_MODE=cpu install/cuda_cpp_pubsub/lib/cuda_cpp_pubsub/cuda_talker
```

Sizes match the Dora CUDA benchmark: 512, 5120, 51200, 512000, 5120000
int64 elements (4 KB -- 40 MB). 100 samples per size, 10 warmup iterations.

## CSV output schema

Both the dora-rs Rust benchmark (`dora-rs/rs-latency/timer.csv`) and the ROS 2
Python benchmark (`ros2/py_pubsub/time.csv`) share the same columns so the
results can be compared directly:

```
date, language, platform, name, size_bytes, avg_us, p50_us, p90_us, p99_us, n
```

Each row summarises one payload size bracket: average, p50, p90, p99 latency
(in microseconds) over `n` samples. The benchmarks use 1000 samples per size
and match payload sizes (`8 B`, `40 KB`, `400 KB`, `4 MB`, `40 MB`) for a
direct comparison.
