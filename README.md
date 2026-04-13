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

## CUDA GPU-to-GPU Latency (p50, microseconds)

Measures GPU-to-GPU data transfer latency using CUDA IPC handles. Only the
64-byte IPC handle traverses the messaging framework -- bulk data stays on GPU.

| Size | Dora C++ | Dora Rust | Dora Python | ROS2 C++ | ROS2 Python |
|------|----------|-----------|-------------|----------|-------------|
| 4 KB | 759 | 656 | 773 | **429** | 623 |
| 40 KB | 605 | 623 | 731 | **393** | 542 |
| 400 KB | 641 | 658 | 767 | **320** | 540 |
| 4 MB | 686 | 699 | 805 | **316** | 513 |
| 40 MB | 758 | 817 | 961 | **492** | 665 |

All benchmarks use the same raw `cudaIpcGetMemHandle`/`cudaIpcOpenMemHandle`
calls -- the difference is purely messaging framework overhead for the small
IPC handle message.

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
