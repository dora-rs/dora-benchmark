# Robotic Dataflow Benchmark

Latency benchmark comparing **dora-rs** and **ROS 2** across Rust, Python, and
C++ on 10 payload sizes from 8 B to 4 MB.

All results below measured on Linux x86\_64 (Intel Core Ultra 9 285K, 24 cores,
64 GB RAM, CPU governor `performance`), 1000 samples per size bracket.

## Results (p50 latency in microseconds, Linux x86\_64)

| Size | dora Rust | dora Python | ROS 2 C++ | ROS 2 Rust (r2r) | ROS 2 Python |
|------|-----------|-------------|-----------|------------------|--------------|
| 8 B | 358 | 523 | 166 | **73** | 366 |
| 1 KB | 339 | 576 | 167 | **73** | 286 |
| 10 KB | 445 | 677 | 179 | **67** | 292 |
| 100 KB | 321 | 544 | **183** | 86 | 318 |
| 500 KB | **268** | 359 | 373 | 375 | 686 |
| 1 MB | **367** | 439 | 5795 | 795 | 1445 |
| 1.5 MB | **293** | 659 | 6364 | 1213 | 2303 |
| 2 MB | **339** | 723 | 6744 | 1658 | 3842 |
| 3 MB | **404** | 709 | 7672 | 2611 | 8203 |
| 4 MB | **514** | 851 | 3095 | 3700 | — |

ROS 2 Rust (r2r) uses CycloneDDS + iceoryx shared memory.
ROS 2 C++ and Python use FastDDS (default middleware).

### Key findings

- **Small messages (8 B - 100 KB):** ROS 2 Rust (r2r) with CycloneDDS + iceoryx
  SHM is fastest at 67-86 µs — constant regardless of size. ROS 2 C++ with
  FastDDS is 166-183 µs. dora Rust is 321-445 µs.
- **500 KB:** dora Rust starts winning at 268 µs. ROS 2 r2r and C++ are tied
  at ~375 µs as CDR serialization begins to dominate.
- **1 MB+:** dora is **dramatically faster**. At 2 MB: dora Rust 339 µs vs
  ROS 2 C++ 6744 µs (**20× faster**) and r2r 1658 µs (**5× faster**).
- **Scaling:** dora latency grows slowly (358 → 514 µs from 8 B to 4 MB)
  because shared memory transfer is nearly free. ROS 2 latency grows
  linearly with size due to CDR serialization (~0.8 µs/KB for r2r,
  ~1.5 µs/KB for C++ FastDDS).
- **dora Python vs Rust:** Python is 1.5-2× slower than Rust within dora,
  but still beats all ROS 2 options at 1 MB+ because the hot path
  (Arrow + shared memory) is the same Rust code.
- **rclrs vs r2r:** rclrs (ros2\_rust) has a known stack allocation issue
  ([#281](https://github.com/ros2-rust/ros2_rust/issues/281)) that causes
  stack overflow with messages > 1 MB. r2r handles large messages properly.

### Why ROS 2 is slow for large messages

Even with CycloneDDS + iceoryx (zero-copy transport), ROS 2 still does CDR
serialization/deserialization. We confirmed:

- `publish()` with loaned messages: **12-15 µs** (true zero-copy, constant)
- Pure iceoryx (no ROS 2): **79 µs** for 4 MB (constant, true zero-copy)
- Full ROS 2 path with `spin()`: **2-4 ms** for 4 MB — CDR deserialization
  in the subscriber's `take()` call accounts for the overhead

dora avoids this by using Apache Arrow format directly in shared memory —
no serialization layer between publisher and subscriber.

## Configurations tested

| Name | Framework | Language | Middleware | Shared Memory |
|------|-----------|----------|------------|---------------|
| dora Rust | dora-rs 0.5.0 | Rust | dora daemon | Arrow + custom SHM |
| dora Python | dora-rs 0.5.0 | Python 3.12 | dora daemon | Arrow + custom SHM |
| ROS 2 C++ | ROS 2 Jazzy | C++ | FastDDS | No |
| ROS 2 Rust (r2r) | ROS 2 Jazzy | Rust (r2r 0.9) | CycloneDDS + iceoryx | Via middleware |
| ROS 2 Python | ROS 2 Jazzy | Python 3.12 (rclpy) | FastDDS | No |

---

## Running the benchmarks

### dora-rs Rust

```bash
cargo install dora-cli --locked
cd dora-rs/rs-latency
cargo build --release --all
dora up
dora start dataflow.yml --attach
cat timer.csv
dora destroy
```

### dora-rs Python

```bash
pip install dora-rs
cd dora-rs/py-latency
dora up
dora start dataflow_node.yml --attach
cat benchmark_data.csv
dora destroy
```

### ROS 2 C++

```bash
source /opt/ros/jazzy/setup.bash
cd ros2/cpp_pubsub
colcon build
source install/setup.sh
ros2 run cpp_pubsub listener &
ros2 run cpp_pubsub talker
cat time.csv
```

### ROS 2 Rust (r2r)

Uses the [`r2r`](https://crates.io/crates/r2r) crate — a standalone Rust
ROS 2 client that builds with plain `cargo` (no colcon needed). Handles
large messages on the heap, unlike `rclrs` which has a stack overflow issue
([ros2-rust/ros2\_rust#281](https://github.com/ros2-rust/ros2_rust/issues/281)).

```bash
source /opt/ros/jazzy/setup.bash
cd ros2/r2r_pubsub
cargo build --release

# With CycloneDDS + iceoryx SHM:
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/../cpp_shm_pubsub/cyclonedds.xml
iox-roudi -c ../cpp_shm_pubsub/roudi_config.toml &
sleep 2

./target/release/r2r-benchmark-listener &
./target/release/r2r-benchmark-talker
cat time.csv

pkill iox-roudi
```

### ROS 2 Python

```bash
source /opt/ros/jazzy/setup.bash
cd ros2/py_pubsub
colcon build
source install/setup.sh
ros2 run py_pubsub listener &
ros2 run py_pubsub talker
cat time.csv
```

### ROS 2 C++ with shared memory (CycloneDDS + iceoryx)

Requires `ros-jazzy-rmw-cyclonedds-cpp`, `ros-jazzy-iceoryx-posh`, and
`ros-jazzy-iceoryx-hoofs`.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2/cpp_shm_pubsub

export LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

colcon build
source install/setup.sh

iox-roudi -c roudi_config.toml &
sleep 2

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml

ros2 run cpp_shm_pubsub listener &
ros2 run cpp_shm_pubsub talker
cat time.csv

pkill iox-roudi
```

## CSV output schema

All benchmarks write the same CSV columns:

```
date, language, platform, name, size_bytes, avg_us, p50_us, p90_us, p99_us, n
```

## Repository structure

```
dora-rs/
  rs-latency/          Rust dora benchmark (node + sink)
  py-latency/          Python dora benchmark (node_1 + node_2)

ros2/
  cpp_pubsub/          C++ ROS 2 benchmark (FastDDS, no SHM)
  cpp_shm_pubsub/      C++ ROS 2 benchmark (CycloneDDS + iceoryx SHM)
  r2r_pubsub/          Rust ROS 2 benchmark (r2r, recommended)
  py_pubsub/           Python ROS 2 benchmark (rclpy)
  rs_pubsub/           Rust ROS 2 benchmark (rclrs, has stack overflow > 1 MB)
```
