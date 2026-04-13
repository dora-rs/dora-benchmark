# Robotic Dataflow Benchmark

Latency benchmark comparing **dora-rs** and **ROS 2** across Rust, Python, and
C++ on the same payload sizes (8 B, 40 KB, 400 KB, 4 MB, 40 MB).

All results below measured on Linux x86\_64 (Intel Core Ultra 9 285K, 24 cores,
64 GB RAM, CPU governor `performance`), 1000 samples per size bracket.

## Results (p50 latency in microseconds, Linux x86\_64)

| Size | dora Rust | dora Python | ROS 2 C++ SHM | ROS 2 C++ | ROS 2 Rust (r2r) | ROS 2 Python |
|------|-----------|-------------|---------------|-----------|------------------|--------------|
| 8 B | 389 | 721 | **45** | 154 | 77 | 279 |
| 40 KB | **222** | 340 | 59 | 173 | 77 | 296 |
| 400 KB | **257** | 350 | 208 | 240 | 351 | 476 |
| 4 MB | **715** | 722 | 2261 | 2058 | 3985 | 16773 |
| 40 MB | **2246** | 2213 | — | — | — | — |

### Key findings

- **Small messages (8 B - 40 KB):** ROS 2 C++ with shared memory
  (CycloneDDS + iceoryx) is fastest at 45-59 µs. ROS 2 Rust (r2r) with
  SHM is close at 77 µs. dora Rust is 222-389 µs.
- **Medium messages (400 KB):** dora Rust wins at 257 µs. ROS 2 C++ SHM
  is 208 µs, r2r SHM is 351 µs.
- **Large messages (4 MB):** dora is **3× faster** than ROS 2 C++ SHM
  (715 µs vs 2261 µs) and **5.6× faster** than r2r SHM (3985 µs). All
  ROS 2 configurations are bottlenecked by CDR serialization.
- **40 MB:** Only dora can handle this size. All ROS 2 configurations
  (C++, Rust, Python) fail to deliver 40 MB messages — the publisher
  succeeds but the subscriber never receives them. This is a fundamental
  DDS limitation for very large variable-size messages.
- **dora Python vs Rust:** Surprisingly close (722 vs 715 µs at 4 MB) because
  the hot path (Arrow + shared memory) is Rust code in both cases.
- **rclrs vs r2r:** rclrs (ros2\_rust) has a known stack allocation issue
  ([#281](https://github.com/ros2-rust/ros2_rust/issues/281)) that causes
  stack overflow with messages > 1 MB and 29 ms latency for 4 MB with
  variable-size types. r2r handles large messages properly on the heap.

### Why ROS 2 C++ SHM is slow at 4 MB

Even with CycloneDDS + iceoryx (zero-copy transport) and loaned messages,
ROS 2 still does CDR serialization/deserialization. We confirmed:

- `publish()` with loaned messages: **12-15 us** (true zero-copy, constant time)
- Pure iceoryx (no ROS 2): **79 us** for 4 MB (constant, true zero-copy)
- Full ROS 2 path with `spin()`: **2261 us** — the CDR deserialization in the
  subscriber's `take()` call accounts for the overhead

dora avoids this by using Apache Arrow format directly in shared memory —
no serialization layer between publisher and subscriber.

## Configurations tested

| Name | Framework | Language | Middleware | Shared Memory |
|------|-----------|----------|------------|---------------|
| dora Rust | dora-rs 0.5.0 | Rust | dora daemon | Arrow + custom SHM |
| dora Python | dora-rs 0.5.0 | Python 3.12 | dora daemon | Arrow + custom SHM |
| ROS 2 C++ SHM | ROS 2 Jazzy | C++ | CycloneDDS + iceoryx | Loaned messages (fixed-size types) |
| ROS 2 C++ | ROS 2 Jazzy | C++ | FastDDS | No |
| ROS 2 Rust (r2r) | ROS 2 Jazzy | Rust (r2r 0.9) | CycloneDDS + iceoryx | Via middleware (variable-size types) |
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

### ROS 2 C++ (no shared memory)

```bash
source /opt/ros/jazzy/setup.bash
cd ros2/cpp_pubsub
colcon build
source install/setup.sh
ros2 run cpp_pubsub listener &
ros2 run cpp_pubsub talker
cat time.csv
```

### ROS 2 C++ with shared memory (CycloneDDS + iceoryx)

Requires `ros-jazzy-rmw-cyclonedds-cpp`, `ros-jazzy-iceoryx-posh`, and
`ros-jazzy-iceoryx-hoofs`.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2/cpp_shm_pubsub

# Ensure iceoryx libs are on the path
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

colcon build
source install/setup.sh

# Start iceoryx RouDi with pre-allocated memory pools
iox-roudi -c roudi_config.toml &
sleep 2

# Switch to CycloneDDS with shared memory enabled
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml

ros2 run cpp_shm_pubsub listener &
ros2 run cpp_shm_pubsub talker
cat time.csv

pkill iox-roudi
```

The talker logs `loan=1` for each size bracket when loaned messages (zero-copy
publish) are active. If you see `loan=0`, the iceoryx mempool chunks are too
small — increase sizes in `roudi_config.toml`.

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

Or without SHM (default FastDDS):

```bash
source /opt/ros/jazzy/setup.bash
cd ros2/r2r_pubsub
./target/release/r2r-benchmark-listener &
./target/release/r2r-benchmark-talker
cat time.csv
```

## CSV output schema

All benchmarks write the same CSV columns:

```
date, language, platform, name, size_bytes, avg_us, p50_us, p90_us, p99_us, n
```

Each row summarises one payload size bracket: average, p50, p90, p99 latency
(in microseconds) over `n` samples. The benchmarks use 1000 samples per size
bracket and match payload sizes (8 B, 40 KB, 400 KB, 4 MB, 40 MB).

## Repository structure

```
dora-rs/
  rs-latency/          Rust dora benchmark (node + sink)
  py-latency/          Python dora benchmark (node_1 + node_2)

ros2/
  cpp_pubsub/          C++ ROS 2 benchmark (FastDDS/CycloneDDS, no SHM)
  cpp_shm_pubsub/      C++ ROS 2 benchmark (CycloneDDS + iceoryx SHM)
    cyclonedds.xml     CycloneDDS shared memory config
    roudi_config.toml  iceoryx memory pool config
    msg/               Fixed-size message types for loaned messages
    src/talker.cpp     Publisher with loaned messages
    src/listener.cpp   Subscriber with UniquePtr callbacks
    src/iox_talker.cpp Pure iceoryx talker (no ROS 2)
    src/iox_listener.cpp Pure iceoryx listener (no ROS 2)
  py_pubsub/           Python ROS 2 benchmark (rclpy)
  r2r_pubsub/          Rust ROS 2 benchmark (r2r, recommended)
  rs_pubsub/           Rust ROS 2 benchmark (rclrs, has stack overflow > 1 MB)
    build.sh           Colcon workspace build script
    ros2_benchmark_rs/ rclrs package source
```
