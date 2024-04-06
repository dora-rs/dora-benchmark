# Robotic Dataflow Benchmark

## Getting started

```bash
## If you do not have dora-rs installed
sudo wget https://github.com/dora-rs/dora/releases/download/v0.3.0/dora-v0.3.0-x86_64-Linux.zip && sudo unzip dora-v0.3.0-x86_64-Linux.zip -d ~/.local/bin
pip install dora-rs==0.3.0

## In development, You can also use:
## maturin develop --release

## To test Python version of dora
cd dora-rs/py-latency
dora up

### For node
dora start dataflow_node.yml --attach

### For operators
dora start dataflow_op.yml --attach

## To test Rust version of dora
## This requires that dora-rs is placed at: "../../../dora/apis/rust/node"
cd dora-rs/rs-latency
cargo build --release --all
dora start dataflow.yml
```

## Getting started ROS2

```bash
cd dora-benchmark/ros2/py_pubsub
colcon build
. install/setup.bash
ros2 run py_pubsub listener & ros2 run py_pubsub talker
cat time.csv
```
