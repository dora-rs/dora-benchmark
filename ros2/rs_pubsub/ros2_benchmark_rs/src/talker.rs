//! ROS 2 Rust talker using rclrs — mirrors dora-rs/rs-latency/node.
//!
//! Publishes `std_msgs::msg::UInt64MultiArray` messages of varying
//! sizes on `/topic`. The first element of each payload is a
//! `CLOCK_MONOTONIC` timestamp in nanoseconds — shared across
//! processes on the same machine, matching Python's
//! `time.perf_counter_ns()` semantics.

use anyhow::{Error, Result};
use rand::Rng;
use rclrs::*;
use std::time::Duration;

/// Match dora-rs/rs-latency sizes (number of u64 elements).
const SIZES: [usize; 5] = [1, 10 * 512, 100 * 512, 1000 * 512, 10000 * 512];
const SAMPLES_PER_SIZE: usize = 1000;
const TICK: Duration = Duration::from_millis(20);

fn clock_monotonic_ns() -> u64 {
    // Same clock source as Python's time.perf_counter_ns() on Linux.
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: clock_gettime with CLOCK_MONOTONIC is always safe on Linux.
    unsafe {
        libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts);
    }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("ros2_benchmark_talker")?;

    let publisher = node.create_publisher::<std_msgs::msg::UInt64MultiArray>("topic")?;

    // Give the subscriber time to connect before we start the timed run.
    std::thread::sleep(Duration::from_secs(1));

    let mut rng = rand::thread_rng();

    for size in SIZES {
        for _ in 0..SAMPLES_PER_SIZE {
            let mut data: Vec<u64> = (0..size).map(|_| rng.r#gen()).collect();
            data[0] = clock_monotonic_ns();

            let msg = std_msgs::msg::UInt64MultiArray {
                layout: Default::default(),
                data,
            };
            publisher.publish(&msg)?;
            std::thread::sleep(TICK);
        }
    }

    // Grace period so the sink records the last bracket.
    std::thread::sleep(Duration::from_secs(3));
    Ok(())
}
