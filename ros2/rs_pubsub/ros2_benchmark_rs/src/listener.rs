//! ROS 2 Rust listener using rclrs — mirrors dora-rs/rs-latency/sink.
//!
//! Subscribes to `/topic`, measures latency per size bracket, and
//! writes results to `time.csv` with the same schema as
//! `dora-rs/rs-latency/timer.csv`.

use anyhow::{Error, Result};
use csv::Writer;
use rclrs::*;
use std::time::Duration;

const LANGUAGE: &str = "Rust";
const NAME_DEFAULT: &str = "ROS 2 Rust (rclrs)";
const PLATFORM: &str = "COMPUTER_PERF";

fn clock_monotonic_ns() -> u64 {
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

struct State {
    current_size: usize,
    latencies: Vec<Duration>,
    date: String,
}

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("ros2_benchmark_listener")?;

    let worker = node.create_worker::<State>(State {
        current_size: 0,
        latencies: Vec::new(),
        date: epoch_string(),
    });

    let _subscription = worker.create_subscription::<std_msgs::msg::UInt64MultiArray, _>(
        "topic",
        move |state: &mut State, msg: std_msgs::msg::UInt64MultiArray| {
            let t_received = clock_monotonic_ns();
            let size_bytes = msg.data.len() * 8;

            if size_bytes != state.current_size {
                if !state.latencies.is_empty() {
                    record_results(state.current_size, &mut state.latencies, &state.date);
                }
                state.current_size = size_bytes;
                state.latencies.clear();
            }

            let t_send = msg.data[0];
            let latency = Duration::from_nanos(t_received.saturating_sub(t_send));
            state.latencies.push(latency);
        },
    )?;

    println!("Waiting for messages...");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

fn epoch_string() -> String {
    use std::time::SystemTime;
    let secs = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    format!("epoch+{secs}")
}

fn percentile(sorted: &[Duration], pct: f64) -> Duration {
    if sorted.is_empty() {
        return Duration::ZERO;
    }
    let idx = ((pct / 100.0) * (sorted.len() - 1) as f64).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

fn record_results(size_bytes: usize, latencies: &mut Vec<Duration>, date: &str) {
    latencies.sort();
    let n = latencies.len() as u32;
    let avg = latencies.iter().sum::<Duration>() / n;
    let p50 = percentile(latencies, 50.0);
    let p90 = percentile(latencies, 90.0);
    let p99 = percentile(latencies, 99.0);

    let path = std::env::var("CSV_TIME_FILE").unwrap_or_else(|_| "time.csv".to_string());
    let file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .expect("open csv");
    let mut wtr = Writer::from_writer(file);

    let name = std::env::var("NAME").unwrap_or_else(|_| NAME_DEFAULT.to_string());
    wtr.write_record([
        date.to_string(),
        LANGUAGE.to_string(),
        PLATFORM.to_string(),
        name,
        size_bytes.to_string(),
        avg.as_micros().to_string(),
        p50.as_micros().to_string(),
        p90.as_micros().to_string(),
        p99.as_micros().to_string(),
        n.to_string(),
    ])
    .expect("write csv row");

    println!(
        "size={size_bytes}  avg={}µs  p50={}µs  p90={}µs  p99={}µs  n={n}",
        avg.as_micros(),
        p50.as_micros(),
        p90.as_micros(),
        p99.as_micros()
    );
}
