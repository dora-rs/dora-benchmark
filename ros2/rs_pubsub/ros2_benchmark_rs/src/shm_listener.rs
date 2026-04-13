//! ROS 2 Rust zero-copy listener using rclrs with CycloneDDS + iceoryx.
//! Runs executor on a thread with large stack to handle fixed-size messages.

use anyhow::Result;
use csv::Writer;
use rclrs::*;
use std::collections::BTreeMap;
use std::time::Duration;

fn clock_monotonic_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts); }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

struct State {
    latencies: BTreeMap<usize, Vec<u64>>,
    warmup: BTreeMap<usize, usize>,
    date: String,
}

fn percentile(sorted: &[u64], pct: f64) -> u64 {
    if sorted.is_empty() { return 0; }
    let idx = ((pct / 100.0) * (sorted.len() - 1) as f64).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

fn record_results(size_bytes: usize, latencies: &mut Vec<u64>, date: &str) {
    latencies.sort();
    let n = latencies.len() as u64;
    let sum: u64 = latencies.iter().sum();
    let avg = sum / n;
    let p50 = percentile(latencies, 50.0);
    let p90 = percentile(latencies, 90.0);
    let p99 = percentile(latencies, 99.0);

    let path = std::env::var("CSV_TIME_FILE").unwrap_or_else(|_| "time.csv".to_string());
    let file = std::fs::OpenOptions::new()
        .create(true).append(true).open(&path).expect("open csv");
    let mut wtr = Writer::from_writer(file);
    let name = std::env::var("NAME").unwrap_or_else(|_| "ROS 2 Rust SHM (rclrs)".to_string());
    wtr.write_record([
        date, "Rust", "COMPUTER_PERF", &name,
        &size_bytes.to_string(), &avg.to_string(), &p50.to_string(),
        &p90.to_string(), &p99.to_string(), &n.to_string(),
    ]).expect("write csv");

    eprintln!("size={size_bytes}  avg={avg}us  p50={p50}us  p90={p90}us  p99={p99}us  n={n}");
}

fn on_msg(state: &mut State, t_send: u64, size_bytes: usize) {
    let t_received = clock_monotonic_ns();
    let latency_us = (t_received.saturating_sub(t_send)) / 1000;

    let warmup = state.warmup.entry(size_bytes).or_insert(0);
    if *warmup < 10 { *warmup += 1; return; }

    let lat = state.latencies.entry(size_bytes).or_default();
    lat.push(latency_us);
    if lat.len() == 1000 {
        record_results(size_bytes, lat, &state.date);
    }
}

fn main() -> Result<()> {
    // Spawn executor on a thread with 64 MB stack to handle large fixed-size messages.
    let handle = std::thread::Builder::new()
        .stack_size(64 * 1024 * 1024)
        .spawn(|| -> Result<()> {
            let context = Context::default_from_env()?;
            let mut executor = context.create_basic_executor();
            let node = executor.create_node("rclrs_shm_listener")?;

            let date = {
                let secs = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_secs()).unwrap_or(0);
                format!("epoch+{secs}")
            };

            let worker = node.create_worker::<State>(State {
                latencies: BTreeMap::new(),
                warmup: BTreeMap::new(),
                date,
            });

            let _sub8 = worker.create_subscription::<cpp_shm_pubsub::msg::Bench8, _>(
                "bench8",
                |state: &mut State, msg: cpp_shm_pubsub::msg::Bench8| {
                    on_msg(state, msg.data[0], 8);
                },
            )?;
            let _sub40k = worker.create_subscription::<cpp_shm_pubsub::msg::Bench40K, _>(
                "bench40k",
                |state: &mut State, msg: cpp_shm_pubsub::msg::Bench40K| {
                    on_msg(state, msg.data[0], 40960);
                },
            )?;
            let _sub400k = worker.create_subscription::<cpp_shm_pubsub::msg::Bench400K, _>(
                "bench400k",
                |state: &mut State, msg: cpp_shm_pubsub::msg::Bench400K| {
                    on_msg(state, msg.data[0], 409600);
                },
            )?;
            let _sub4m = worker.create_subscription::<cpp_shm_pubsub::msg::Bench4M, _>(
                "bench4m",
                |state: &mut State, msg: cpp_shm_pubsub::msg::Bench4M| {
                    on_msg(state, msg.data[0], 4096000);
                },
            )?;

            eprintln!("Waiting for messages...");
            executor.spin(SpinOptions::default()).first_error()?;
            Ok(())
        })?;

    handle.join().unwrap()?;
    Ok(())
}
