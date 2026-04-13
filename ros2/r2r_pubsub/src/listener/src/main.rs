//! ROS 2 Rust listener using r2r — measures latency with CLOCK_MONOTONIC.

use csv::Writer;
use futures::StreamExt;
use r2r::std_msgs::msg::UInt64MultiArray;
use r2r::QosProfile;
use std::time::Duration;

fn clock_monotonic_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts); }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
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
    let name = std::env::var("NAME").unwrap_or_else(|_| "ROS 2 Rust (r2r)".to_string());
    wtr.write_record([
        date, "Rust", "COMPUTER_PERF", &name,
        &size_bytes.to_string(), &avg.to_string(), &p50.to_string(),
        &p90.to_string(), &p99.to_string(), &n.to_string(),
    ]).expect("write csv");

    eprintln!("size={size_bytes}  avg={avg}us  p50={p50}us  p90={p90}us  p99={p99}us  n={n}");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "r2r_benchmark_listener", "")?;
    let sub = node.subscribe::<UInt64MultiArray>("/benchmark_topic", QosProfile::default())?;

    let date = {
        let secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs()).unwrap_or(0);
        format!("epoch+{secs}")
    };

    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;

    let local = tokio::task::LocalSet::new();

    let last_msg = std::sync::Arc::new(std::sync::Mutex::new(None::<std::time::Instant>));
    let last_msg2 = last_msg.clone();

    local.spawn_local(async move {
        let mut sub = sub;
        let mut current_size: usize = 0;
        let mut latencies: Vec<u64> = Vec::new();
        let mut warmup = 0;

        while let Some(msg) = sub.next().await {
            *last_msg2.lock().unwrap() = Some(std::time::Instant::now());
            let t_received = clock_monotonic_ns();
            let size_bytes = msg.data.len() * 8;

            if size_bytes != current_size {
                if latencies.len() >= 100 {
                    record_results(current_size, &mut latencies, &date);
                }
                current_size = size_bytes;
                latencies.clear();
                warmup = 0;
            }

            if warmup < 10 { warmup += 1; continue; }

            let t_send = msg.data[0];
            let latency_us = (t_received.saturating_sub(t_send)) / 1000;
            latencies.push(latency_us);

            if latencies.len() == 1000 {
                record_results(current_size, &mut latencies, &date);
            }
        }
    });

    rt.block_on(local.run_until(async {
        loop {
            node.spin_once(Duration::from_millis(50));
            tokio::task::yield_now().await;

            if let Some(last) = *last_msg.lock().unwrap() {
                if last.elapsed() > Duration::from_secs(30) {
                    break;
                }
            }
        }
    }));

    Ok(())
}
