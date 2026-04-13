use csv::Writer;
use dora_node_api::arrow::array::{AsArray, PrimitiveArray};
use dora_node_api::arrow::datatypes::UInt64Type;
use dora_node_api::{self, DoraNode, Event};
use eyre::ContextCompat;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use uhlc::system_time_clock;
use uhlc::HLC;

static LANGUAGE: &str = "Rust";
static PLATFORM: &str = "i7-8750@2.20GHz";
static NAME: &str = "adora daemon Rust";

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    // latency is tested first
    let latency = true;

    let mut current_size = 0;
    let mut n = 0;
    let mut start = Instant::now();
    let mut latencies = Vec::new();
    let hlc = HLC::default();
    let timestamp = hlc.new_timestamp().to_string();
    let date = timestamp
        .split('.')
        .next()
        .expect("Could not extract date from timestamp.");

    // Preallocated vector
    let sizes = [1, 10 * 512, 100 * 512, 1000 * 512, 10000 * 512];
    let mut root_vec = HashMap::new();
    for size in sizes {
        root_vec.insert(size, vec![0u64; size]);
    }

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id: _,
                data,
                metadata: _,
            } => {
                // check if new size bracket
                let array: &PrimitiveArray<UInt64Type> =
                    data.as_primitive_opt().context("not a primitive array")?;
                let array = array.values();
                let time_u64 = array.get(0).context("could not slice data")?;
                let t_send = uhlc::NTP64(*time_u64);

                let t_received = system_time_clock();

                latencies.push((t_received - t_send).to_duration());
                let data_len = data.len() * 8;
                if data_len != current_size {
                    if n > 0 {
                        record_results(start, current_size, n, latencies, latency, date);
                    }
                    current_size = data_len;
                    n = 0;
                    start = Instant::now();
                    latencies = Vec::new();
                }
                n += 1;
            }
            Event::InputClosed { id } => {
                println!("Input `{id}` was closed");
            }
            other => eprintln!("Received unexpected input: {other:?}"),
        }
    }

    if n > 0 {
        record_results(start, current_size, n, latencies, latency, date);
    }
    println!("finished");
    Ok(())
}

fn percentile(sorted: &[Duration], pct: f64) -> Duration {
    if sorted.is_empty() {
        return Duration::ZERO;
    }
    let idx = ((pct / 100.0) * (sorted.len() - 1) as f64).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

fn record_results(
    _start: Instant,
    current_size: usize,
    n: u32,
    mut latencies: Vec<Duration>,
    _latency: bool,
    date: &str,
) {
    latencies.sort();
    let avg_latency = latencies.iter().sum::<Duration>() / n;
    let p50 = percentile(&latencies, 50.0);
    let p90 = percentile(&latencies, 90.0);
    let p99 = percentile(&latencies, 99.0);

    let file = std::fs::OpenOptions::new()
        .write(true)
        .append(true)
        .create(true)
        .open("timer.csv")
        .unwrap();
    let mut wtr = Writer::from_writer(file);
    let name = std::env::var("NAME").unwrap_or_else(|_| NAME.to_string());

    wtr.write_record(&[
        date.to_string(),
        LANGUAGE.to_string(),
        PLATFORM.to_string(),
        name,
        current_size.to_string(),
        avg_latency.as_micros().to_string(),
        p50.as_micros().to_string(),
        p90.as_micros().to_string(),
        p99.as_micros().to_string(),
        n.to_string(),
    ])
    .unwrap();
}
