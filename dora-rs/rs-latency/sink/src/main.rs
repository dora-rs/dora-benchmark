use csv::Writer;
use dora_node_api::{self, DoraNode, Event};
use std::time::{Duration, Instant};
use uhlc::system_time_clock;
use uhlc::HLC;

static LANGUAGE: &str = "Rust";
static PLATFORM: &str = "i7-8750@2.20GHz";
static NAME: &str = "dora-rs daemon Rust";

fn main() -> eyre::Result<()> {
    let (_node, mut events) = DoraNode::init_from_env()?;

    // latency is tested first
    let mut latency = true;

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

    while let Some(event) = events.recv() {
        let t_received = system_time_clock();
        match event {
            Event::Input { id, data } => {
                // check if new size bracket
                let data = data.unwrap();
                let time_bytes = data.get(0..8).unwrap();
                let time_u64: &[u64] = bytemuck::cast_slice(time_bytes);
                let t_send = uhlc::NTP64(*time_u64.first().unwrap());
                latencies.push((t_received - t_send).to_duration());
                let data_len = data.len();
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

    record_results(start, current_size, n, latencies, latency, date);
    println!("finished");
    Ok(())
}

fn record_results(
    start: Instant,
    current_size: usize,
    n: u32,
    latencies: Vec<Duration>,
    latency: bool,
    date: &str,
) {
    let avg_latency = latencies.iter().sum::<Duration>() / n;
    let file = std::fs::OpenOptions::new()
        .write(true)
        .append(true)
        .create(true)
        .open("timer.csv")
        .unwrap();
    let mut wtr = Writer::from_writer(file);
    wtr.write_record(&[
        date.to_string(),
        LANGUAGE.to_string(),
        PLATFORM.to_string(),
        NAME.to_string(),
        current_size.to_string(),
        avg_latency.as_micros().to_string(),
    ])
    .unwrap();
}
