use dora_node_api::Event;
use dora_node_api::{self, arrow::array::UInt64Array, dora_core::config::DataId, DoraNode};
use rand::Rng;
use std::time::Duration;
use uhlc::system_time_clock;

fn main() -> eyre::Result<()> {
    let latency = DataId::from("latency".to_owned());
    let _throughput = DataId::from("throughput".to_owned());

    let (mut node, mut events) = DoraNode::init_from_env()?;
    // 10 sizes from 8 B to 4 MB, in u64 elements (multiply by 8 for bytes)
    // 8B, 1KB, 10KB, 100KB, 500KB, 1MB, 1.5MB, 2MB, 3MB, 4MB
    let sizes: [usize; 10] = [
        1,           // 8 B
        128,         // 1 KB
        1280,        // 10 KB
        12800,       // 100 KB
        64000,       // 500 KB
        131072,      // 1 MB
        196608,      // 1.5 MB
        262144,      // 2 MB
        393216,      // 3 MB
        524288,      // 4 MB
    ];

    // test latency first
    for size in sizes {
        for _ in 0..1000 {
            if let Some(event) = events.recv() {
                match event {
                    Event::Input {
                        id: _,
                        data: _,
                        metadata,
                    } => {
                        let mut random_data: Vec<u64> = rand::thread_rng()
                            .sample_iter(rand::distributions::Standard)
                            .take(size)
                            .collect();
                        let t_send = system_time_clock().as_u64();
                        let beginning_slice = random_data.get_mut(0).unwrap();
                        *beginning_slice = t_send;

                        let random_data: UInt64Array = random_data.into();

                        node.send_output(latency.clone(), metadata.parameters, random_data)?;
                    }
                    _ => {}
                }
            } else {
                break;
            }
        }
    }

    Ok(())
}
