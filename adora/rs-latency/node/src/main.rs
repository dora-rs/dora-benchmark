use adora_node_api::Event;
use adora_node_api::{self, arrow::array::UInt64Array, adora_core::config::DataId, AdoraNode};
use rand::Rng;
use uhlc::system_time_clock;

fn main() -> eyre::Result<()> {
    let latency = DataId::from("latency".to_owned());
    let _throughput = DataId::from("throughput".to_owned());

    let (mut node, mut events) = AdoraNode::init_from_env()?;
    let sizes = [1, 10 * 512, 100 * 512, 1000 * 512, 10000 * 512];

    // test latency first
    for size in sizes {
        for _ in 0..100 {
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

                        // BUG: This crashes the daemon connection with:
                        //   "fatal event stream error: daemon channel broken"
                        //   "failed to deserialize DaemonReply"
                        // Workaround: use send_output_raw() with bytemuck::cast_slice()
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
