use dora_node_api::{self, dora_core::config::DataId, DoraNode};
use rand::Rng;
use std::time::Duration;
use uhlc::system_time_clock;

fn main() -> eyre::Result<()> {
    let latency = DataId::from("latency".to_owned());
    let throughput = DataId::from("throughput".to_owned());

    let (mut node, _events) = DoraNode::init_from_env()?;
    let sizes = [
        8,
        64,
        512,
        2048,
        4096,
        4 * 4096,
        10 * 4096,
        100 * 4096,
        1000 * 4096,
        10000 * 4096,
    ];

    // test latency first
    for size in sizes {
        for _ in 0..100 {
            let mut random_data: Vec<u8> = rand::thread_rng()
                .sample_iter(rand::distributions::Standard)
                .take(size)
                .collect();
            let t_send = system_time_clock().as_u64();
            let t_send: &[u8] = bytemuck::bytes_of(&t_send);
            let beginning_slice = random_data.get_mut(0..8).unwrap();
            beginning_slice.copy_from_slice(t_send);
            node.send_output(
                latency.clone(),
                Default::default(),
                random_data.len(),
                |out| {
                    out.copy_from_slice(&random_data);
                },
            )?;

            // sleep a bit to avoid queue buildup
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    Ok(())
}
