//! ROS 2 Rust talker using r2r — uses CLOCK_MONOTONIC for
//! cross-process timestamps.

use r2r::std_msgs::msg::UInt64MultiArray;
use r2r::QosProfile;
use rand::Rng;
use std::time::Duration;

// 10 sizes from 8 B to 4 MB (in u64 elements)
const SIZES: [usize; 10] = [1, 128, 1280, 12800, 64000, 131072, 196608, 262144, 393216, 524288];
const SAMPLES: usize = 1000;
const TICK: Duration = Duration::from_millis(20);

fn clock_monotonic_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts); }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "r2r_benchmark_talker", "")?;
    let publisher =
        node.create_publisher::<UInt64MultiArray>("/benchmark_topic", QosProfile::default())?;

    std::thread::sleep(Duration::from_secs(1));

    let mut rng = rand::thread_rng();

    for size in SIZES {
        // Use longer tick for large messages to avoid overwhelming DDS.
        let tick = TICK;
        let n = SAMPLES;
        eprintln!("Starting {}B bracket (tick={}ms)", size * 8, tick.as_millis());
        for _ in 0..n {
            let mut data: Vec<u64> = (0..size).map(|_| rng.gen()).collect();
            data[0] = clock_monotonic_ns();

            let msg = UInt64MultiArray {
                layout: Default::default(),
                data,
            };
            if let Err(e) = publisher.publish(&msg) {
                eprintln!("publish failed for {}B: {:?}", size * 8, e);
                break;
            }
            node.spin_once(Duration::from_millis(0));
            std::thread::sleep(tick);
        }
    }

    std::thread::sleep(Duration::from_secs(3));
    Ok(())
}
