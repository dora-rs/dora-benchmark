//! ROS 2 Rust talker using rclrs with fixed-size messages on heap.
//! Uses CycloneDDS + iceoryx for shared memory transport.

use anyhow::Result;
use rclrs::*;
use std::time::Duration;

const SAMPLES: usize = 1000;
const TICK: Duration = Duration::from_millis(20);

fn clock_monotonic_ns() -> u64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts); }
    (ts.tv_sec as u64) * 1_000_000_000 + (ts.tv_nsec as u64)
}

fn main() -> Result<()> {
    // Run on a thread with 64 MB stack for large fixed-size messages.
    std::thread::Builder::new()
        .stack_size(64 * 1024 * 1024)
        .spawn(run)?
        .join()
        .unwrap()
}

fn run() -> Result<()> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("rclrs_shm_talker")?;

    let pub8 = node.create_publisher::<cpp_shm_pubsub::msg::Bench8>("bench8")?;
    let pub40k = node.create_publisher::<cpp_shm_pubsub::msg::Bench40K>("bench40k")?;
    let pub400k = node.create_publisher::<cpp_shm_pubsub::msg::Bench400K>("bench400k")?;
    let pub4m = node.create_publisher::<cpp_shm_pubsub::msg::Bench4M>("bench4m")?;

    std::thread::sleep(Duration::from_secs(2));

    eprintln!("Starting 8B bracket");
    for _ in 0..SAMPLES {
        let mut msg = cpp_shm_pubsub::msg::Bench8::default();
        msg.data[0] = clock_monotonic_ns();
        pub8.publish(&msg)?;
        std::thread::sleep(TICK);
    }

    eprintln!("Starting 40KB bracket");
    for _ in 0..SAMPLES {
        let mut msg = Box::new(cpp_shm_pubsub::msg::Bench40K::default());
        msg.data[0] = clock_monotonic_ns();
        pub40k.publish(&*msg)?;
        std::thread::sleep(TICK);
    }

    eprintln!("Starting 400KB bracket");
    for _ in 0..SAMPLES {
        let mut msg = Box::new(cpp_shm_pubsub::msg::Bench400K::default());
        msg.data[0] = clock_monotonic_ns();
        pub400k.publish(&*msg)?;
        std::thread::sleep(TICK);
    }

    eprintln!("Starting 4MB bracket");
    for _ in 0..SAMPLES {
        let mut msg = Box::new(cpp_shm_pubsub::msg::Bench4M::default());
        msg.data[0] = clock_monotonic_ns();
        pub4m.publish(&*msg)?;
        std::thread::sleep(TICK);
    }

    std::thread::sleep(Duration::from_secs(3));
    Ok(())
}
