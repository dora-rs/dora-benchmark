use dora_node_api::{
    arrow::array::{Array, Int8Array},
    DoraNode, Event, MetadataParameters, Parameter,
};
use std::collections::BTreeMap;
use std::io::Write;

// Raw CUDA runtime FFI — just the IPC functions we need.
#[repr(C)]
#[derive(Copy, Clone)]
struct CudaIpcMemHandle {
    reserved: [u8; 64],
}

extern "C" {
    fn cudaSetDevice(device: i32) -> i32;
    fn cudaFree(devPtr: *mut u8) -> i32;
    fn cudaIpcOpenMemHandle(
        devPtr: *mut *mut u8,
        handle: CudaIpcMemHandle,
        flags: u32,
    ) -> i32;
    fn cudaIpcCloseMemHandle(devPtr: *mut u8) -> i32;
    fn cudaDeviceSynchronize() -> i32;
}

const CUDA_IPC_MEM_LAZY_ENABLE_PEER_ACCESS: u32 = 1;
const WARMUP_SAMPLES: i32 = 10;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;

    // Init CUDA context.
    unsafe {
        let err = cudaSetDevice(0);
        if err != 0 {
            eyre::bail!("cudaSetDevice failed: {}", err);
        }
        cudaFree(std::ptr::null_mut());
    }
    eprintln!("CUDA context ready on device 0");

    let mut latencies: BTreeMap<i64, Vec<f64>> = BTreeMap::new();
    let mut warmup_count: BTreeMap<i64, i32> = BTreeMap::new();

    while let Some(event) = events.recv() {
        match event {
            Event::Input {
                id,
                metadata,
                data,
            } => {
                if id.as_str() != "cuda_data" {
                    continue;
                }

                let t_send = match metadata.parameters.get("time") {
                    Some(Parameter::Integer(v)) => *v,
                    _ => continue,
                };
                let num_elements = match metadata.parameters.get("num_elements") {
                    Some(Parameter::Integer(v)) => *v,
                    _ => continue,
                };

                // Extract 64-byte IPC handle.
                let int8_array = data
                    .as_any()
                    .downcast_ref::<Int8Array>()
                    .ok_or_else(|| eyre::eyre!("expected Int8Array"))?;
                let values = int8_array.values();
                if values.len() != 64 {
                    eyre::bail!("expected 64-byte handle, got {}", values.len());
                }

                let mut handle = CudaIpcMemHandle { reserved: [0u8; 64] };
                unsafe {
                    std::ptr::copy_nonoverlapping(
                        values.as_ptr() as *const u8,
                        handle.reserved.as_mut_ptr(),
                        64,
                    );
                }

                // Open IPC handle.
                let mut d_ptr: *mut u8 = std::ptr::null_mut();
                let err = unsafe {
                    cudaIpcOpenMemHandle(
                        &mut d_ptr,
                        handle,
                        CUDA_IPC_MEM_LAZY_ENABLE_PEER_ACCESS,
                    )
                };
                if err != 0 {
                    eprintln!("cudaIpcOpenMemHandle failed: err={}", err);
                    // Still send ACK so sender doesn't hang.
                    node.send_output(
                        "next".into(),
                        MetadataParameters::default(),
                        dora_node_api::arrow::array::UInt8Array::from(vec![0u8]),
                    )?;
                    continue;
                }
                unsafe { cudaDeviceSynchronize(); }

                let t_received = clock_monotonic_ns();

                unsafe { cudaIpcCloseMemHandle(d_ptr); }

                // Send ACK.
                node.send_output(
                    "next".into(),
                    MetadataParameters::default(),
                    dora_node_api::arrow::array::UInt8Array::from(vec![0u8]),
                )?;

                let size_bytes = num_elements * 8;
                let latency_us = (t_received - t_send) as f64 / 1000.0;

                let wc = warmup_count.entry(size_bytes).or_insert(0);
                if *wc < WARMUP_SAMPLES {
                    *wc += 1;
                } else {
                    latencies.entry(size_bytes).or_default().push(latency_us);
                }
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    // Write CSV results.
    let csv_path = std::env::var("CSV_TIME_FILE").unwrap_or_else(|_| "time.csv".into());
    let mut file = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&csv_path)?;

    for (size_bytes, mut lats) in latencies {
        lats.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let n = lats.len();
        if n == 0 { continue; }
        let avg = lats.iter().sum::<f64>() / n as f64;
        let p50 = lats[n / 2];
        let p90 = lats[std::cmp::min((n as f64 * 0.9) as usize, n - 1)];
        let p99 = lats[std::cmp::min((n as f64 * 0.99) as usize, n - 1)];

        eprintln!(
            "size={}  avg={:.0}us  p50={:.0}us  p90={:.0}us  p99={:.0}us  n={}",
            size_bytes, avg, p50, p90, p99, n
        );
        writeln!(
            file,
            "Rust,COMPUTER_PERF,dora Rust CUDA IPC,{},{},{},{},{},{}",
            size_bytes, avg as i64, p50 as i64, p90 as i64, p99 as i64, n
        )?;
    }

    eprintln!("receiver done");
    Ok(())
}

fn clock_monotonic_ns() -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts); }
    ts.tv_sec as i64 * 1_000_000_000 + ts.tv_nsec as i64
}
