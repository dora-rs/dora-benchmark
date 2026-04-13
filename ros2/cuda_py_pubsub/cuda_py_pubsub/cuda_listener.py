"""ROS 2 Python CUDA IPC listener — GPU-to-GPU latency benchmark.

Receives CUDA IPC handles, opens GPU memory, measures latency, records CSV.
Supports IPC mode (default) and CPU fallback mode (CUDA_BENCH_MODE=cpu).
"""

import csv
import ctypes
import datetime
import os
import platform
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cuda_interfaces.msg import CudaIpc
from std_msgs.msg import Empty, UInt8MultiArray

# ---------------------------------------------------------------------------
# CUDA runtime via ctypes
# ---------------------------------------------------------------------------
_libcudart = ctypes.CDLL("libcudart.so")


class _CudaIpcMemHandle(ctypes.Structure):
    _fields_ = [("reserved", ctypes.c_byte * 64)]


def _setup_cuda():
    lib = _libcudart

    lib.cudaSetDevice.restype = ctypes.c_int
    lib.cudaSetDevice.argtypes = [ctypes.c_int]

    lib.cudaMalloc.restype = ctypes.c_int
    lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]

    lib.cudaFree.restype = ctypes.c_int
    lib.cudaFree.argtypes = [ctypes.c_void_p]

    lib.cudaMemcpy.restype = ctypes.c_int
    lib.cudaMemcpy.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int
    ]

    lib.cudaDeviceSynchronize.restype = ctypes.c_int
    lib.cudaDeviceSynchronize.argtypes = []

    lib.cudaIpcOpenMemHandle.restype = ctypes.c_int
    lib.cudaIpcOpenMemHandle.argtypes = [
        ctypes.POINTER(ctypes.c_void_p), _CudaIpcMemHandle, ctypes.c_uint
    ]

    lib.cudaIpcCloseMemHandle.restype = ctypes.c_int
    lib.cudaIpcCloseMemHandle.argtypes = [ctypes.c_void_p]

    return lib


_cuda = _setup_cuda()
_cudaMemcpyHostToDevice = 1
_cudaIpcMemLazyEnablePeerAccess = 1

WARMUP_SAMPLES = 10
DATE = datetime.datetime.now().isoformat(timespec="seconds")


def _cuda_check(err, msg=""):
    if err != 0:
        raise RuntimeError(f"CUDA error {err}: {msg}")


def record_results(name, size_bytes, latencies_us):
    """Append one CSV row per size bracket."""
    arr = np.array(latencies_us)
    avg = int(arr.mean())
    p50 = int(np.percentile(arr, 50))
    p90 = int(np.percentile(arr, 90))
    p99 = int(np.percentile(arr, 99))
    n = len(latencies_us)

    csv_path = os.getenv("CSV_TIME_FILE", "time.csv")
    with open(csv_path, "a", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow([
            DATE,
            f"Python {platform.python_version()}",
            "COMPUTER_PERF",
            name,
            size_bytes,
            avg,
            p50,
            p90,
            p99,
            n,
        ])

    print(
        f"size={size_bytes}  avg={avg}us  p50={p50}us  "
        f"p90={p90}us  p99={p99}us  n={n}"
    )


class CudaListener(Node):
    def __init__(self):
        super().__init__("cuda_benchmark_listener")

        is_cpu = os.getenv("CUDA_BENCH_MODE", "ipc") == "cpu"
        self.name = os.getenv("NAME") or (
            "ROS 2 Python CUDA CPU" if is_cpu else "ROS 2 Python CUDA IPC"
        )

        # Init CUDA context.
        _cuda_check(_cuda.cudaSetDevice(0), "cudaSetDevice")
        _cuda_check(_cuda.cudaFree(ctypes.c_void_p(0)), "cudaFree(0)")

        self.latencies = defaultdict(list)
        self.warmup_count = defaultdict(int)
        self.pending_cpu_data = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.ack_pub = self.create_publisher(Empty, "cuda_ack", qos)

        self.ipc_sub = self.create_subscription(
            CudaIpc, "cuda_ipc", self.on_ipc_msg, qos
        )
        self.cpu_sub = self.create_subscription(
            UInt8MultiArray, "cuda_cpu_data", self.on_cpu_data, qos
        )

        self.get_logger().info(f"CUDA benchmark listener ready — name={self.name}")

    def on_cpu_data(self, msg):
        self.pending_cpu_data = msg

    def on_ipc_msg(self, msg):
        t_send = msg.timestamp_ns
        num_elements = msg.num_elements
        elem_size = msg.element_size
        size_bytes = num_elements * elem_size

        if msg.mode == 0:
            # CUDA IPC mode.
            handle = _CudaIpcMemHandle()
            ctypes.memmove(
                ctypes.byref(handle), bytes(msg.ipc_handle), 64
            )

            d_ptr = ctypes.c_void_p()
            _cuda_check(
                _cuda.cudaIpcOpenMemHandle(
                    ctypes.byref(d_ptr), handle, _cudaIpcMemLazyEnablePeerAccess
                ),
                "cudaIpcOpenMemHandle",
            )
            _cuda_check(_cuda.cudaDeviceSynchronize(), "sync after open")

            t_received = time.perf_counter_ns()

            _cuda_check(
                _cuda.cudaIpcCloseMemHandle(d_ptr), "cudaIpcCloseMemHandle"
            )
        else:
            # CPU fallback mode.
            if self.pending_cpu_data is None:
                self.get_logger().warn("CPU mode: data message not yet received")
                self.ack_pub.publish(Empty())
                return

            data = bytes(self.pending_cpu_data.data)
            self.pending_cpu_data = None

            d_ptr = ctypes.c_void_p()
            _cuda_check(
                _cuda.cudaMalloc(ctypes.byref(d_ptr), len(data)), "cudaMalloc"
            )
            _cuda_check(
                _cuda.cudaMemcpy(
                    d_ptr, ctypes.c_char_p(data), len(data), _cudaMemcpyHostToDevice
                ),
                "cudaMemcpy H2D",
            )
            _cuda_check(_cuda.cudaDeviceSynchronize(), "sync after H2D")

            t_received = time.perf_counter_ns()

            _cuda_check(_cuda.cudaFree(d_ptr), "cudaFree")

        # Publish ACK (after closing handle / freeing memory).
        self.ack_pub.publish(Empty())

        # Record latency (skip warmup).
        warmup = self.warmup_count[size_bytes]
        if warmup < WARMUP_SAMPLES:
            self.warmup_count[size_bytes] = warmup + 1
            return

        latency_us = (t_received - t_send) / 1000
        self.latencies[size_bytes].append(latency_us)

    def flush_all(self):
        for size_bytes in sorted(self.latencies.keys()):
            lat = self.latencies[size_bytes]
            if lat:
                record_results(self.name, size_bytes, lat)


def main(args=None):
    rclpy.init(args=args)
    node = CudaListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush_all()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
