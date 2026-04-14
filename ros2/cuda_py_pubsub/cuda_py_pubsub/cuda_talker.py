"""ROS 2 Python CUDA IPC talker — GPU-to-GPU latency benchmark.

Allocates GPU buffers via raw cudaMalloc (ctypes), extracts CUDA IPC handles,
and publishes them as ROS 2 messages.  Supports IPC mode (default) and CPU
fallback mode (CUDA_BENCH_MODE=cpu).
"""

import ctypes
import os
import time
import threading

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
    """Declare CUDA runtime function signatures."""
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

    lib.cudaIpcGetMemHandle.restype = ctypes.c_int
    lib.cudaIpcGetMemHandle.argtypes = [
        ctypes.POINTER(_CudaIpcMemHandle), ctypes.c_void_p
    ]

    return lib


_cuda = _setup_cuda()
_cudaMemcpyHostToDevice = 1
_cudaMemcpyDeviceToHost = 2

# ---------------------------------------------------------------------------
# Benchmark parameters — match Dora cuda-latency exactly
# ---------------------------------------------------------------------------
SIZES = [1, 8, 64, 512, 5120, 51200, 512000, 5120000]  # int64 element counts
SAMPLES_PER_SIZE = 1000
WARMUP_SAMPLES = 10
ELEMENT_SIZE = 8  # sizeof(int64_t)


def _cuda_check(err, msg=""):
    if err != 0:
        raise RuntimeError(f"CUDA error {err}: {msg}")


class CudaTalker(Node):
    def __init__(self):
        super().__init__("cuda_benchmark_talker")

        self.ipc_mode = os.getenv("CUDA_BENCH_MODE", "ipc") != "cpu"
        self.get_logger().info(
            f"CUDA benchmark talker — mode={'ipc' if self.ipc_mode else 'cpu'}"
        )

        # Init CUDA context.
        _cuda_check(_cuda.cudaSetDevice(0), "cudaSetDevice")
        _cuda_check(_cuda.cudaFree(ctypes.c_void_p(0)), "cudaFree(0)")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.ipc_pub = self.create_publisher(CudaIpc, "cuda_ipc", qos)
        self.cpu_pub = self.create_publisher(UInt8MultiArray, "cuda_cpu_data", qos)

        self.ack_event = threading.Event()
        self.ack_sub = self.create_subscription(
            Empty, "cuda_ack", self._on_ack, qos
        )

    def _on_ack(self, _msg):
        self.ack_event.set()

    def run_benchmark(self):
        """Run the full benchmark (call from a non-spinning thread)."""
        # Wait for subscriber discovery.
        time.sleep(4)

        rng = np.random.default_rng(42)

        for size in SIZES:
            byte_size = size * ELEMENT_SIZE

            # Pre-generate host data.
            host_data = rng.integers(0, 1000, size=size, dtype=np.int64)
            host_bytes = host_data.tobytes()

            total = WARMUP_SAMPLES + SAMPLES_PER_SIZE
            self.get_logger().info(
                f"size={byte_size}B ({size} elements), "
                f"warmup={WARMUP_SAMPLES}, samples={SAMPLES_PER_SIZE}"
            )

            # Allocate GPU buffer once per size bracket to avoid CUDA VA
            # reuse conflicts with IPC handle mapping on the receiver side.
            d_ptr = ctypes.c_void_p()
            _cuda_check(
                _cuda.cudaMalloc(ctypes.byref(d_ptr), byte_size),
                "cudaMalloc",
            )

            for _ in range(total):
                # Re-fill buffer each iteration.
                _cuda_check(
                    _cuda.cudaMemcpy(
                        d_ptr,
                        ctypes.c_char_p(host_bytes),
                        byte_size,
                        _cudaMemcpyHostToDevice,
                    ),
                    "cudaMemcpy H2D",
                )
                _cuda_check(_cuda.cudaDeviceSynchronize(), "sync after fill")

                if self.ipc_mode:
                    # Get IPC handle.
                    handle = _CudaIpcMemHandle()
                    _cuda_check(
                        _cuda.cudaIpcGetMemHandle(ctypes.byref(handle), d_ptr),
                        "cudaIpcGetMemHandle",
                    )

                    # Build and publish message.
                    msg = CudaIpc()
                    msg.ipc_handle = list(bytes(handle))
                    msg.timestamp_ns = time.perf_counter_ns()
                    msg.num_elements = size
                    msg.element_size = ELEMENT_SIZE
                    msg.mode = 0
                    self.ipc_pub.publish(msg)
                else:
                    # CPU fallback: copy D2H and send raw bytes.
                    cpu_buf = (ctypes.c_uint8 * byte_size)()
                    _cuda_check(
                        _cuda.cudaMemcpy(
                            cpu_buf, d_ptr, byte_size, _cudaMemcpyDeviceToHost
                        ),
                        "cudaMemcpy D2H",
                    )

                    meta = CudaIpc()
                    meta.ipc_handle = [0] * 64
                    meta.timestamp_ns = time.perf_counter_ns()
                    meta.num_elements = size
                    meta.element_size = ELEMENT_SIZE
                    meta.mode = 1
                    self.ipc_pub.publish(meta)

                    data_msg = UInt8MultiArray()
                    data_msg.data = list(bytes(cpu_buf))
                    self.cpu_pub.publish(data_msg)

                # Wait for ACK.
                self.ack_event.clear()
                if not self.ack_event.wait(timeout=5.0):
                    self.get_logger().warn(
                        f"ACK timeout at size={byte_size}B"
                    )

                time.sleep(0.005)

            # Free GPU buffer after bracket completes.
            _cuda_check(_cuda.cudaFree(d_ptr), "cudaFree")

        self.get_logger().info("Benchmark complete.")
        time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    node = CudaTalker()

    # Spin in background so ACK callbacks are processed.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.run_benchmark()

    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
