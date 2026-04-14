#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dora CUDA IPC sender using raw cudaIpcGetMemHandle (no numba overhead)."""

import ctypes
import os
import time

import numpy as np
import pyarrow as pa
from dora import Node

# ---------------------------------------------------------------------------
# CUDA runtime via ctypes (same approach as the ROS2 Python benchmark)
# ---------------------------------------------------------------------------
_libcudart = ctypes.CDLL("libcudart.so")


class _CudaIpcMemHandle(ctypes.Structure):
    _fields_ = [("reserved", ctypes.c_byte * 64)]


_libcudart.cudaSetDevice.restype = ctypes.c_int
_libcudart.cudaSetDevice.argtypes = [ctypes.c_int]
_libcudart.cudaMalloc.restype = ctypes.c_int
_libcudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
_libcudart.cudaFree.restype = ctypes.c_int
_libcudart.cudaFree.argtypes = [ctypes.c_void_p]
_libcudart.cudaMemcpy.restype = ctypes.c_int
_libcudart.cudaMemcpy.argtypes = [
    ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int,
]
_libcudart.cudaDeviceSynchronize.restype = ctypes.c_int
_libcudart.cudaDeviceSynchronize.argtypes = []
_libcudart.cudaIpcGetMemHandle.restype = ctypes.c_int
_libcudart.cudaIpcGetMemHandle.argtypes = [
    ctypes.POINTER(_CudaIpcMemHandle), ctypes.c_void_p,
]

_cudaMemcpyHostToDevice = 1
_cudaMemcpyDeviceToHost = 2

# ---------------------------------------------------------------------------

# Sizes as number of int64 elements. Adds more granularity at the small end.
SIZES = [1, 8, 64, 512, 5120, 51200, 512000, 5120000]
SAMPLES_PER_SIZE = 1000
INTER_SEND_SLEEP = 0.005  # 5 ms
ELEMENT_SIZE = 8  # int64

DEVICE = os.getenv("DEVICE", "cuda")

# Init CUDA context.
_libcudart.cudaSetDevice(0)
_libcudart.cudaFree(ctypes.c_void_p(0))

pa.array([])
node = Node()
rng = np.random.default_rng(42)

time.sleep(4)

for size in SIZES:
    byte_size = size * ELEMENT_SIZE
    host_data = rng.integers(1000, size=size, dtype=np.int64)
    host_bytes = host_data.tobytes()

    # Allocate GPU buffer once per size bracket.
    d_ptr = ctypes.c_void_p()
    _libcudart.cudaMalloc(ctypes.byref(d_ptr), byte_size)

    for _ in range(SAMPLES_PER_SIZE):
        _libcudart.cudaMemcpy(
            d_ptr, ctypes.c_char_p(host_bytes), byte_size, _cudaMemcpyHostToDevice,
        )
        _libcudart.cudaDeviceSynchronize()

        if DEVICE == "cpu":
            cpu_buf = (ctypes.c_uint8 * byte_size)()
            _libcudart.cudaMemcpy(cpu_buf, d_ptr, byte_size, _cudaMemcpyDeviceToHost)
            metadata = {"time": time.perf_counter_ns(), "device": "cpu"}
            node.send_output(
                "cuda_data",
                pa.array(list(bytes(cpu_buf)), type=pa.int8()),
                metadata,
            )
        else:
            handle = _CudaIpcMemHandle()
            _libcudart.cudaIpcGetMemHandle(ctypes.byref(handle), d_ptr)
            t_send = time.perf_counter_ns()
            metadata = {
                "time": t_send,
                "device": "cuda",
                "num_elements": size,
                "element_size": ELEMENT_SIZE,
                "size": size * ELEMENT_SIZE,
            }
            # Convert to signed int8 (same format as dora.cuda)
            handle_bytes = bytes(handle)
            signed = [(b if b < 128 else b - 256) for b in handle_bytes]
            node.send_output(
                "cuda_data", pa.array(signed, type=pa.int8()), metadata,
            )

        node.next()
        time.sleep(INTER_SEND_SLEEP)

    _libcudart.cudaFree(d_ptr)
