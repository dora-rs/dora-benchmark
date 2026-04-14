#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dora CUDA IPC sender for C++ benchmark.

Packs [timestamp_ns (8B)][num_elements (8B)][ipc_handle (64B)] into a single
uint8 Arrow array, so the C++ receiver can use `event_as_input` (no Arrow
FFI overhead) and decode everything from raw bytes.
"""

import ctypes
import struct
import time

import numpy as np
import pyarrow as pa
from dora import Node

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

SIZES = [1, 8, 64, 512, 5120, 51200, 512000, 5120000]
SAMPLES_PER_SIZE = 1000
INTER_SEND_SLEEP = 0.005
ELEMENT_SIZE = 8

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

    d_ptr = ctypes.c_void_p()
    _libcudart.cudaMalloc(ctypes.byref(d_ptr), byte_size)

    for _ in range(SAMPLES_PER_SIZE):
        _libcudart.cudaMemcpy(
            d_ptr, ctypes.c_char_p(host_bytes), byte_size, _cudaMemcpyHostToDevice,
        )
        _libcudart.cudaDeviceSynchronize()

        handle = _CudaIpcMemHandle()
        _libcudart.cudaIpcGetMemHandle(ctypes.byref(handle), d_ptr)

        t_send = time.perf_counter_ns()

        # Pack [timestamp (i64)][num_elements (i64)][handle (64B)] = 80 bytes
        payload = struct.pack("<qq", t_send, size) + bytes(handle)
        node.send_output(
            "cuda_data", pa.array(list(payload), type=pa.uint8()),
        )

        node.next()
        time.sleep(INTER_SEND_SLEEP)

    _libcudart.cudaFree(d_ptr)
