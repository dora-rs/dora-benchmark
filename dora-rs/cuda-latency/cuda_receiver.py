#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Dora CUDA IPC receiver using raw cudaIpcOpenMemHandle (no numba overhead)."""

import ctypes
import os
import time

import pyarrow as pa
from tqdm import tqdm
from dora import Node
from helper import record_results

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
_libcudart.cudaIpcOpenMemHandle.restype = ctypes.c_int
_libcudart.cudaIpcOpenMemHandle.argtypes = [
    ctypes.POINTER(ctypes.c_void_p), _CudaIpcMemHandle, ctypes.c_uint,
]
_libcudart.cudaIpcCloseMemHandle.restype = ctypes.c_int
_libcudart.cudaIpcCloseMemHandle.argtypes = [ctypes.c_void_p]

_cudaMemcpyHostToDevice = 1
_cudaIpcMemLazyEnablePeerAccess = 1

# ---------------------------------------------------------------------------

# Init CUDA context.
_libcudart.cudaSetDevice(0)
_libcudart.cudaFree(ctypes.c_void_p(0))

pa.array([])
pbar = tqdm(total=100)
node = Node("node_2")

current_size = 8
n = 0
i = 0
latencies = []
DEVICE = os.getenv("DEVICE", "cuda")
NAME = f"dora torch {DEVICE}"

while True:
    event = node.next()
    if event["type"] == "INPUT":
        t_send = event["metadata"]["time"]

        if event["metadata"]["device"] != "cuda":
            # CPU path
            handle = event["value"].to_pylist()
            _libcudart.cudaDeviceSynchronize()
            t_received = time.perf_counter_ns()
            length = len(handle)
        else:
            # CUDA IPC path — raw ctypes, no numba
            # Signed int8 -> unsigned bytes
            handle_bytes = bytes([(b + 256) % 256 for b in event["value"].to_pylist()])
            handle = _CudaIpcMemHandle()
            ctypes.memmove(ctypes.byref(handle), handle_bytes, 64)

            d_ptr = ctypes.c_void_p()
            _libcudart.cudaIpcOpenMemHandle(
                ctypes.byref(d_ptr), handle, _cudaIpcMemLazyEnablePeerAccess,
            )
            _libcudart.cudaDeviceSynchronize()
            t_received = time.perf_counter_ns()

            num_elements = event["metadata"]["num_elements"]
            elem_size = event["metadata"]["element_size"]
            length = num_elements * elem_size

            _libcudart.cudaIpcCloseMemHandle(d_ptr)
    else:
        break

    if length != current_size:
        if n > 0:
            pbar.close()
            pbar = tqdm(total=100)
            record_results(NAME, current_size, latencies)
        current_size = length
        n = 0
        latencies = []

    pbar.update(1)
    latencies.append((t_received - t_send) / 1000)
    node.send_output("next", pa.array([]))

    n += 1
    i += 1

record_results(NAME, current_size, latencies)
