#!/usr/bin/env python
"""Dora CUDA IPC benchmark sender.

Allocates PyTorch GPU tensors, serializes them as CUDA IPC handles via
dora.cuda, and sends them to the receiver node.  Supports both CUDA
zero-copy (default) and CPU fallback (DEVICE=cpu).
"""

import os
import time

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from dora.cuda import torch_to_ipc_buffer

torch.tensor([], device="cuda")

# Match ROS2 CUDA benchmark sizes: number of int64 elements.
SIZES = [512, 5120, 51200, 512000, 5120000]
SAMPLES_PER_SIZE = 100

DEVICE = os.getenv("DEVICE", "cuda")

pa.array([])
node = Node()

rng = np.random.default_rng(42)

time.sleep(4)

for size in SIZES:
    for _ in range(SAMPLES_PER_SIZE):
        random_data = rng.integers(1000, size=size, dtype=np.int64)
        torch_tensor = torch.tensor(random_data, dtype=torch.int64, device="cuda")
        t_send = time.perf_counter_ns()
        if DEVICE == "cpu":
            torch_tensor = torch_tensor.to("cpu")
            metadata = {}
            metadata["time"] = t_send
            metadata["device"] = "cpu"
            node.send_output("latency", pa.array(torch_tensor.numpy()), metadata)
        else:
            ipc_buffer, metadata = torch_to_ipc_buffer(torch_tensor)
            metadata["time"] = t_send
            metadata["device"] = "cuda"
            node.send_output("latency", ipc_buffer, metadata)

        # Wait for receiver ACK before sending next.
        node.next()
        time.sleep(0.05)
