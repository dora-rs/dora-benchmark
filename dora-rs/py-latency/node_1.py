#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import numpy as np
import pyarrow as pa
from dora import Node

SIZES = [
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
    8,
]


node = Node()
pa.array([])

# test latency first
for size in SIZES:
    for _ in range(0, 100):
        random_data = np.random.randint(1000, size=size, dtype=np.int64)
        t_send = time.perf_counter_ns()
        random_data = np.array([[t_send, t_send], [t_send, t_send]])

        ## Arrow Test: Using Pybytes
        # node.send_output("latency", random_data.tobytes())
        ## Arrow Test: Using Arrow
        node.send_output("latency", pa.array(random_data))
        time.sleep(0.1)
