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
    10 * 512,
    100 * 512,
    1000 * 512,
    10000 * 512,
    8,
]


node = Node()
pa.array([])

# test latency first
for size in SIZES:
    for _ in range(0, 100):
        random_data = np.random.randint(1000, size=size, dtype=np.uint64)
        t_send = time.perf_counter_ns()
        random_data[0] = np.array([time.perf_counter_ns()])

        node.send_output("latency", pa.array(random_data))
        time.sleep(0.05)
