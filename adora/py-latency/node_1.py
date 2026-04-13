#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import numpy as np
import pyarrow as pa
from dora import Node

SIZES = [1, 10 * 512, 100 * 512, 1000 * 512, 10000 * 512]
SAMPLES_PER_SIZE = 1000


node = Node()
pa.array([])

# test latency first
for size in SIZES:
    for _ in range(SAMPLES_PER_SIZE):
        now = time.time()
        random_data = np.random.randint(1000, size=size, dtype=np.uint64)
        random_data[0] = time.perf_counter_ns()

        node.send_output("latency", pa.array(random_data))
        elapsed = time.time() - now
        remaining = 0.02 - elapsed
        if remaining > 0:
            time.sleep(remaining)
