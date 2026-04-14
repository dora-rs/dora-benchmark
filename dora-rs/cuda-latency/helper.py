import csv
import os
import platform
from datetime import datetime

import numpy as np
from dora import __version__

LATENCY = True

DATE = str(datetime.now())
LANGUAGE = f"Python {platform.python_version()}"
PLATFORM = platform.platform()
DORA_VERSION = __version__
LOG_HEADER = [
    "Date",
    "Language",
    "Dora Version",
    "Platform",
    "Name",
    "Size (bit)",
    "avg (us)",
    "p50 (us)",
    "p90 (us)",
    "p99 (us)",
    "n",
]


def record_results(name, current_size, latencies):
    arr = np.array(latencies)
    avg = int(arr.mean())
    p50 = int(np.percentile(arr, 50))
    p90 = int(np.percentile(arr, 90))
    p99 = int(np.percentile(arr, 99))
    n = len(latencies)

    csv_file = os.getenv("CSV_TIME_FILE", "benchmark_data.csv")
    append = os.path.isfile(csv_file)
    log_row = [
        DATE,
        LANGUAGE,
        DORA_VERSION,
        PLATFORM,
        name,
        current_size,
        avg,
        p50,
        p90,
        p99,
        n,
    ]
    if append:
        with open(csv_file, "a", encoding="utf-8") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(log_row)
    else:
        with open(csv_file, "w+", encoding="utf-8") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(LOG_HEADER)
            w.writerow(log_row)

    print(
        f"size={current_size}  avg={avg}us  p50={p50}us  "
        f"p90={p90}us  p99={p99}us  n={n}"
    )
