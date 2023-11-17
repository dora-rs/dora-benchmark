import csv
import os
import platform
from datetime import datetime

import numpy as np

LATENCY = True

DATE = str(datetime.now())
LANGUAGE = f"Python {platform.python_version()}"
PLATFORM = platform.platform()
DORA_VERSION = "0.1.0"
LOG_HEADER = [
    "Date",
    "Language",
    "Dora Version",
    "Platform",
    "Name",
    "Size (bit)",
    "Latency (ns)",
]


def record_results(name, current_size, latencies):
    avg_latency = np.array(latencies).mean()

    csv_file = os.getenv("CSV_TIME_FILE", "benchmark_data.csv")
    append = os.path.isfile(csv_file)
    log_row = [
        DATE,
        LANGUAGE,
        DORA_VERSION,
        PLATFORM,
        name,
        current_size,
        avg_latency,
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
