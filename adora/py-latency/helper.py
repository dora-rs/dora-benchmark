import csv
import os
import platform
from datetime import datetime

import numpy as np

DATE = str(datetime.now().isoformat(timespec="seconds"))
LANGUAGE = f"Python {platform.python_version()}"
PLATFORM = platform.platform()


def record_results(name, current_size, latencies):
    arr = np.array(latencies)
    avg_us = int(arr.mean())
    p50_us = int(np.percentile(arr, 50))
    p90_us = int(np.percentile(arr, 90))
    p99_us = int(np.percentile(arr, 99))
    n = len(latencies)

    csv_file = os.getenv("CSV_TIME_FILE", "benchmark_data.csv")
    with open(csv_file, "a", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow([
            DATE,
            LANGUAGE,
            PLATFORM,
            name,
            current_size,
            avg_us,
            p50_us,
            p90_us,
            p99_us,
            n,
        ])
