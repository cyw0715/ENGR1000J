"""Fit and audit HC-SR04 absolute-distance calibration from capture CSV files.

Fits true_mm = a * measured_mm + b after robust per-point median aggregation.
Raw rows are retained; invalid status rows are excluded from fitting but counted.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from datetime import datetime
from pathlib import Path


def percentile(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * p
    lo, hi = math.floor(index), math.ceil(index)
    return ordered[lo] if lo == hi else ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def fit_linear(xs: list[float], ys: list[float]) -> tuple[float, float]:
    x_mean, y_mean = statistics.mean(xs), statistics.mean(ys)
    denom = sum((x - x_mean) ** 2 for x in xs)
    if denom == 0:
        raise ValueError("need at least two distinct measured distances")
    a = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom
    b = y_mean - a * x_mean
    return a, b


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture_dir", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    files = sorted(args.capture_dir.glob("sr04_true_*mm_*.csv"))
    if len(files) < 3:
        raise ValueError("need captures at three or more true distances")

    grouped: dict[float, list[float]] = defaultdict(list)
    invalid: dict[float, int] = defaultdict(int)
    for path in files:
        with path.open(newline="", encoding="utf-8") as file:
            for row in csv.DictReader(file):
                truth = float(row["true_mm"])
                if row["valid"] == "1" and row["status"] == "ok" and int(row["sr04_distance_mm"]) >= 0:
                    grouped[truth].append(float(row["sr04_distance_mm"]))
                else:
                    invalid[truth] += 1

    truths = sorted(grouped)
    if len(truths) < 3:
        raise ValueError("fewer than three distances have valid samples")
    points = []
    for truth in truths:
        values = grouped[truth]
        median = statistics.median(values)
        mad = statistics.median(abs(x - median) for x in values)
        points.append({
            "true_mm": truth,
            "count": len(values),
            "invalid": invalid[truth],
            "measured_median_mm": median,
            "measured_mean_mm": statistics.mean(values),
            "measured_stddev_mm": statistics.pstdev(values),
            "measured_p05_mm": percentile(values, 0.05),
            "measured_p95_mm": percentile(values, 0.95),
            "mad_mm": mad,
        })

    xs = [p["measured_median_mm"] for p in points]
    ys = [p["true_mm"] for p in points]
    a, b = fit_linear(xs, ys)
    for point in points:
        point["fitted_true_mm"] = a * point["measured_median_mm"] + b
        point["residual_mm"] = point["fitted_true_mm"] - point["true_mm"]
    residuals = [p["residual_mm"] for p in points]
    rmse = math.sqrt(sum(x * x for x in residuals) / len(residuals))
    max_abs = max(abs(x) for x in residuals)

    report = {
        "created_utc": datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "model": "true_distance_mm = scale * sr04_measured_mm + offset_mm",
        "scale": a,
        "offset_mm": b,
        "fit_points": len(points),
        "range_true_mm": [min(truths), max(truths)],
        "point_rmse_mm": rmse,
        "max_abs_point_residual_mm": max_abs,
        "points": points,
        "warnings": [
            "Calibration is valid only for the tested target material, normal alignment, and true-distance range.",
            "Do not use the fitted equation as actuator-control truth without independent terrain and attitude validation.",
        ],
    }
    import json
    out = args.out or args.capture_dir / "sr04_calibration_report.json"
    out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"REPORT_FILE={out}")
    print(f"MODEL true_mm = {a:.8f} * measured_mm + {b:.3f}")
    print(f"POINT_RMSE_MM={rmse:.3f} MAX_ABS_RESIDUAL_MM={max_abs:.3f}")
    for point in points:
        print(f"POINT true={point['true_mm']:.1f} measured_median={point['measured_median_mm']:.1f} "
              f"stddev={point['measured_stddev_mm']:.2f} valid={point['count']} invalid={point['invalid']} "
              f"residual={point['residual_mm']:+.2f}")


if __name__ == "__main__":
    main()
