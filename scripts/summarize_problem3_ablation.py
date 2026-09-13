#!/usr/bin/env python3
"""汇总问题3 A--E 消融运行JSON；只读结果，不接触模拟器或事后真值。"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="汇总问题3 A--E消融结果")
    parser.add_argument("summaries", nargs="+", type=Path, help="各局summary JSON")
    parser.add_argument("--output", type=Path, default=Path("res/tables/problem3/ablation_comparison.csv"))
    arguments = parser.parse_args()
    rows = []
    for path in arguments.summaries:
        data = json.loads(path.read_text(encoding="utf-8"))
        rows.append({
            "variant": data.get("settings", {}).get("ablation_variant", "unknown"),
            "seed": data.get("seed"),
            "cleared_over_total": data.get("cleared_over_total"),
            "clearance_ratio": data.get("clearance_ratio"),
            "total_virtual_time_s": data.get("total_virtual_time_s"),
            "movement_distance_m": data.get("total_movement_distance_m"),
            "measure_count": data.get("measure_count"),
            "switch_count": data.get("switch_count"),
            "clear_failure_count": data.get("clear_failure_count"),
            "summary_file": str(path),
        })
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(sorted(rows, key=lambda row: str(row["variant"])))
    print(arguments.output)


if __name__ == "__main__":
    main()
