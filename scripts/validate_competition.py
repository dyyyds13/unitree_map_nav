"""Repeat the local fixed-layout competition fixture and aggregate real results."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=600.)
    parser.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2])
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 600:
        parser.error("--seconds must be in (0,600]")
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("Use a new/empty output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    root = Path(__file__).resolve().parents[1]
    for index, seed in enumerate(args.seeds):
        folder = args.output.resolve() / f"run-{index:02d}-seed-{seed}"
        command = [sys.executable, "-c", "from go2_rescue.cli import main; main()", "mujoco",
                   "--task-fixture", "--course-policy", "ordered", "--seconds", str(args.seconds), "--seed", str(seed), "--output", str(folder)]
        if args.record:
            command.append("--record")
        print(f"Running local fixed-layout case {index+1}/{len(args.seeds)}, sensor seed={seed}", flush=True)
        with (args.output / f"terminal-{index:02d}.log").open("w") as terminal:
            process = subprocess.run(command, cwd=root, stdout=terminal, stderr=subprocess.STDOUT)
        summary_path = folder / "summary.json"
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        evaluation = summary.get("evaluation", {})
        results.append({"seed": seed, "returncode": process.returncode,
                        "mission_complete": summary.get("mission_complete", False),
                        "local_task_pass": evaluation.get("local_task_pass", False),
                        "full_course_pass": summary.get("full_course_pass", False),
                        "ordered_traversal_verified": evaluation.get("ordered_traversal_verified", False),
                        "stop_reason": summary.get("stop_reason", "startup_failure"),
                        "simulation_seconds": summary.get("simulation_seconds"),
                        "target_error_p90_m": evaluation.get("target_error_p90_m"),
                        "water_entry_episodes": evaluation.get("water_entry_episodes"),
                        "obstacle_contact_episodes": evaluation.get("obstacle_contact_episodes"),
                        "out_of_bounds_episodes": evaluation.get("out_of_bounds_episodes"),
                        "fell": summary.get("fell"), "results": str(folder)})
        report = {"scope": "same supplied geometry; seeds change sensor noise only; not official scoring or random layouts",
                  "cases": results, "passed": sum(r["full_course_pass"] for r in results), "total": len(results)}
        (args.output / "validation.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps(results[-1], ensure_ascii=False), flush=True)
    lines = ["# Local competition regression", "", report["scope"], "",
             f"Passed: {report['passed']}/{report['total']}", "",
             "| Seed | Mission finished | Full ordered course passed | Stop reason | P90 error (m) | Water | Contacts | Fall |",
             "|---|---|---|---|---|---|---|---|"]
    for result in results:
        lines.append("| " + " | ".join(str(result.get(k)) for k in ["seed", "mission_complete", "full_course_pass",
                         "stop_reason", "target_error_p90_m", "water_entry_episodes", "obstacle_contact_episodes", "fell"]) + " |")
    (args.output / "validation.md").write_text("\n".join(lines)+"\n")
    return 0 if report["passed"] == report["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
