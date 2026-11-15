"""Read-only finish-perception replay; never rewrites physical-run results."""
from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np

from go2_rescue.config import Config
from go2_rescue.perception import Perception
from go2_rescue.recording import replay
from go2_rescue.tracking import Tracker
from go2_rescue.types import Pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("Use a new/empty output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    config = Config.load(args.run / "config.json")
    if config.target_quota <= 0:
        parser.error("This comparison requires a declared positive target count")
    summary = json.loads((args.run / "summary.json").read_text())
    rows = {round(row["timestamp"], 4): row for row in
            map(json.loads, (args.run / "trajectory.jsonl").read_text().splitlines())}
    perception = Perception(config)
    # Zero reproduces the pre-fix unlimited reporting behaviour. It does not
    # change detection, data association, tracking or confirmation thresholds.
    baseline, fixed = Tracker(replace(config, target_quota=0)), Tracker(config)
    old_events, new_events = [], []
    compared, differences = 0, 0
    for frame in replay(args.run / "recording"):
        row = rows[round(frame.timestamp, 4)]
        if row.get("diagnostics", {}).get("course", {}).get("stage") != "finish_search":
            continue
        pose = Pose(**row["pose"])
        if pose.quality < config.min_pose_quality or row["reason"] in {"excessive_tilt", "stale_state"}:
            continue
        observations = []
        visible = frame.image is not None and frame.image_age <= config.max_sensor_age
        if visible:
            image = perception.undistort(frame.image)
            observations = [perception.locate_ball(d, frame, pose) for d in perception.detect_balls(image)]
        old_events.extend(baseline.update(observations, pose, frame.timestamp))
        new_events.extend(fixed.update(observations, pose, frame.timestamp))
        compared += 1
        same = len(baseline.tracks) == len(fixed.tracks)
        for old, new in zip(baseline.tracks, fixed.tracks):
            same &= all(np.array_equal(value, asdict(new)[key]) for key, value in asdict(old).items())
        same &= ((len(baseline.reported_positions) >= config.target_quota)
                 == (len(fixed.reported_positions) >= config.target_quota))
        differences += int(not same)
    original = [e for e in summary["events"] if e["kind"] == "target"]
    before, after = [asdict(e) for e in old_events], [asdict(e) for e in new_events]
    reproduced = before == original
    # Reuse error measurements only for the exactly reproduced original event.
    # This is offline scoring, never a perception or navigation input.
    original_errors = summary["evaluation"]["target_position_errors_m"]
    errors = [original_errors[original.index(event)] for event in after if event in original]
    passed = bool(reproduced and differences == 0 and len(after) == config.target_quota
                  and len(errors) == len(after) and max(errors, default=float("inf")) <= .5)
    report = {"scope": "recorded finish perception/tracking replay; NOT a new physical run or full Autonomy replay",
              "physical_run": str(args.run.resolve()), "physical_run_original_full_course_pass": summary["full_course_pass"],
              "physical_traversal_verified": summary["evaluation"]["ordered_traversal_verified"],
              "frames_compared": compared, "original_reports_exactly_reproduced": reproduced,
              "tracking_and_return_predicate_mismatches": differences,
              "before": before, "after": after, "after_target_errors_m": errors,
              "report_fix_replay_pass": passed,
              "limitation": "Declared-count cap does not establish unknown-count recognition accuracy"}
    (args.output / "report.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
