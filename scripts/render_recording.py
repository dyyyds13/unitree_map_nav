"""Render a sensor recording + controller log as a labelled software-run video."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from go2_rescue.config import Config
from go2_rescue.perception import Perception
from go2_rescue.recording import replay


def main():
    p = argparse.ArgumentParser()
    p.add_argument("run", help="Directory containing config.json, trajectory.jsonl and recording/")
    p.add_argument("--output", required=True)
    args = p.parse_args()
    root, output = Path(args.run), Path(args.output)
    if output.exists():
        p.error("Output already exists")
    rows = [json.loads(line) for line in (root / "trajectory.jsonl").read_text().splitlines()]
    frames = list(replay(root / "recording"))
    if len(rows) != len(frames) or not rows:
        p.error("Recording/log lengths do not agree")
    config = Config.load(root / "config.json")
    perception = Perception(config)
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), 10., (800, 360))
    if not writer.isOpened():
        p.error("Video encoder unavailable")
    recent = []
    try:
        for row, frame in zip(rows, frames):
            canvas = np.full((360, 800, 3), 28, np.uint8)
            if frame.image is not None:
                image = perception.undistort(frame.image)
                for detection in perception.detect_balls(image):
                    x, y = map(int, detection.center)
                    cv2.circle(image, (x, y), int(detection.radius), (80, 230, 80), 2)
                canvas[70:310, 20:340] = cv2.resize(image, (320, 240))
            recent.extend(e["kind"] for e in row["events"])
            lines = ["SOFTWARE SENSOR FIXTURE", "Not a real robot / physics test",
                     f"t = {row['timestamp']:.1f}s  state: {row['state']}",
                     f"pose quality: {row['pose']['quality']:.2f}",
                     f"vx={row['velocity']['vx']:.2f} vy={row['velocity']['vy']:.2f}",
                     f"yaw_rate={row['velocity']['wz']:.2f}",
                     f"events: {', '.join(recent[-4:])}", row["reason"] or "planning"]
            for i, line in enumerate(lines):
                x, y = (20, 30) if i == 0 else (365, 65 + (i - 1) * 35)
                cv2.putText(canvas, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .47, (220, 220, 220), 1, cv2.LINE_AA)
            writer.write(canvas)
        cv2.imwrite(str(output.with_suffix(".png")), canvas)
    finally:
        writer.release()
    print(output.resolve())


if __name__ == "__main__":
    main()
