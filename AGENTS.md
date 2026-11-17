# Repository Instructions

## Source of truth
- The working tree, tests, and Git history are authoritative for implemented behavior.
- `docs/CURRENT_STATE.md` summarizes current progress; `docs/HANDOFF.md` is a session handoff, not proof of correctness.
- Never treat an unverified claim in documentation as an implemented feature.

## Read context selectively
- For a new development phase or cross-session continuation, read `docs/CURRENT_STATE.md` and, if present, `docs/HANDOFF.md` first. Inspect relevant code before making changes.
- Read `docs/ARCHITECTURE.md` for architectural changes; `docs/DECISIONS.md` when revisiting important design choices.
- For a small isolated edit, inspect only the affected files; do not read every document automatically.
- Do not ingest whole directories, large logs, datasets, or old transcripts unless needed.

## Engineering workflow
- Identify scope and acceptance criteria before nontrivial changes.
- Make focused changes; preserve unrelated user modifications.
- Run the smallest relevant tests first, then broader checks where practical. Distinguish passed, failed, and not-run tests.
- Do not claim a feature is complete without evidence.
- Ask before destructive commands, commits, pushes, deployments, or changes outside the requested scope.

## Persistent context
- Use the `project-handoff` skill for explicit checkpoint, handoff, and resume requests.
- After a significant milestone or architectural decision, update the relevant docs; not after every trivial edit.
- Keep handoff concise and actionable. Store detailed technical explanations in dedicated documents, not the handoff.
- Do not automatically invoke the handoff skill in every turn or create recurring documentation churn.

## Project-specific constraints
- V1 uses Go2 factory locomotion via the native SDK. Do not introduce ROS 2, RL training, pretrained gait policies or an ONNX submission workflow without a new user request.
- The user additionally authorized MuJoCo simulation with an open-source MPC controller. Simulation uses pinned go2-convex-mpc; preserve both native and simulation backends and the common Autonomy interface.
- Simulation root truth is for sensor generation/rendering/diagnostics only. Do not drive navigation from truth, overwrite root state during stepping, or label a derived local task scene as the unchanged official MJB.
- Hardware model and sensors are unknown. Keep calibrated=false in the supplied unknown-device template. Do not claim real-robot validation from synthetic tests.
- Preserve original physical/visual resources covered by SHA256SUMS. The user authorized consolidating runtime/status documentation into root README.md; original official README and SHA256SUMS are archived byte-for-byte in source-metadata/training-package-original/. Root SHA256SUMS redirects only its original README entry to that archive.
- The user subsequently authorized a local map variant moving the original east pipe2 ramp to the west entrance. --task-fixture now defaults to this labelled entry-ramp variant; --pipe-ramp official restores the original layout. Preserve source files and provenance; never count modified geometry as official full-course evidence.
- On 2026-11-16 the user accepted V1 ending at the current safe stop inside pipe2 and cancelled further exit-descent work. Preserve the drop guard and truthful incomplete-competition diagnostics; do not implement descent without a new request.
- Later on 2026-11-16 the user requested broad mapping/search for multiple football victims near rubble, removal of the finish-area football, and permits bypassing the pipe to finish. This supersedes the pipe-interior endpoint for the new search mission; preserve legacy ordered mode and its drop guard. Official target/termination transport is explicitly deferred by the user because no protocol was supplied; local task events must not be called official signals.
- Use .venv/bin/python; tests: .venv/bin/python -m pytest -q. Runtime entry: go2-rescue (src/go2_rescue/cli.py).
- Core units: metres, seconds, radians; body frame x forward/y left/z up; IMU quaternions wxyz; map frame is start-relative. Hardware observations must not include simulator truth.
- Native command output is opt-in. Do not run live/native with --execute against a robot without an explicit user request for a hardware motion test.
