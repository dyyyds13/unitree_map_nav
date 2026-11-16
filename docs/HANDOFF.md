# Session Handoff

2026-11-16, Asia/Shanghai. Read AGENTS.md and docs/CURRENT_STATE.md first. No real-hardware execution or active simulation.

Latest documentation: root README.md consolidates runtime/status/work allocation; README_V1 has been removed. Official README/checksum originals are archived in source-metadata/training-package-original/; root checksum redirects only README. User reports pipe bypass is permitted; current code has not switched routes. No descent work. Python3.10.22 remains baseline. Git checkout main b245201 already contains the prior PR1 delivery; the user authorized publishing the consolidated README plus docs/images/ordered-route-plan.png (early plan, not a completed run); verify the latest publication from Git.

## Accepted scope

The user ends V1 at safe parking inside circular pipe2. Do not implement the east22cm descent for this delivery. Preserve the explicit ordered_pipe2_exit_requires_validated_descent_capability zero-speed guard. This is not a completed official competition mission or full pipe traversal.

The user requested replacing https://github.com/liangjihui643-beep/unitree_map_nav.git contents using a merge while preserving repository history. Local source workspace must remain intact. Staging checkout: /home/l/下载/unitree_map_nav; replacement branch: replace/go2-rescue-v1-20261116, based on main d7a7d1fdd4bed9f4fee8b00dc5a56b69af3250f6. Verify GitHub PR/merge status from Git rather than inferring success from this preparation note. Exclude .venv/output/recordings/caches and credentials; include runnable source, configs, tests, pinned third-party MPC and original SHA-covered resources.

## Current evidence

Final original-start physical run: output/entry-ramp-original-start-07,260sim/311.594wall seconds. Passed ascent/descent/S/dead_end/rubble, entered the local pipe at197.8s, stopped under guard at203s. Continuous2ms audit: no obstacle contacts/fall/water/bounds. Final1s net speed0.000395m/s. All recording flags off. Runtime time_limit and mission_complete/full_course_pass=false are retained. Portable summary and derived evidence: docs/validation/entry-ramp-original-start-07/. Full117tests passed47.03s, and the replacement checkout repeated117passed70.38s; original SHA, compile and pip checks passed. Hardware remains unvalidated/calibrated=false.

## Implementation details to preserve

Local ramp scene/route is explicitly modified; official MJB/SHA assets unchanged. Entry26degree classification region(actual18.7),8cm step/global14degree constraints,20cm arrival regions, .22m/s pipe cap. Only declared corridor uses full oriented footprint/2cm margin, accepted scan velocity, short safe near-goal fallback and sensor-only local terrain submap; global map/water retained. Ceiling absolute heights correct headroom after floor changes. Local slope MPC uses per-foot lidar support/friction normals,1.5Hz/75% stance and longitudinal scan feedback; original2.5Hz solver elsewhere. No truth navigation/root overwrites. Native backend remains factory SDK, no ROS2/RL/training.

Default simulation forward.5m/s, explicit reverse.25m/s. Default no video/raw/physics/snapshots/scene archive; viewer reuses10Hz front camera inset. Viewer teardown retains storage until render cleanup. See README.md, docs/ENTRY_RAMP_NAVIGATION.md, docs/LOCAL_MAP.md and docs/DECISIONS.md. Preserve failed historical evidence; do not call isolated resets full-route validation.
