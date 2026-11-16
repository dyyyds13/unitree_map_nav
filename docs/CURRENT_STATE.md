# Current State

2026-11-16, Asia/Shanghai. Original source workspace had no Git metadata; publishing checkout preserves the existing GitHub history. No real-hardware execution. Read AGENTS.md first.

## Documentation consolidation and next-route requirement

2026-11-16: user requested one concise root README combining runtime instructions, actual progress, limitations and responsibilities. README_V1.zh-CN.md is merged and removed; original official README/SHA256SUMS are byte-for-byte archived in source-metadata/training-package-original/. Root checksum list changes only the original README path. Physical/visual assets and algorithm code are unchanged. Local Git checkout was merged to main b245201 via PR1; the user has now authorized publishing the consolidated README and supplied route illustration. Verify the publication commit from Git.

The supplied image is copied unchanged to docs/images/ordered-route-plan.png and captioned as an early original-layout plan, not a completed run.

User reports organizers permit bypassing the pipe. This changes the proposed next route, not existing code: current ordered route still requires pipe2 and stops under its drop guard. No descent implementation requested. Python3.10.22 remains the validated baseline;3.12 and autonomous coverage exploration are unvalidated. See README.md for concise scope and team responsibilities.

## Accepted V1 scope and latest verified result

2026-11-16: the user explicitly ends V1 at the current safe stop inside circular pipe2. Do not implement the east22cm descent in this delivery. Subsequent pipe exit, far-finish ball/report and official finish remain outside this accepted milestone; do not relabel the runtime as a completed competition mission.

Latest physical evidence: output/entry-ramp-original-start-07, original start(.8,.8), local entry-ramp variant,260sim seconds/311.594wall seconds. Autonomy passed ascent, descent, S_curve, dead_end and rubble; entered pipe at197.8s, reached the explicit zero-speed exit guard at203.0s, and remained stopped inside. Final one-second net speed0.000395m/s; continuous2ms audit:0obstacle contacts, no fall, no water, no bounds violations. Runtime ended at its260s time limit, mission_complete=false/full_course_pass=false. All recording flags off. Small portable evidence is in docs/validation/entry-ramp-original-start-07/; large local output logs are excluded from Git.

Local entry scene selects configs/local-entry-ramp-route.json: flat approach alignment, bounded26degree classification region(actual ramp18.7degrees), .22m/s pipe cap, full oriented-body observed corridor planning,20cm waypoint tolerance and short safe near-goal fallback. At pipe entry a sensor-only terrain submap removes accumulated height-reference error while retaining the global map/water evidence. DWA uses accepted scan motion in the corridor. Ground/roof separation preserves absolute headroom; special-region MPC uses measured per-foot surfaces,1.5Hz/75% stance and longitudinal scan feedback. Original2.5Hz MPC remains elsewhere; global14degree/8cm limits and all measured obstacle/support/water checks remain. No truth navigation or root state overwrite. Official files and original-route capability guards remain unchanged.

Final full suite:117passed in47.03s; replacement checkout repeated117passed in70.38s. Compile, pip consistency and original SHA checks passed; GUI4s auto-close/camera smoke passed. No hardware motion or active simulation. See docs/ENTRY_RAMP_NAVIGATION.md.

Repository delivery: user requested replacing the existing GitHub unitree_map_nav contents through a history-preserving merge. Source workspace remains intact; staging checkout is /home/l/下载/unitree_map_nav on replace/go2-rescue-v1-20261116, based on main d7a7d1fdd4bed9f4fee8b00dc5a56b69af3250f6. Virtualenv, output, recordings and caches are not deliverables.

## Prior user-authorized local map + front camera snapshot

User now explicitly requested moving the east pipe2 ramp to the west entry. This supersedes the prior static-map restriction ONLY for the labelled local variant. Source SHA-covered assets unchanged. task_scene(pipe_ramp=entry) recompiles one180degree world transform around tube2 centre; full other-geometry/expected-transform vertex audit, provenance geometry_changes list. --task-fixture defaultsentry; --pipe-ramp official retains previous static layout, no-fixture still originalMJB. AGENTS/DECISIONS updated. Never count modified geometry as official full_course_pass.

West ramp spans aboutx7.95→8.60 at y6.22, rises0→.22m; original east ramp gone, east exit now22cm drop. Original slope/dimensions unchanged. Pipe autonomous capability guard remains; not a validated whole-pipe route. docs/LOCAL_MAP.md.

--viewer now shows real10Hz navigation camera inset bydefault, up to480x360 display, BGR→RGB, resized with viewport; --hide-camera disables. No additional camera render/video/raw/physics/snapshot/archive enabled. Handle.set_images uses existing frame. Actual screen visually inspected.8s pipe-approach scripted stage entry-ramp-camera-01:75camera updates,433syncs,8.008wall,no fall/recordings; original-start autonomous entry-ramp-autonomy-01:12sim/17.935wall under concurrent tests,115camera updates,562syncs,no fall/recording. Neither is pipe/full-course proof.

Frozen-view Ctrl+C preview first exited139; fixed teardown by clearing overlays and waiting for pinned3.17 Handle's Simulate weakref to expire after close(exit request), preserving model/image lifetimes through render-thread destruction. entry-ramp-close-01 Ctrl+C and autonomy-01 automatic close both exit0.107tests43.50s plus resized-inset test passed; SHA/pip/compile passed. No active simulation. GUI previewXWD/PNG in/tmp was manual QA only, automatic image output remains off.

## Latest dead-end response and pipe geometry audit

2026-11-15. Explicit reverse ordered segments now use separate max_reverse_vx(sim default.25m/s, hardware.08); --max-reverse-speed overrides independently. Ordinary ordered nudges/SE2 recovery keep.08, margin escape.05, marked portal reverse.15. Retain original velocity sample endpoints and all body/support/water/braking checks. Development trial02 changed endpoints and fell on ascent25.518s; final restored the original sampling and constrained speed to intended reverse segments.

Frontal evidence collected during approach,3consecutive scans; inspection region.25m(previous.15), stationary window averaging plus pose stability,1s hold retained. Root cause of slow hold:millimetre jitter across arrival boundary continually reset its timer. checkpoint_near() supplies shared2cm hold hysteresis to Course and Autonomy; large motion still resets. Diagnostics log measured twist and front confirmation.

output/deadend-speed-final:original start130sim/132.888wall, passed ascent/descent/S/dead_end/both return portals, continued to rubble waypoint6.0diagnostic obstacle/water/bounds/fall, all recordingFalse, no video/images/physics/scene binary. User baseline run-20261115-222501:dead_end69.9→141.1s(71.2s); final69.9→108s(38.1s), first retreat7.8→1.9s, starts reversing83.4→75.2s. Front confirmed72.9s. NOT full-course/hardware validation.105tests43.44s, SHA/pip/compile passed; no active simulation. docs/DEAD_END.md.

Original pipe geometry read-only audit:official MJB equals training-package-v8.zip entry bySHA256. Outside floor0, inside flat insert.2200000025m, actual22cm curb belongs to supplied official model. docs/PIPE2_MODEL_AUDIT.json. User baseline reachedpipewp0 and stopped intentionally at8cm capability guard; autonomous22cm traversal still unintegrated. Do not lower source geometry or imply this reverse patch solves the pipe.

Earlier output directories(including pipe2-stage-10 and smooth-junction-final) are absent from the current working tree; their descriptions below are historical records, not presently inspectable physical evidence. Use current user baseline and deadend-speed-final for latest directly available measurements.

## Latest performance and S-junction fix

2026-11-15 late session: video/raw sensor capture/physics archives/map-image snapshots now explicit opt-ins. Default run retains only small config/provenance/route/runtime/trajectory/summary files; no runtime-scene.mjb export unless --save-physics or --snapshots. Frame history removed; rolling timings bounded. Viewer sync decoupled from10Hz video, wall-clock cap60Hz. Camera remains10Hz for navigation. Simulation scopes BLAS to1thread/restores on exit; batched ground medians preserve original results. threadpoolctl3.6.0 added to simulation extras/lock.

Current physics/MPC/navigation/OpenCV are CPU; actual GL_RENDERER on this RTX3060 machine is NVIDIA GeForce RTX3060 Laptop GPU. smooth-viewer-02 actual GUI smoke:8sim/8.010wall,434syncs(~54.2/s), no recordings/fall. Not autonomous or a60fps guarantee.

User fast-20261115-215744/220055 runs stalled at S_curve waypoint4: safe-turn rejected and alignment region too small. Route prior moves that alignment point25cm earlier to the bay; 1.5s turn sweep no longer overshoots requested heading. Marked portal margin restored to min(safety_margin,.02), full body/foot-water checks remain. Official static map unchanged. smooth-junction-01 original-start120sim/106.591wall passed S and dead-end enter/exit; smooth-junction-02 intermediate version passed both return-portal points and visited rubble, then deliberately interrupted151.8sim for final GUI verification,0diagnostic incidents. Final GUI smooth-junction-final PASSED the requested junction segment:155sim/167.889wall,5067syncs(mean30.18/s), completed ascent/descent/S/dead_end and both return portals(145.3/148.4s), reached rubble waypoint3.0diagnostic obstacle/water/bounds/fall; stopped by155s time limit, NOT whole-course success. All recording flagsFalse, no media/physics/scene binary;1.24MB small diagnostics. No active simulation. Full suite99passed42.06s; pip/compile/SHA checks passed. docs/PERFORMANCE.md.

## Latest speed change

User requested about0.5m/s. Simulation defaults now max_vx=.5 and passage_max_vx=.5; --max-speed overrides both, explicit config-file speeds are honored. Hardware defaults .3/.08 and calibration gating unchanged. DWA lookahead includes braking distance(up to1.2m), retaining body/ground/hazard/braking constraints. Original2.5Hz MPC gait and torque limits remain; experimental fast gait/force bounds were removed.

speed-05-flat-05 and speed-05-u-exit-02:8sim-second scripted controller stages with continuous2ms evaluation, actual cruise means .596/.582m/s, no contacts/fall/water/bounds and stopped net speeds .036/.012m/s. speed-check.json files pass. These are not autonomous missions. speed-05-autonomy-01:30s original-start Autonomy, commands .48-.50, ascent stage complete26.2s, actual ascent/platform contact, no incidents; stopped at time limit in L1_descent, not a full pass. First straight-east tests01/02/03 approached an official curved wall and fell; preserve raw failures, do not blame the gait from that unsuitable strip.

Final suite96passed36.38s. docs/SPEED.md has commands, limits and evidence. The earlier speed-only change preserved a passage margin expression min(safety_margin,.2); the latest junction fix supersedes it with .02. Curb entry clip remains[-.3,.45], NOT the .045 used in the historical clean tube test. This faster curb clip has not been validated and ordinary --max-speed does not integrate or bypass curb capability. No active simulation.

## Historical full-course requirement (superseded by accepted V1 scope above)

The user explicitly requires CIRCULAR TUNNEL2. Required route: original start before ramp(0.8,0.8,yaw0) → L1 ascent/platform/descent → S bend → dead-end inspection/reverse exit → rubble region avoiding piles → tunnel2 west-to-east including22cm curb/descending exit → far-finish ball/report/stop. Official static geometry must remain unchanged. Real factory SDK and MuJoCo pinned go2-convex-mpc remain separate backends. No ROS2/RL/training added; unknown hardware calibrated=false.

configs/official-course-route.json now includes the required tunnel2 approach/traversal/exit and declares22cm step capability, measured elevation/distance evidence. This is an explicit manually authored fixed-public-map prior, not unknown-layout generalization. The read-only evaluator specifically audits tube2 floor contact and west-to-east traversal; tunnel3 cannot pass. It also audits body/leg/roof contacts. Foot spheres FL/FR/RL/RR are attached to calf bodies, not the fixed *_foot frames.

## Pipe2 work

src/go2_rescue/step_terrain.py and ConvexMPC.enable_curb() implement opt-in sensor-only curb detection, lifted-then-translated swings, different front/rear placement timing and body-height/pitch references. Final candidate uses1Hz diagonal gait; initial slow single-leg crawl failed. The historical clean candidate capped entry vx0.045m/s(current workspace clip0.45 is unvalidated); after entry returns to standard gait and lidar support-plane adaptation(up to30degrees only in this opt-in mode). There is no plant-root pose/velocity input, runtime pose overwrite or root force.

scripts/test_pipe2_controller.py is an explicitly labelled PARTIAL-STAGE controller diagnostic: reset once at(8.1,6.22,.30), scripted forward intent plus measured heading/joint-odometry feedback and estimated-distance stopping; NOT Autonomy, original start, random layouts or hardware. It produces actual physics/video. Truth is rendering/read-only evaluation/stop checks only.

- pipe2-stage-01/02/04/05 fell;03 entered but did not exit before95s. Preserve failures.
-06 exited at55s;07 additionally stopped at61.4s. These lacked continuous collision scoring; their10Hz offline contact audit is insufficient for a clean-pass claim.
-08 exited and stopped,61.402sim seconds, no fall/water/bounds, but continuous2ms contact checking caught one short leg contact at22.832s(tunnel2_segment_16):stage_pass=false.09 still had that contact despite earlier hind placement. Raw results stay unchanged.
-10 PASSED the isolated stage:62.102sim seconds,107.464wall seconds under concurrent test load, entered/traversed/exited and stopped; continuous2ms audit0obstacle contacts/0water/0bounds/no fall. stage_pass=true, full_course_pass=false. Hind lead-.085(previous09-.055,08-.025). This is one specified reset/phase/noise case, not multi-pose robustness or autonomous traversal. Video/physics/source snapshots/provenance archived for10. See docs/PIPE2.md.

IMPORTANT: autonomous step traversal is NOT INTEGRATED. Default8cm navigation capability remains conservative. At pipe stage Autonomy returns blocked/ordered_pipe2_requires_validated_step_capability. It does not silently route through3, clear the curb map, or enlarge global terrain limits. Do not tell the user the default whole-course command now passes. Remaining necessary work is robust entry/exit over pose/phase variations, a collision-checked3D navigation/locomotion interface, then a fresh original-start full run.

## Historical full-route evidence

output/ordered-course-13 traversed the OLD TUNNEL3 route from original start in475.102sim/981.808wall seconds: seven old stages, then finish;0old-evaluator obstacle/water/bounds/fall. Raw strict full_course_pass=false because reports had0.06897m and0.96256m errors. This is NOT evidence for required tunnel2 and must not be rewritten.

Tracker caps reports at a positive declared target_quota, unknown count0 unlimited. output/ordered-course-13-report-replay/report.json replays226finish frames with archived estimated poses, reproduces both original reports, corrected single report error0.06897m; state/return mismatches0. It is component replay, not a new physical/full Autonomy run. This does not solve unknown-count visual false positives or guarantee first-report correctness.

ordered-course-14 was interrupted on user confirmation of mandatory2, during L1_descent; raw logs saved. It is not a completed post-report-patch full run.

## Existing navigation and limitations

Point-to-line ICP with optional stable start-relative IMU heading; overhead planes excluded ONLY from planar matching, retained in full3D mapping. Coverage/inlier-aware quality avoids undercounting intentional ceiling removal. Unknown hardware does not enable stable-yaw assumptions.

Ramp/S/dead-end grid0.10m; at rubble rebuild0.05m from sensor keyframes/estimated poses. Historical U-exit narrow mode used0.02m margin and0.08m/s; current workspace uses the restored2cm portal margin described above and configurable forward cap(defaultsim0.5), preserving physical body/water checks. Normal extra margin0.05m resumes afterward. Pose-region SE(2) recovery, bounded buffer-only escape, transit gates and strict checkpoint headings remain.

Ramp recovery is slow and estimated ground consistency is imperfect. Historic full-run navP95.1853s, so end-to-end10Hz is not established. No80%success, formal scoring, world-coordinate protocol, hardware calibration or native motion validation.

## Checks and entry points

Latest full suite:117passed47.03s after entry-ramp integration;105passed43.44s after the dead-end response change;96passed36.38s was the speed change;90passed68.63s was the earlier tube-stage milestone. No active simulation. Official SHA256SUMS, pip check and compileall passed this turn. Scene SHA256:2dd875d238040d9a8616869cb2e30d00abf131b48b89b011af6dc10bd264089c.

README.md, docs/PIPE2.md, docs/COMPETITION_V1.md, docs/ARCHITECTURE.md, docs/VALIDATION.md. Use .venv/bin/python from project root and new output directories. Never label a custom-reset stage as full-course success.
