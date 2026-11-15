# Sources and dependencies for V1

The official resource package retains its original NOTICE.md and LICENSES unchanged.
The implementation does not redistribute pretrained RL weights. Navigation is implemented locally; the simulation MPC source is vendored separately with its license.

- go2-convex-mpc (MIT): https://github.com/elijah-waichong-chan/go2-convex-mpc
  Pinned commit 1c63c6a762779887ab0431fd60db681dede6cb32. See third_party/go2_convex_mpc/LICENSE, SOURCE.json and ADAPTATION.md.
- MuJoCo 3.17.0: https://github.com/google-deepmind/mujoco
- CasADi 3.6.7: https://web.casadi.org/
- OSQP 1.1.3: https://osqp.org/

- Unitree SDK2 Python (optional hardware dependency): https://github.com/unitreerobotics/unitree_sdk2_python
  API shape checked against the official SportClient and VideoClient examples. Install the version compatible with actual firmware and retain its license.
- KISS-ICP 1.3.0 (optional odometry backend, MIT): https://github.com/PRBonn/kiss-icp
- NumPy 1.26.4: https://numpy.org/
- SciPy 1.13.1: https://scipy.org/
- OpenCV contrib Python headless 4.10.0.84: https://github.com/opencv/opencv-python
- PythonRobotics (algorithm reference, not vendored): https://github.com/AtsushiSakai/PythonRobotics
- Project handoff starter: user-provided /home/l/下载/codex-handoff-starter.zip.

Installed dependency versions are recorded in requirements-v1.lock.txt. Dependency packages carry their own upstream licenses.
