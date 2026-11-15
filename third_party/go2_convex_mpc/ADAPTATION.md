# Pinned Go2 convex MPC integration

Upstream: https://github.com/elijah-waichong-chan/go2-convex-mpc
Commit: 1c63c6a762779887ab0431fd60db681dede6cb32
License: MIT; retained in LICENSE. Original hashes are in SOURCE.json.

The centroidal QP, discrete dynamics, gait scheduler and leg torque controller
are reused. In centroidal_mpc.py, com_trajectory.py, gait.py and leg_controller.py,
the PinGo2Model import was moved behind TYPE_CHECKING and postponed annotations
were enabled. This lets the same algorithms consume a MuJoCo-native model adapter
without installing Pinocchio or a second, different robot URDF.
The leg controller flattens the one-column contact mask before scalar indexing,
avoiding a deprecated NumPy array-to-scalar assignment without changing its values.

src/go2_rescue/mpc_locomotion.py provides that adapter using the supplied official
Go2 XML, IMU and joint observations, named joints/actuators, model mass/inertia,
contact-scheduled kinematic odometry, the upstream discrete centroidal dynamics,
and a local body-velocity reference. The local adapter additionally fits a
near-foot support plane from body-frame lidar. TerrainGait preserves upstream
swing XY/timing and adjusts touchdown Z; the COM reference follows the plane's
height, roll and pitch. Vertical velocity from stance kinematics is integrated.
This is a project-specific plane adaptation, not an upstream full-body stack;
future foot levers in the QP still use the current measured levers.

Runtime tuning: friction=0.4, swing height=0.065 m, 2.5 Hz trot with 0.65 duty,
16 prediction stages, approximately 42 Hz QP updates, approximately 167 Hz leg
updates, 500 Hz official MuJoCo integration. These values have only been tested
in the flat start area and the focused official-ramp climb documented in
docs/MUJOCO.md. This does not establish traversal of all obstacles or slopes.

The original go2_robot_data.py and mujoco_model.py remain available as upstream
reference, but are not executed; no upstream example's scripted commands or
teleport-style replay is used by the autonomous simulation mode.
