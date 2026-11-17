# 死胡同回归与依赖版本审查

最新进展：Python3.12环境下调整前墙检查目标与靠近速度后，两次复测已退出死胡同。详见 [参数调整与验证](DEAD_END_TUNING.md)。下文保留原始故障审查；尚未确定历史成功/失败首次26.9秒分歧的触发原因。

后续用户要求回退：已恢复Python3.10及成功记录中的相机快速渲染行为；下面的3.12当前环境、相机撤回状态属于此前审查记录。当前回退与长程验证见 [回退记录](ROLLBACK.md)。

2026-11-16，针对切换Python3.12后死胡同未能退出的问题，审查当前两个保留环境及实际运行记录。

## 结论

本次审查不能排除先前代码或运行环境改动。最初的3.10/3.12对照使用同一份改过的源码，只能检验该配置下的解释器差异，不能替代改动前后对照。

未发现依赖声明、版本组合或导入方面的冲突。固定条件下，Python3.10.22和3.12.3均复现同一故障，不能将当前死胡同卡住归为3.12独有的不兼容；也没有证据支持通过随意降级NumPy/CasADi来解决。

这不解释历史成功与失败运行首次分歧的触发原因。此前119项测试和10秒短测不能证明死胡同长路线通过，之前的3.12短测结论仅在其明确范围内有效。

## 环境审查

- 两个实际解释器分别运行pip check，均返回 `No broken requirements found`。
- 所有已安装运行依赖及项目包版本一致。唯一版本差异是安装工具pip：3.10环境23.0.1、3.12环境24.0。
- 实际导入并运行MuJoCo3.17.0、CasADi3.6.7、NumPy1.26.4、SciPy1.13.1、OSQP1.1.3和OpenCV4.10.0.84。
- 两个环境加载的NumPy OpenBLAS均为0.3.23.dev，SciPy OpenBLAS均为0.3.27；记录的CPU架构选择也相同。安装包的cp310/cp312标签匹配各自解释器。仿真摘要均记录BLAS限制为1线程。

完整环境、wheel标签和构建信息保留于本地 `output/version-audit/py310.environment.json` 和 `py312.environment.json`。

## 对照运行

同一当前源码、原起点(.8,.8)、seed0、PYTHONHASHSEED0、ordered路线、task-fixture入口坡、fast-render、D3D12/GLFW传感器后端，运行130仿真秒。关闭交互窗口和实时等待以排除窗口交互；不改变物理/传感器/导航频率。两个独立进程使用不同输出目录，并行运行的墙钟时间不用于速度比较。

| 条目 | Python3.10.22 | Python3.12.3 |
|---|---|---|
| 运行结束 | 130秒时限 | 130秒时限 |
| 阶段/检查点 | dead_end / index1 | dead_end / index1 |
| 前墙已确认 | 是 | 是 |
| 状态 | recovering | recovering |
| 异常/跌倒 | 无/无 | 无/无 |
| 轨迹记录数 | 1294 | 1294 |

删除每条轨迹中仅统计耗时的 `processing_seconds` 后，1294条完整JSON记录精确相同，包括时间戳、位姿、运动指令、事件、规划诊断和只读物理位置。它们也与用户失败运行 `output/run-20261116-183129` 的相同时间前缀精确相同。规范化记录SHA256：`fb271925e82eec640bbb485b64c5d89fda4fb26722588c8c78f0fce5911bb33e`。

本地证据：`output/version-audit/py310/`、`py312/`、`audit-summary.json`、`exact-comparison.json`。结论限于这一同配置/同种子测试，未宣称所有版本组合或场景均等价。

## 历史差异与卡住位置

成功记录 `run-20261116-171847` 与失败记录 `run-20261116-183129` 的config.json相同，均启用fast-render及D3D12。前者93.9秒完成dead_end，后者260秒仍在index1。两个历史summary未记录Python解释器版本，不能仅凭它们独立证明版本因果。

两次保存的 `ordered-course.json` 与当前 `configs/local-entry-ramp-route.json` 字节一致，SHA256均为 `1895c62957fb710995e324328c61402a1f545d91e136300c47880605dc7f3428`。启动日志的起点、初始朝向、ordered策略、场景种类和260秒时限也一致。历史记录未包含源码快照或完整环境快照，仍不足以恢复所有运行条件。

逐帧比较：约26.9秒（L1_descent）首次出现物理位置、估计位姿和指令差异，27.6秒开始出现规划reason差异；因此偏差在进入死胡同之前已经发生。原始传感器、逐物理步状态未录制，现有历史日志不足以确认这一早期分歧的触发机制。

受控失败运行末帧129.9秒：估计位置(3.5066,3.6501)，前墙检查点目标(3.55,3.2)，相距约0.452米，大于配置的0.25米到达阈值。前墙已确认359次，但未完成检查点到达/停稳条件，不能推进到下一倒退检查点。dead_end记录中包含218次 `ordered_no_observed_route`、291次 `ordered_se2_recovery` 和5次 `ordered_plan_recovery`。代码对应 `course.checkpoint_near()` 与控制器的无路径/恢复分支。

目前可以确定卡住时的表现是估计位姿、观测地图、可达性规划与严格检查点条件之间的闭环未推进；这尚不能解释成功记录变为失败记录的触发原因。

## 先前改动核对与撤回

用户确认以 `output/run-20261116-171847` 为恢复基准：93.9秒完成死胡同进退，随后进入管道。

- 仿真源码：改过窗口刷新调度，增加 `--fast-render`、窗口耗时统计；该选项此前也关闭了传感器相机的阴影/反射。
- 本机启动脚本：设置D3D12/NVIDIA渲染后端、窗口15Hz上限并开启快速渲染。
- Python迁移：切换虚拟环境链接到3.12，保留3.10环境；依赖锁定版本不变。
- 当前已撤回 `--fast-render` 对传感器相机的影响。它只影响窗口和俯视视频，summary新增 `compute.sensor_rendering=original`。新增回归测试逐帧比较该开关两种状态的RGB、点云、IMU与物理终态。未调整死胡同到达阈值、导航路线或安全限制。

另外直接从Git HEAD提取原始 `mujoco_sim.py`，在独立进程载入，未覆盖工作区；不使用快速渲染、窗口或实时等待，原起点/ordered/task-fixture/seed0，运行130仿真秒。D3D12/GLFW和llvmpipe/EGL两种相机后端均仍卡在dead_end检查点1。证据位于 `output/change-audit/git-original/`、`git-original-software/`，脚本为 `output/wsl-setup/audit_original.py`。这两组使用当前3.12环境及其余HEAD一致的算法源码，**不等于完整恢复历史成功进程的环境与状态**，不能据此宣布先前改动无关或故障已修好。

## 撤回后的实际验证

Python3.12.3、D3D12/GLFW、真实窗口15Hz上限、fast-render仅显示、原起点/ordered/task-fixture/seed0运行130仿真秒；未使用实时等待。结果仍为dead_end检查点1，前墙已确认，未退出；未跌倒、无异常，诊断障碍接触/进水/越界均0。与HEAD原始仿真代码对照的1294条记录去掉计算耗时后完全一致；与历史失败记录的物理位置、估计位姿、速度指令及reason也逐条相同。相机撤回改变了部分感知诊断，但没有恢复本例退出行为。

证据：`output/change-audit/reverted-sensor-py312/`、`output/change-audit/rollback-comparison.json`；脚本 `output/wsl-setup/audit_reverted.py`、`compare_rollback.py`。完整软件测试119 passed in69.84s，原始资源SHA校验通过，`git diff --check`通过。**软件测试通过不代表死胡同回归已解决；目前尚未恢复用户确认的成功基准。**
