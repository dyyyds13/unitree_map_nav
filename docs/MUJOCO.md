# 官方 MuJoCo 场景与开源 MPC

本版本已经把同一套 Autonomy 感知、定位、建图、探索、规划和任务代码接入真实 MuJoCo 动力学。
实物仍用 Go2 厂家运动服务；仿真使用开源凸 MPC 输出12路关节力矩。两者共用机体速度接口，不需要ROS 2、RL或训练。

最新变更：用户授权将2号管道出口坡移到当前西侧入口，`--task-fixture`默认加载标记清楚的本地修改版；`--pipe-ramp official`保留原布局。右下角默认显示实时前视相机，不录制。详见[本地场景说明](LOCAL_MAP.md)。

## 直接运行

在项目根目录执行。当前 `.venv` 已安装 MuJoCo 3.17.0 和 CasADi 3.6.7。

原始比赛 MJB，自主探索（默认不录制）：

```bash
.venv/bin/go2-rescue mujoco --seconds 20 --output output/my-official-run
```

桌面可视化窗口（在有DISPLAY的桌面终端运行）：

```bash
.venv/bin/go2-rescue mujoco --viewer --realtime --seconds 60 --output output/my-viewer-run
```

远端终点任务（默认本地入口坡＋新增足球和实验终点）：

```bash
.venv/bin/go2-rescue mujoco --viewer --task-fixture --seconds 300 --output output/my-finish-task
```

按用户要求，黑白球现位于远端终点隔墙内的世界坐标 `(11.35, 9.05, 0.11)`，实验终点标记位于 `(11.00, 8.80, 0.43)`；机器人仍从 `(0.8, 0.8)` 出发。
终点区域根据模型内具名finish隔墙确定；ArUco仍是本地实验标识，不是赛方正式结束协议。
默认场景配置不再使用20秒短任务预算，而使用600秒预算、60秒返程预留；`--seconds` 单独控制本次运行上限。300秒运行命令不承诺能完成全场导航。
位置只用于场景生成、渲染和离线诊断，不提供给自主导航。

需要复现旧的近距离回归时显式指定：

```bash
.venv/bin/go2-rescue mujoco --task-fixture --task-layout near-start --seconds 22 --output output/my-near-start-regression
```

单独检查 MPC 的站立、前进、转向和速度停止：

```bash
.venv/bin/go2-rescue mujoco --control smoke --seconds 8 --output output/my-mpc-smoke
```

每次使用新的输出目录。默认无窗口，通过EGL离屏渲染；`--viewer`需要桌面OpenGL/GLFW。
工具环境未默认设置DISPLAY；本次通过本机 `DISPLAY=:0` 和桌面Xauthority完成了实际交互窗口运行验证，另已验证EGL离屏渲染/视频。
`--realtime`仅在计算足够快时限制超前，不能保证当前实现实时运行。

### 任务结束与终端诊断

`--seconds` 是仿真时间上限，任务完成会提前停止。旧版起点短流程现需 `--task-layout near-start`：任务预算20秒，探索约8秒后预留12秒前往实验终点；它不是遍历全场的任务。现在默认的 `finish-area` 布局把球和实验终点移到远端，不能再用原19.8秒结果代表其验收。
已检查用户的 `output/my-sim`：19.8秒正常发出finish，没有异常或跌倒。旧版随后立即关闭窗口，运行中也没有进度提示。

现在运行时会向终端输出状态、每2仿真秒的进度和所有任务事件，并保存到 `runtime.jsonl`。
停止时明确输出 `mission_finished`、`simulation_time_limit`、`mission_budget_exhausted`、跌倒或异常等原因；`summary.json` 增加 `stop_reason`、`mission_complete` 和 `time_limit_seconds`，时间用完不再标为completed。

`--viewer` 默认在结束后保留最终画面，物理步进已经停止。关闭窗口或按Ctrl+C退出；批处理需要自动关闭窗口时加 `--close-viewer-on-stop`。
无窗口运行会自动退出，仍保存全部结果。仿真未接入实物串口，这里记录的是终端和程序诊断信息。

新环境安装：

```bash
.venv/bin/python -m pip install -e '.[simulation]'
```

源码中的 `third_party/go2_convex_mpc` 必须保留。完整已验证的Python依赖版本见项目锁定文件。

## 两种场景的区别

**默认场景**直接读取提供的 `scene/mujoco/competition-course.mjb`。原文件有12个力矩执行器、41个传感器、0个相机，没有动态足球。
运行时增加RGB渲染和72×9束雷达采样，不向导航提供真值位置、几何ID或地图。
该原始MJB没有本项目可识别的实验终点，所以探索后可能停在waiting_finish；不会因此伪造任务完成。

**`--task-fixture` 场景**从同一MJB复制772个静态几何，并使用提供的Go2 XML，增加一个0.43kg、半径0.11m的动态足球，以及本地ArUco 42终点和照明。
复制后的网格世界顶点误差约1.2e-8米。构建时会自动检查几何误差；运行目录保存来源SHA-256、`scene_provenance.json` 和 `runtime-scene.mjb`。
这用于测试完整任务，不能称为未修改的官方比赛布局或官方终点。
所有原始资源保持不变，SHA256SUMS已重新验证通过。

`--spawn X Y`仅设置一次仿真初始位置，默认在起点附近的(0.8,0.8)。它不是导航目标或预设路线。
`--control smoke`是明确标注的控制器回归动作序列；默认autonomy使用原有自主算法产生指令。

## 运动控制器

开源项目：[go2-convex-mpc](https://github.com/elijah-waichong-chan/go2-convex-mpc)，MIT许可证。
固定提交 `1c63c6a762779887ab0431fd60db681dede6cb32`；源码、原始哈希、许可证及修改说明都保存在third_party目录。

复用其质心凸MPC、16步预测模型、摩擦约束QP、步态调度和摆腿力矩控制。
本项目适配官方Go2模型的质量/惯量、MuJoCo雅可比和按名称解析的关节/执行器映射。
不是只求一次平衡力或把关节动画称为MPC。

- 物理步进500Hz，腿控制约167Hz，QP更新约42Hz，感知/导航按仿真时间10Hz。
- 四足IMU/关节观测驱动接触调度下的运动学状态估计；没有把仿真根节点位置/速度作为控制器输入。
- 运行循环只向`data.ctrl`写力矩，机器人状态由`mj_step`推进；不使用根节点外力、mocap牵引或每步改写qpos/qvel。
- 真值位置仅用于相机/雷达生成、渲染和`diagnostics_only`离线评估。
- 速度停止后保留原地小跑以保持步态连续；不是关闭执行器让机器人自由倒下。
- 尚未验证碎石、台阶、陡坡和整张赛场的通过能力。当前控制器和地面支持地图仍是保守初版。

### 斜坡误判排查与修正

用户的 `output/finish-area-run` 在约12秒进入waiting_finish，最终位于世界(0.978,1.647)，还未到实际坡道。确认存在多个问题：固定高度分类会把累计升高超过10cm的缓坡当障碍；局部避障也独立误用该阈值；起点通道的0.32m圆半径加0.12m裕量及栅格取整会过度膨胀；不足3格的探索边界原来会被丢弃。原MPC则使用平地COM/落脚高度，在坡脚附近停滞。

现在改用地面连续性、局部坡度/高差和地面相对净空，A*/DWA统一分类；小范围插值只在有返回点的短三角形内部生效，陡坡、过大台阶和大面积缺测仍禁行。斜率估计不跨墙/石块边缘平滑，避免把墙边平地也误判为陡坡后再次膨胀。
仿真已知Go2尺寸采用有朝向的机身包络检查，实物未知配置仍保持原保守圆形参数。仿真默认坡度上限14°（原10°低于实测约12°的坡面）；实物默认不提高。

MPC从机体雷达观测估计脚下附近坡面，调整COM高度/俯仰/横滚、竖直参考速度及摆腿落脚Z。没有使用仿真真实高度或地形ID。加入8.6秒真实官方坡道回归测试：要求走过旧控制器约y=4.8米的停滞位置，并实际升高。

可复现的坡道控制器单测（一次性初始化在坡前，不是起点自主导航）：

```bash
.venv/bin/go2-rescue mujoco --viewer --control ramp-smoke --spawn 1.1 4.0 --spawn-yaw 1.5707963267948966 --seconds 22 --output output/my-ramp-check --record
```

`output/mpc-ramp-verified-01` 使用原始官方MJB：向前指令至18秒，随后零速度保持至22秒，未跌倒，最终位置(1.074,5.898,0.617)，初始(1.1,4.0,0.3)，水平位移约1.90米；雷达观测坡度约12°。结果含真实动力学视频、观测与状态记录。
这证明能够在该坡面上持续向前爬升并停止，不代表已越过坡顶、避开水洼、完成从原起点到终点的自主流程。

从原起点的中间验证 `output/terrain-navigation-02` 已从原停滞处前进到世界y约3.92米，但仍因后续局部规划/地形限制停车，未完成任务。`terrain-navigation-03` 也未完成，保留为失败记录。当前完整赛场导航仍需继续验收。
`output/terrain-navigation-04` 再次运行60秒，无跌倒，最终世界(0.990,3.919,0.329)，状态waiting_finish、无可达探索边界，仍未完成任务。`local_planner_no_safe_motion` 现在用于明确区分DWA无安全速度与没有探索目标。
随后还将净空单独保存为地面相对 `headroom`，避免历史未分类坡面留下虚假“低屋顶”；地面下包络采用低层样本中位数，降低反复取最小值导致的负高程漂移。最新map.npz包含headroom字段。

**最终补丁验证 `output/terrain-navigation-05`：**从原起点(0.8,0.8,0.3)运行共同Autonomy算法35秒，自主接近并爬坡，未跌倒，最终世界(0.866,5.167,0.459)，水平位移4.37米，雷达观测坡度约12°。此轮以35秒测试上限停止，状态仍exploring，未发finish。它验证自主上坡，不验证坡顶/水洼/整场终点。

## 已验证结果

| 运行目录 | 结果 |
|---|---|
| `output/mujoco-mpc-smoke-01` | 原始MJB，8秒，前进/转向/停止，位移0.416米，未跌倒 |
| `output/mujoco-official-final` | 原始MJB，自主运行12秒，位移0.918米，未跌倒，墙面未误报为球；没有终点就不发finish |
| `output/mujoco-task-verified` | 官方几何本地任务，19.8秒，start/target/finish各一次，未跌倒；视频含最终状态 |
| `output/stop-check-viewer-01` | 实际桌面窗口再次完整运行，19.802仿真秒 / 56.05墙钟秒，193帧观测，826次QP；start/target/finish各一次，无异常、无跌倒，停止原因为mission_finished |
| `output/stop-check-hold-01` | 实际桌面窗口0.8秒控制器短测，到达时间上限后保留画面与进程，Ctrl+C正常退出；时间上限未标为任务成功 |
| `output/finish-area-placement-01` | 新默认布局：球/标记已移到远端终点，实际MuJoCo从原起点运行3秒，无跌倒，仅start事件；另有全场与终点近景finish-area-overview.png；未验证从起点到远端的完整任务 |

这些是局部集成回归，不代表覆盖整张地图，也不是80%比赛成功率。
当前整套仿真运行速度约慢于实时2–3倍；最终本地任务耗时47.6秒完成19.8秒仿真，感知/导航P95约0.225秒，MPC QP P95约3.1毫秒，具体结果见各目录summary。

## 输出和复查

默认仅保存配置、路线、场景来源说明、trajectory/runtime日志和summary。以下视频/大文件均需显式开启；不再自动生成。性能与分叉口修复见[性能报告](PERFORMANCE.md)。

- `simulation.mp4`（`--video`）：左侧第三人称物理场景，右侧实际渲染的前视RGB与任务状态。
- `preview.png`（同时传`--video --snapshots`）：最后状态预览。
- `trajectory.jsonl`：自主定位、速度指令、状态事件及耗时。
- `runtime.jsonl`：终端实时诊断、状态变化、任务事件与明确停止原因。
- `physics.npz`（`--save-physics`）：实际物理步进产生的采样状态（诊断记录，不送给导航）。
- `map.npz` / `map.png`（`--snapshots`）：传感器构建的地图。
- `recording/`：加`--record`保存的RGB、雷达、IMU及运动学速度。
- `runtime-scene.mjb`：仅在本地任务显式传`--save-physics`或`--snapshots`时导出，默认不写约64MB的场景副本。
- `summary.json`：是否跌倒、MPC求解次数、任务事件、时间与诊断位移。

回放时必须使用同次运行保存的相机/传感器配置：

```bash
.venv/bin/go2-rescue replay --input output/my-local-task/recording --config output/my-local-task/config.json --output output/my-local-replay --steps 300
```

执行 `.venv/bin/python -m pytest -q` 可验证原有算法及新增MuJoCo物理测试。最新斜坡修复后42项通过（43.95秒），包含原始时间上限/远端球位置测试，以及缓坡/陡坡/缺测/净空/机身朝向和真实动力学爬坡回归；官方SHA256SUMS再次通过。
