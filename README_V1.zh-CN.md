# Go2 搜救算法初代版本

本版本按“实物调用厂家行走接口、仿真使用开源MPC、无需训练、不依赖 ROS 2”实现。

**2026-11-16初版范围已确认：原起点→L1上/下坡→S弯→死胡同进退→碎石区→本地入口坡→2号管内停稳。出口22厘米下降台阶不在本次实现范围。** 最终原起点串联已验证入管和停稳，117项测试通过；后续出管、终点找球和比赛完成仍未验收。[实际结果](docs/ENTRY_RAMP_NAVIGATION.md)。
官方资源包的 README.md 和物理/视觉资源保持原样。本文件是新增算法的入口。

现已接入**官方MuJoCo场景＋开源MPC运动控制器**。运行命令、两种场景区别和实际仿真结果见 [MuJoCo说明](docs/MUJOCO.md)。

```bash
.venv/bin/go2-rescue mujoco --viewer --task-fixture --course-policy ordered --realtime --seconds 600 --output "output/run-$(date +%Y%m%d-%H%M%S)"
```

按最新要求，`--task-fixture`现在默认使用**本地入口坡版本**：把2号管道原东侧斜坡移到当前西侧进管口，官方MJB原文件保持不变。窗口右下角默认显示实时前视相机，复用10Hz传感器图像，不录屏；大窗口放大至480×360。`--pipe-ramp official`恢复原布局，`--hide-camera`隐藏相机。见[本地地图和相机说明](docs/LOCAL_MAP.md)。本地入口现已接通自主爬坡入管专项；东口22厘米下降台阶仍有停车保护，尚未验收完整穿出。

仿真普通行走和窄口前进上限现默认为**0.5m/s**，可以添加`--max-speed .5`显式设置；重启后生效，启动日志打印最终限速。实际速度仍由障碍、转弯和制动检查决定。[速度设置与实测](docs/SPEED.md)。

仿真明确后退段的默认上限现为**0.25m/s**（可加`--max-reverse-speed .25`覆盖）；普通行走中小幅倒车、脱困保持慢速。死胡同前墙连续3帧确认，停稳计时有2厘米保持区域，避免定位边界抖动反复重置。实物默认倒退仍为0.08m/s。详见[死胡同改进与管道模型核对](docs/DEAD_END.md)。

默认关闭视频、RGB/雷达原始录制、物理归档、图片/地图快照和运行场景二进制导出；仅保留小型诊断日志和配置。窗口与录屏解耦，刷新上限60Hz。物理、MPC、导航使用CPU，图像渲染使用OpenGL，启动日志显示实际显卡。详见[性能与分叉口修复](docs/PERFORMANCE.md)。

默认将黑白球和实验终点放在赛场远端终点区域，机器人仍从上坡前的原起点出发。顺序模式要求上坡、下坡、S弯、死胡同进退、碎石区域、圆形2号管道，再到终点找球报位。官方布局人工路线先验在`configs/official-course-route.json`，本地入口坡路线在`configs/local-entry-ramp-route.json`；按场景自动选择，官方原文件不改，整圈仍未验收。旧的起点附近短回归需显式添加 `--task-layout near-start`。

**当前必须穿过圆形2号管道，3号通道不能代替。** 本地入口坡使用雷达支撑面、定向完整机身碰撞检查和坡面MPC，先在平地对齐再入管；针对稀疏地面/顶壁混淆和路线偏差的修复见[入口自主导航验证](docs/ENTRY_RAMP_NAVIGATION.md)。目前在管内输出`ordered_pipe2_exit_requires_validated_descent_capability`，保护未验证的22厘米出口下降。原官方直角入口布局仍输出`ordered_pipe2_requires_validated_step_capability`。不能通过增大配置中的步高冒充适配完成；旧3号路线和历史2号脚本控制器试验均不是当前整圈验证。

历史后续流程已加入水洼观测禁行、隧道净空、侧移结合转向的恢复规划、失败目标暂缓重试、严格球体确认，以及终点与碰撞/入水/报位误差诊断。运行与验收见[比赛流程说明](docs/COMPETITION_V1.md)，完成程度以实际长测结果为准。

## 当前能运行什么

- 独立命令行程序，支持软件传感器场景、录制回放、本机传感器桥接和原生 SDK 接入。
- MuJoCo 3.17.0官方MJB自主运行、开源凸MPC力矩控制、前视RGB和72×9雷达、第三人称视频及可选交互窗口。
- 雷达平面 ICP、关键帧、保守回环候选和位姿图优化；可切换成熟的 KISS-ICP 三维里程计。
- 占据/地面支持/净空栅格、障碍膨胀、A*、DWA、前沿探索、卡住回退和重规划。
- 黑白足球的边缘/圆弧 RANSAC、纹理复核、地面几何定位、雷达球面复核、多帧跟踪和去重报位。
- 本地 ArUco 终点检测、开始/目标/结束事件，以及缺少终点时的超时停车。
- SDK 速度限幅、变化率约束、超时 StopMove、异常退出停车。没有训练/RL/ONNX依赖。

**这是可运行的算法初版，尚未完成实机验收。** 当前型号、雷达驱动、相机参数及固件权限未知。
SDK 传输已用模拟客户端验证，未连接机器狗。原生雷达驱动不能在型号未知时凭空指定。
软件场景仅是运动学与合成传感器测试，不模拟腿部动力学，不代表 MuJoCo 比赛成绩或实物通过率。

## 快速运行

以下命令在项目根目录执行。原开发机已建立 `.venv`（Python 3.10.22）；新克隆仓库需先按下方安装步骤创建环境。虚拟环境和output不随Git分发。无需改系统 Python 或 ROS Noetic。

```bash
.venv/bin/go2-rescue demo --output output/my-demo --steps 200 --record
.venv/bin/go2-rescue replay --input output/my-demo/recording --output output/my-replay --steps 200
.venv/bin/python -m pytest -q
```

每次使用新的输出目录，避免覆盖记录。输出包含 `summary.json`、`trajectory.jsonl`、`map.npz`、`map.png`；加 `--record` 保存可重复回放的原始观测。

验证带足球与近距离本地终点的完整软件流程：

```bash
.venv/bin/go2-rescue demo --mission-fixture --config configs/software-mission.json --output output/my-mission --steps 100 --record
```

本次运行结果在 `output/mission-02`；`demo.mp4` 是带检测和任务事件的演示视频。重新录制后可生成同类视频：

```bash
.venv/bin/python scripts/render_recording.py output/my-mission --output output/my-mission/demo.mp4
```

从新电脑安装：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements-v1.lock.txt
.venv/bin/python -m pip install -e . --no-deps
```

没有 Python 3.10 时使用独立 conda 环境安装，不替换系统解释器。

## 连接实物

1. 复制 `configs/go2-unknown.json` 为实际设备配置，填写型号、网卡、雷达到机体外参、相机内外参、真实图像尺寸和静态机体高度。
2. 核实该型号/固件能使用 SDK 的 SportClient、状态订阅和相机服务。上层输出机体 `vx、vy、wz`，底层仍由 Go2 控制。
3. 为实际雷达实现 [驱动契约](docs/HARDWARE_ADAPTER.md)；或将已有驱动输出接入本机 TCP 桥。校准、时间同步及丢帧情况须通过原始数据回放验证。
4. 安装与设备匹配的官方 `unitree_sdk2_python`，先运行不带 `--execute` 的传感器与规划流程。
5. 完成标定和配置核对后，将配置中的 `calibrated` 设为 true，启用 `--execute` 才向 SDK 发命令。未知型号模板会拒绝执行。

已有本机传感器桥时：

```bash
.venv/bin/go2-rescue live --config configs/my-go2.json --endpoint tcp://127.0.0.1:8765 --output output/live-observe --steps 1000 --record
```

使用原生 SDK 状态/相机与实际雷达驱动时（`my_lidar` 是需按设备提供的模块名）：

```bash
.venv/bin/go2-rescue native --config configs/my-go2.json --lidar-provider my_lidar:open_driver --sport-velocity-frame body --output output/native-observe --steps 1000 --record
```

`--sport-velocity-frame` 必须依据固件状态接口选择 `body` 或 `imu_world`。不能凭名称假定速度坐标系。
上述命令默认不发送运动命令。实物测试时在相同命令上显式添加 `--execute`；程序不自动起立、不切换步态或关闭厂家服务。

## 边界与后续工作

- 地图使用起点局部坐标 `map_start`。没有赛方起点到世界坐标约定时，不能声称符合正式报位接口。
- 默认平面定位面向近似平地；有关键帧回环，但仍需在真实重复走廊中验证误闭环和漂移。`kiss` 后端提供三维里程计，此版本未对它做回环修正。
- 步高/坡度阈值采取保守配置；没有实测前不会把碎石、陡坡或低隧道当作可通过地形。未建足够地面支持的区域也会停车。
- 几何识别针对黑白足球。遮挡、黑白圆形杂物、反光与低分辨率都会影响结果；合成图片通过不等于真实检测精度达标。
- 足球定位假定目标位于附近支撑平面上；起伏地面或不可靠圆弧中心会增大误差。缺少足够几何证据时不报位。
- ArUco 42仅为本地测试终点，不是官方终点协议，也不意味着可在比赛场地自行粘贴标记。
- 相机 SDK 未提供曝光时间戳；当前用请求时刻保守估计帧龄。实物需测量延迟，必要时接入能提供时间戳的相机驱动。
- 目前未报告80%实物成功率。下一步是连接实际设备，录制原始传感器数据，再做真实场景验收。
- 后续训练版本可以替换定位、识别或决策模块；初代不需要先训练模型。

接口与数据流见 [架构](docs/ARCHITECTURE.md)，修订实施范围见 [初代方案](docs/V1_REAL_ROBOT_PLAN.md)，实测记录见 [验证报告](docs/VALIDATION.md)。
