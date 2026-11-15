# Unitree Go2 赛事训练资源包 v004

本包包含冻结视觉场景、独立 GLB 训练件、可移植 MuJoCo/MJX 场景、碰撞源、训练契约模板、Blender 源文件与验证证据。

## 快速入口

- 在线完整预览：`preview/scene/scene.visual.glb`
- 独立训练件：`preview/logical/*.glb`
- 全部独立模块：`preview/modules/*.glb`
- 高保真 MuJoCo：`mujoco/competition-scene-fidelity.xml`
- CPU 快速档：`mujoco/competition-scene-fast.xml`
- MJX 快速档：`mujoco/competition-scene-mjx_fast.xml`
- 模型索引：`preview/model-index.json`
- 场景与训练契约：`training/`
- 本地验证报告：`validation/VALIDATION-REPORT.md`

## 在线渲染

GLB 保留世界坐标 XYZ/Z-up。Three.js 等 Y-up 渲染器只在展示父节点上应用 `rotation.x = -Math.PI / 2`；不要改写模型顶点，否则会破坏与 MuJoCo 世界坐标的映射。详见 `preview/viewer-contract.json`。

## 物理档位

- `fast`：每个乱石堆 24 个内部椭球代理，用于烟测和快速迭代。
- `fidelity`：每个乱石堆 100 个源石块凸碰撞分片，坡道使用 VHACD 分解；用于高保真训练候选。
- `mjx_fast`：MJX 兼容 box-only 快速碰撞档，用于批量容量与训练集成。

5 个乱石堆始终是 5 个静态逻辑 body；内部碰撞分片不是独立障碍或动态刚体。

## 自检

```bash
python tools/validate_training_package.py .
```

安装 `mujoco==3.17.0` 后，自检会额外编译三套 MJCF。

## 边界

本包不含参赛 locomotion policy/controller，不代表目标 NVIDIA GPU 10,000 步长稳、完整策略通关或真实 Go2 Sim2Real 已通过。训练前先阅读 `PACKAGE-NOTICE.md` 和验证报告。
