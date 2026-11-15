# Robotics Bench · Unitree Go2 主具身本体视觉资源

本包将 Embody Sim 已验证的 Unitree Go2 GLB 固定为 Robotics Bench 首期赛事的主具身本体视觉资源。

- `model/unitree-go2.glb`：平台在线 PBR 与运动学预览模型，内嵌 `Walk_InPlace` 循环动作；
- `model/unitree-go2.articulation.json`：42 links / 41 joints 层级；
- `model/unitree-go2.validation.json`：源 URDF 与 GLB 摘要验证；
- `preview/unitree-go2-preview.png`：由赛事 MuJoCo 物理模型 `home` 关键帧重新渲染的 z 轴向上站立姿态预览图；
- `LICENSE.txt`、`SOURCE.json`：BSD-3-Clause 许可与来源记录。

GLB 保留 33 个网格和 7 组 PBR 材质。`Walk_InPlace` 使用 8 条关节 quaternion 轨道，仅作原地运动学展示，File Explorer 会自动循环播放；它不是厂商动作、运动策略、控制器或动力学验证。材质是平台视觉分组与参数优化，不应称为厂商实测材质。
本包不含完整竞赛场景、布局坐标、MuJoCo 物理权威模型或运动策略。赛事物理评测仍应绑定独立、固定版本的 MuJoCo 模型与 runner。
