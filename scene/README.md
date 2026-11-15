# 场景资源

- `mujoco/competition-course.mjb`：完整赛事场景的单文件 MuJoCo 训练入口，可使用 `mujoco.MjModel.from_binary_path(...)` 加载；
- `visual/competition-course.glb`：完整赛事场景的 PBR 视觉预览，不包含重复的 Go2 实例；采用标准 glTF Y-up，并携带 File Explorer 可直接识别的轴向元数据；
- `../robot/go2/visual/model/unitree-go2.glb`：独立的 Go2 视觉、层级与运动学预览模型；
- 场景中的 Go2、乱石堆和其他碰撞数据均已编译进 MJB，不再以零散 OBJ 依赖向参赛选手分发。

本目录不包含 Blender、MJCF 制作源文件或拆分碰撞网格。视觉 GLB 已去除重复且姿态错误的 Go2 预览实例，并恢复暖灰混凝土、灰砖、砂浆和石灰岩 PBR metallic-roughness 参数；训练、验证和评测仍以 MJB 场景为准。

如需查看 Go2 外观与层级，请使用独立 Go2 视觉模型；它不替代 `go2.xml` 或场景 MJB 的物理定义。

