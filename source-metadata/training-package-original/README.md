# Robotics Bench Go2 赛事训练资源包

状态：可分发版本。训练、验证和评测仍以 MuJoCo 物理文件为权威。

本包面向 MuJoCo 训练、验证与评测，统一收录 Go2 本地训练模型、完整赛事场景、独立障碍及生命探测目标。内部来源版本仅保留在审计元数据中，不用于公开资源分类和目录命名。

## 资源分类

### 1. Go2 训练模型

- `robot/go2/physics/go2.xml`：Unitree Go2 MuJoCo 本地训练物理模型；
- `robot/go2/physics/assets/`：该模型引用的 16 个 OBJ 网格；
- `robot/go2/visual/`：与赛事场景一致的 `unitree-go2-pbr-v1` PBR/关节主本体及预览资料；GLB 内嵌 `Walk_InPlace` 循环动作，平台 File Explorer 可自动播放。

`go2.xml` 包含 12 个关节与 12 个电机执行器，是当前本地训练物理基线。`robot/go2/visual/` 中的 GLB、动作和关节元数据只用于视觉、层级及运动学预览，其 Manifest 明确 `physics_authority=false`；两者不能混为同一物理权威文件。`Walk_InPlace` 不是 locomotion policy、控制器、实机轨迹或评分依据。

### 2. 场景资源

- `scene/mujoco/competition-course.mjb`：可直接加载的单文件 MuJoCo 物理场景；
- `scene/visual/competition-course.glb`：File Explorer 可直接查看的完整场景视觉模型，不内嵌重复的 Go2；
- `robot/go2/visual/model/unitree-go2.glb`：独立 Go2 视觉模型及循环 walk 动作。

机器人与场景碰撞数据已经编译进 MJB，不再以数百个零散 OBJ 向参赛选手分发。视觉 GLB 不替代 MJB，Blender 制作源文件和制作脚本不进入参赛资源包。

### 3. 障碍资源

`obstacles/` 只包含 9 个独立障碍：5 个整体乱石堆、3 个隧道和 1 个坡道组合。每个乱石堆作为一个独立模型交付，不按单块岩石拆分。每个障碍包含 MuJoCo `model.xml`、`model.mjz`、预览 GLB 和元数据。碰撞层独立于视觉模型：每组乱石堆使用 100 个凸网格，坡道使用与完整场景一致的 161 个凸分解网格；Tunnel2 使用 24 个保持中空通道的凸环段网格，Tunnel3 的 5 个结构碰撞件等形转换为凸网格。Tunnel1 明确设计为不可通行，碰撞定义保持不变。

### 4. 生命探测资源

`life-detection/football/` 包含足球生命探测替代目标的 MuJoCo 模型、MJZ、预览 GLB 和语义元数据。它仅代表赛事目标代理，不代表真实人体或生理信号。

## 运行边界

- 当前平台赛事边界仅为 MuJoCo 训练、验证与评测；
- 不提供 Isaac Sim / Isaac Lab、Gazebo、Webots 等其他仿真框架的在线运行；
- RGB 需要独立冻结相机与渲染合同，不能因存在视觉模型就假定策略必然获得 RGB；
- 正式隐藏布局、目标摆放坐标、裁判真值、评分配置和参赛策略不在本包中。

## 视觉预览约定

- 完整场景、障碍、足球与 Go2 GLB 均使用 glTF 2.0 右手坐标系、Y-up 文件约定，并携带 File Explorer 可直接识别的 `roboticsBench.upAxis=Y` 展示元数据；MuJoCo XML/MJZ/MJB 继续使用 Z-up 物理坐标；
- 完整场景 GLB 保留原赛事几何范围且恢复与独立障碍一致的暖灰混凝土、灰砖、砂浆和石灰岩 PBR metallic-roughness 参数，不再以白模方式展示；
- 9 个独立障碍 GLB 使用与赛事全景图一致的石灰岩、暖灰混凝土、灰砖与砂浆 PBR 色彩分组，并显式携带非金属度、粗糙度及基础色参数；
- 障碍与足球 GLB 通过顶层 `PreviewRootCentered` 展示节点将包围盒中心对齐原点，避免交互旋转时模型偏离视野；内部网格局部坐标保持不变，足球预览同时携带标准 Y-up/PBR 展示契约；
- Go2 GLB 保持原始本体、层级和材质，在本体层级之外增加明确的 Z-up→Y-up 坐标转换根节点；不参与障碍模型的居中处理；
- 完整场景视觉预览使用 `scene/visual/competition-course.glb`，且不包含重复机器狗；训练、验证和评测统一加载 `scene/mujoco/competition-course.mjb`；
- Go2 GLB 的 `Walk_InPlace` 仅由 8 个关节 quaternion 轨道构成，循环播放时不改变根节点位置；
- 展示根、颜色、金属度和粗糙度均不具备物理权威性。训练、验证和评测仍以 XML、OBJ、MJZ 与 MJB 为准。

## 完整性校验

在本目录执行：

```bash
shasum -a 256 -c SHA256SUMS
```

所有条目必须返回 `OK`。统一版本和关键入口见 `MANIFEST.json`，原始来源文件与许可边界见 `source-metadata/`、`LICENSES/` 和 `NOTICE.md`。

