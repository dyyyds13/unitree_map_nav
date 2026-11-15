# Robotics Go2 模块化场景 v006

**v006 本体资产：** 引入 Embody Sim 已验证的 `Unitree Go2` GLB，放置于起点 H 区域。机器人保持为独立模块，完整保留 42-link/41-joint 层级、33 个网格与 7 组 PBR 材质，不与地板、墙体或障碍物合并。该 GLB 用于视觉与运动学预览，不替代 MuJoCo 物理模型。

**v005 隧道2修正：** 已删除圆筒内部错误横板。圆筒本体保持为完全贯通的开放环形管；通往隧道3的独立门槛组件保持不变。

**2026-10-23 当前主场景已删除示意标注。** 8组地面箭头、9组悬空标牌及隐藏尺寸说明均已从对象数据中删除，并非隐藏。保留起终点H标记和全部实体障碍；当前28个模块、707条资产记录。重新打开已保存文件后核验通过，其他实体几何、变换、材质保持一致。当前预览为 `renders/06-overview-clean.png` 与 `renders/07-top-clean.png`，结果见 `annotation-free-review.json`。旧渲染和旧审计属于删除前历史记录；清理前原件位于 `revisions/`。

本版制作时基于随附的 v003 磁盘副本，保留原场地布局、墙体与三隧道连通关系。v001～v003 均保留；制作过程没有操作当时含未保存改动的 Blender 窗口，也没有修改任何业务仓库。迁移时的独立内存快照另见下文。

## 交付入口

- `robotics-go2-modular-v004.blend`：主场景、28 个模块根、独立组件、程序材质、隐藏的世界 AABB 及内嵌脚本。
- `independent-append-reassembly-v004.blend`：空工程实际 Append 后的独立重组验证，包含上坡、水洼、100 块石头组成的石堆、旋转后的圆管。
- `renders/01-overview-2048.png`：删除前历史2048×1536全景。
- `renders/02-puddle-clearance.png`：逐断面净宽核对视图；绿色仅为0.50m静态参考带。
- `renders/03-modules-world-aabb.png`：原场景主要模块的世界 AABB。
- `renders/04-independent-append-reassembly.png`、`05-independent-append-world-aabb.png`：实际重组与变换后的包围盒。

## 水洼测量

| 项目 | v003 | v004 |
| --- | --- | --- |
| 名义水面宽 | 0.30m | 0.50m |
| 实测水面最大宽 | 0.293322m | 0.488870m |
| 实测水面纵向长度 | 0.745167m | 0.745167m |
| 计入湿边的左侧最小连续干燥净宽 | 1.323073m | 1.120680m |

水面右缘锚定在原位置 X=1.626661m，向左加宽，长度、坡面位置及自然不规则轮廓保留。湿边最宽约0.523091m；湿边至右护墙最小仍有0.062213m间隙。最窄干燥断面在 Y≈6.291216m，实测左墙内表面 X=0m，湿边左缘 X≈1.120680m。独立大石头保持原位置，与水洼前后分离。

校核取当前实际 Go2 包络宽0.31m，1.5倍为0.465m，执行阈值取≥0.47m。用 evaluated 实体墙/石块等障碍的三角面投影，与湿边逐 Y 求交，覆盖边界顶点、区间中点和≤2mm均匀采样，共965个断面。墙厚不计入干燥净宽。完整改前/改后逐断面数据在 `puddle-clearance.json`，不是只比较一个包围盒。

悬浮中文信息牌及文字原本为参考图的信息标注，现已从主场景删除；地面支撑体和测量辅助物也不计入净宽障碍。此结果仅是静态几何宽度，不等于验证了摆腿、姿态、挂载、摩擦或 Go2 动态通过能力。旧“水洼≤路宽1/3”约束已取消。

## 独立复用

在 Blender 中 File → Append → 选择主工程 → Collection → `ASSET_*`。每个模块以 `ROOT__*` Empty 为整体移动入口；几何子对象仍可独立选取、移动、隐藏或导出。28个模块根和679条几何组件记录见 `asset-manifest.json`。当前模型没有FONT文字对象，箭头与标牌模块也已删除。

| Collection | 内容 / 依赖 |
| --- | --- |
| `ASSET_BaseGround` | 纯平12×10m基础底板及独立接缝/裂纹曲线；未烘入坡道或障碍 |
| `ASSET_Ascent`、`ASSET_Platform`、`ASSET_Descent` | 上坡、平台、下坡三个独立封闭实体；原顶面6290个关键顶点按1e-5m精度核对 |
| `ASSET_Puddle` | 水面和湿边两个独立对象；原设计依附12°坡面，重组时需要匹配的支撑面 |
| `ASSET_Pile_01`…`ASSET_Pile_05` | 每堆独立根/Collection，各保留100个独立石块 |
| `ASSET_IndependentBoulder` | 独立大石及邻近碎片 |
| `ASSET_Tunnel1`、`ASSET_Tunnel2`、`ASSET_Tunnel3` | 三隧道独立，圆管内平底、方涵各顶板均保持可拆卸对象 |
| `ASSET_JunctionThreshold` | 2→3的独立斜门槛；与两隧道配合使用 |
| `ASSET_Marker_START/FINISH` | 起终点圆形H地面标记；箭头和文字标牌已删除 |

**隧道3共用墙依赖**：`ASSET_Tunnel3` 不含复制的右墙，它使用场景中现有 `ASSET_Wall_EastBoundary`。独立复用时一起 Append 该墙，或接入新场景已有边界墙。隧道1的墙体开口依赖 `ASSET_FinishWestPartitions`。墙体按各构造分别分组。

主模块原点位于实际 evaluated AABB 底部中心，尺度为1；石块子对象保留原生旋转与枢轴，通过石堆根整体落地，单块摆放可使用其 bbox min Z。改变原点时已补偿程序材质 Object 坐标，保持原外观。水洼是贴坡的视觉资产，没有流体物理。

## 包围盒入口

在主工程的 Text Editor 选择 **`refresh_bboxes.py` → Run Script**，或在脚本中导入后调用 `refresh()`。

- 基于 modifiers 求值后的真实几何顶点和 `matrix_world`，生成米制世界轴对齐包围盒（World AABB），覆盖模块聚合及独立组件。
- `asset-manifest.json` 包含源资产稳定ID、实例ID、名称、min/max、dimensions、世界位置/矩阵、相对父级的位置/旋转/尺度及依赖。旋转后世界 AABB 会随实际几何重新计算，不是局部定向盒。
- 排除相机、灯光、FONT文字、QA标注和包围盒辅助物自身；主场景中已无文字标牌。
- 辅助物位于 **`98_ASSET_BOUNDING_BOXES`**，默认不显示、不参与最终渲染。启用该 Collection 可查看模块线框；再启用 `BBOX_COMPONENTS` 可查看每个几何组件。
- 移动或旋转后重新运行脚本刷新。仅替换带生成标记的自有辅助 Collection，不重复堆积；当前刷新生成707个包围盒，保留实体清单与删除前逐项一致。脚本写出清单，但不自动保存 `.blend`。
- 重复 Append 可保留相同源资产ID，使用不同实例ID记录，已验证不会覆盖清单中的另一实例。

AABB 用于占地、定位和粗略碰撞检查，不能代替精确碰撞体或运动通过验收。

## 实际重组验收和重现

`append-test-report.json` 记录从最终磁盘工程向全新空工程真实 Append 的结果：上坡/水洼、石堆、圆管四个模块，导入前后 evaluated 几何与材质节点一致；圆管旋转30°、石堆旋转-18°并移动后，AABB与变换后顶点独立计算结果一致；尺度均为1。额外验证了注释/辅助物排除、重复Append的两个实例和刷新幂等性。可直接打开独立重组工程审阅。

所有脚本只在**独立后台进程**重建或验证，不应粘贴重建脚本到已有未保存工程中。下列初版历史命令会重新生成示意标注；审阅当前清理版使用 `source/render_annotation_free_views.py`，不要直接运行旧 `verify_v004.py`。历史命令：

```sh
/Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup --python source/rebuild_v004.py -- --render
/Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup --python source/verify_v004.py
```

`source/base-v003.blend` 是保留的输入副本，使 v004 可以在本目录内重现。字体已打包，材质为程序节点；新场景不依赖当前原工程窗口。`source/parameters.json`、`module-audit.json`、`puddle-clearance.json`、`asset-manifest.json` 和 `append-test-report.json` 分别记录参数、地形/刷新检查、断面、资产和重组证据。


## 2026-10-23 本地迁移

当前目录位于 `robotics-bench/temp/robotics-go2-blender-20261022/v004/`。主工程、独立 Append 工程、历史版本、`.blend1` 和输入副本均已改用当前目录可解析的字体/参考图路径及目录内渲染输出。字体继续打包，并补齐 `source/Chinese_ArialUnicode.ttf`，便于日后解包。

迁移只修复路径，并为少数原本零用户的材质设置 Fake User 保留标志，防止 Blender 保存时自动丢弃它们；所有材质节点/参数、几何、UV、变换、相机和集合内容均与冻结原件逐项一致。原始副本、旧哈希和首次校验异常记录保留在 `temp/migration-before-reference-fix-20261023-094535/` 及 `temp/migration-tools/`。

后台重开主工程与独立 Append 工程后，750 条和109 条资产的清单/世界 AABB 均与原记录完全相同，刷新两次保持幂等；四条隧道射线和四块地形闭合性通过。本次没有重建或重新渲染，既有 PNG/SVG 像素文件哈希未变。

当前 GUI 内存工程另存于 `../migration-session/current-memory-relocated-20261023-094151.blend`，含739个对象，与最初内存快照内容指纹一致，没有合入本版正式场景。迁移入口见 `../README-迁移入口.md`，完整验证见 `temp/migration-validation.json`。
