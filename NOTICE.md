# 本地汇总与使用边界

本资源包是 Robotics Bench Go2 赛事训练资料的可分发汇总。汇总行为不扩大任何源资产的许可范围。

- Unitree Go2 MuJoCo 模型与相关网格按随包 BSD 3-Clause License 提供，固定来源提交见 `source-metadata/go2-physics/`；
- PBR Go2 模型的来源、哈希、派生边界和 BSD 3-Clause 许可见 `robot/go2/visual/`；
- 场景内 Unitree Go2 相关许可见根目录 `LICENSES/`，场景来源审计见 `source-metadata/scene/`；
- 障碍和生命探测目标不构成 MIT、商用、任意再分发或其他开放许可，当前参赛选择边界见 `source-metadata/participant-selection.json`；
- 正式对外分发前，仍须由资源所有方确认完整场景、障碍和目标资产的发布授权与最终赛事版本。

本包内嵌的 `Walk_InPlace` 是 Robotics Bench 为 File Explorer 预览添加的原地运动学动作，不是 Unitree 官方动作、控制器、运动策略、实机轨迹或物理验证。本包不包含官方 Unitree 控制器、运动策略、SDK2/DDS、ROS 2 Bridge、正式隐藏赛场坐标、裁判真值或评分配置。

