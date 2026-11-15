# 本地入口坡场景与实时前视相机

2026-11-15，用户明确要求修改地图。官方原始MJB、XML、网格与SHA清单覆盖资源均不改动。

```bash
cd /home/l/下载/robotics-bench-go2-competition-training-resources
.venv/bin/go2-rescue mujoco --viewer --realtime --task-fixture --pipe-ramp entry --seconds 600 --output "output/local-$(date +%Y%m%d-%H%M%S)"
```

`--task-fixture`下默认即为entry，可省略`--pipe-ramp entry`；显式`--pipe-ramp official`恢复官方原布局。没有task-fixture时默认仍加载原MJB，entry须配合task-fixture。机器人原起点、目标球、其他障碍不变。

## 实际几何修改

将原`tunnel2_to3_sloped_continuous_threshold`绕2号管道中心旋转180°，从东侧移至西侧入口，重新编译碰撞模型；不是只移动视觉模型。原坡形状、尺寸、摩擦和碰撞参数保留，其他静态几何逐顶点核对。

在y=6.22米沿入口测量的地面高度：

| 世界x(m) | 地面z(m) |
|---:|---:|
| 7.85 | 0 |
| 7.96 | 0.0034 |
| 8.05 | 0.0338 |
| 8.20 | 0.0846 |
| 8.40 | 0.1523 |
| 8.58 | 0.2132 |
| 8.65 | 0.2200 |

原东侧x=10.9处现为地面0，斜坡已移走；因此出口有22厘米下降台阶，仍需专门控制/验证。没有拉长坡道、修改管内地面或删除管壁。scene_provenance.json记录源哈希、唯一改动和预期几何误差；summary和窗口标记LOCAL: RAMP AT ENTRY。modified_course_geometry=true的记录不会作为官方地图整圈通过证据。自主过管能力保护仍在，本次未完成入管/出管自主控制验收。

## 相机显示

带--viewer时，右下角默认显示ROBOT FRONT CAMERA。图像来自当前导航实际使用的320×240、10Hz相机观测，较大窗口显示至480×360；不额外渲染相机、不写视频、不保留历史图像。缩放窗口会自适应。--hide-camera可隐藏。

--video / --record / --save-physics / --snapshots仍全部默认关闭。本次不自动导出大场景二进制。

## 验证

- 几何射线确认上述连续入口坡、原出口坡移走；每个未修改部件与原MJB保持一致，原始SHA清单全通过。
- `entry-ramp-camera-01`：管道前指定起点的脚本MPC窗口短测8sim/8.008wall秒，433次窗口sync、75次前视更新，未跌倒；录制全关闭。不是自主过管或整圈。
- 实际窗口已人工检查，能同时看到新入口坡和前视图像。
- `entry-ramp-autonomy-01`：原起点Autonomy真实窗口12sim/17.935wall秒（并行运行测试，不是性能基准），115次前视更新、562次sync，未跌倒；无媒体/物理归档，未到管道。
- 冻结画面Ctrl+C关闭曾触发GLFW/渲染线程退出竞争（preview进程139，保留记录）；修复后`entry-ramp-close-01`为0，自动关闭的autonomy-01也为0。
- 全量107项测试通过（43.50秒）；另有最终尺寸适配测试通过，pip/compile/原始资源SHA通过。未执行实物运动。
