# MuJoCo速度设置与实测

2026-11-15。普通行走和窄口的默认前进上限已从0.20/0.08m/s提高到0.50m/s。这是规划器上限，弯道、障碍、接近检查点和制动检查仍会减速或停车，不是强制全程恒速。

## 实时窗口

在项目根目录执行，重新启动进程后生效：

```bash
.venv/bin/go2-rescue mujoco --viewer --task-fixture --course-policy ordered --max-speed .5 --seconds 600 --output "output/fast-$(date +%Y%m%d-%H%M%S)"
```

现在也可以将`.5`换为`.3`等数值，无需编辑源码。启动日志会打印实际采用的普通/窄口前进上限。

配置优先级是命令行`--max-speed` → `--config`里的`max_vx`和`passage_max_vx` → 无配置文件时的仿真默认0.5。修复了仿真初始化无条件覆盖配置文件速度的问题。旧配置文件未填写`passage_max_vx`时仍采用该字段的0.08默认值；显式传`--max-speed .5`可同时覆盖两项。

侧移、转向、恢复退出是不同参数，本次没有将它们都改为0.5。未知实物的默认前进0.3m/s、窄口0.08m/s及`calibrated=false`不变，未向实物发送命令。

## 后续死胡同倒退提速

明确标记reverse的路线段，仿真`max_reverse_vx`默认为0.25m/s，CLI `--max-reverse-speed`独立覆盖；`--max-speed`仅调前进。普通ordered检查点和SE(2)恢复保留最多0.08m/s的小幅倒车，缓冲区退出最多0.05m/s；这次提速不扩大车身/水面/后方制动约束。实物默认仍0.08m/s。见[死胡同验证](DEAD_END.md)。

## 实现

- `mujoco_sim.simulation_config`提供仿真速度默认及命令行覆盖。
- `Autonomy.passage_dwa`读取`Config.passage_max_vx`，不再硬编码0.08m/s的前进上限。
- DWA前视距离加入制动距离项，最大1.2m，避免短前视点限制持续巡航；完整机身、地面、净空、水面和制动轨迹检查保留。
- 原MPC的普通2.5Hz步态继续使用。试验性的高速步态/力上限改动已撤回，没有提高执行器力矩上限、修改官方场景或运行中搬动根位姿。

后续分叉口排查已将标记窄口的额外余量恢复为`min(config.safety_margin, .02)`，保留完整车身和水面检查；S弯末端提前在转向区域对齐。视频/归档默认关闭，详见[性能报告](PERFORMANCE.md)。

## 实测结果

`speed-smoke`是脚本控制器专项：1秒后逐渐加速，中段巡航，6秒前逐渐减速，然后停车；它不使用自主导航。下列两个指定起点的专项都运行8仿真秒，并每2毫秒检查物理接触。

| 记录 | 期望速度 | 实测巡航均值 | 最后1秒净速度 | 最大侧偏 | 障碍接触/跌倒 |
|---|---:|---:|---:|---:|---:|
| `output/speed-05-flat-05`，开阔地面 | 0.50 | 0.596m/s | 0.036m/s | 0.011m | 0/0 |
| `output/speed-05-u-exit-02`，U形出口 | 0.50 | 0.582m/s | 0.012m/s | 0.018m | 0/0 |

两次均无入水、越界；`speed-check.json`通过。实际速度高于指令约16～19%，不能把0.5m/s上限说成已精确跟踪0.500m/s。这里只验证指定起点、单一方向的短时控制，不代表窄口的定位规划闭环或实物速度精度。

`output/speed-05-autonomy-01`从原起点使用共同Autonomy运行30仿真秒，规划指令多次达到0.48～0.50m/s；26.2秒完成L1上坡阶段，实际接触上坡和平台，未跌倒/碰撞/入水/越界，30秒上限结束于L1_descent。尚未完成下坡和全程，`full_course_pass=false`。高速时的定位误差仍需长测，不代表整圈成功率。

早期`speed-05-flat-01/02/03`从原起点向东直行的测试逼近弯墙并失稳，测试选段不合适，保留原始失败记录；不能据此断言原MPC在开阔平地不能达到0.5m/s。

最终全量测试：96项通过。新增覆盖速度配置优先级/非法值、0.5m/s巡航及障碍制动拒绝、原始模型力矩控制加速/停车；官方SHA256清单继续校验。

## 重复专项

```bash
.venv/bin/go2-rescue mujoco --control speed-smoke --max-speed .5 --spawn 6.4 2.65 --course-policy explore --seconds 8 --save-physics --output output/my-speed-test
.venv/bin/python scripts/verify_speed.py output/my-speed-test
```

必须使用新目录。`--save-physics`显式保存实际刚体结果`physics.npz`；视频另需`--video`，默认关闭。验证脚本只读物理记录生成`speed-check.json`，不会把控制器专项标为完整任务成功。

## 2号圆管

2号入口22厘米台阶的自主接入仍未完成，默认ordered到此仍会明确停车。`--max-speed`不会解除越阶能力检查。

历史干净单段`pipe2-stage-10`的入口裁剪是0.045m/s；当前工作区已有的入口裁剪为前进0.45m/s、倒退0.3m/s，本次保留该设置，但没有验证它在台阶处的稳定性。旧62.1秒通过记录不适用于这项新裁剪，不能声称2号台阶已按0.5m/s通过。详见`docs/PIPE2.md`。
