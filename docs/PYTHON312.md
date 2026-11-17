# Python 3.12 仿真环境

最新状态：用户再次要求使用比赛所需Python3.12，项目 `.venv` 已切回保留的 `venv312`（3.12.3），MuJoCo保持3.17.0。依赖锁定版本不变，对应3.12包均可导入，pip check通过。此前3.10回退历史见 [回退记录](ROLLBACK.md)。

最新长程验证：前墙检查目标与靠近限速调整后，两次原起点seed0测试均106.5秒退出死胡同；实际窗口130秒逐物理步审计无障碍接触/进水/越界/跌倒。260秒测试最终停在管道入口，未完成整场。119项测试通过（70.44秒）。见 [参数调整](DEAD_END_TUNING.md)。下文早期短测和未恢复状态为历史记录。

回归状态更正：后续130秒对照中，3.10和3.12都复现死胡同卡住。以下安装与短测记录不代表恢复历史长路线表现；同一份改过代码的Python对照也不能排除先前代码改动。相机快速渲染改动已经撤回，详见 [回归审查](VERSION_AUDIT.md)。

2026-11-16，本机 Ubuntu 24.04 / WSL2 已切换到 Python 3.12.3。无需修改算法或仿真源码，也无需升级原锁文件中的依赖版本。

## 改动位置

- `.python-version`：新增 `3.12`，供支持此文件的环境管理工具选择解释器；它不会自动改变已存在的虚拟环境。
- `README.md`：创建环境的命令改为 `python3.12 -m venv .venv`，更新验证范围。
- `requirements-v1.lock.txt`：只更新验证环境注释，所有依赖版本不变。MuJoCo3.17.0、CasADi3.6.7、NumPy1.26.4、SciPy1.13.1及KISS-ICP1.3.0均已安装。
- `pyproject.toml`：现有 `requires-python = ">=3.10"` 已允许3.12，无需修改；保留原3.10兼容范围。
- 本机 `.venv`：链接目标从 `/root/.local/share/go2-rescue/venv` 改为 `/root/.local/share/go2-rescue/venv312`。前者仍保留3.10环境，后者是独立创建的3.12环境。虚拟环境不随Git分发。

## 在另一台 Linux / WSL 中安装

需要系统已有 `python3.12` 和对应venv支持。在新的项目目录中执行；已有3.10的 `.venv` 时先备份或移开它，不要混用两个版本的site-packages。

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-v1.lock.txt
.venv/bin/python -m pip install -e . --no-deps
.venv/bin/python --version
.venv/bin/python -m pip check
.venv/bin/python -m pytest -q
```

本机Windows启动命令保持不变，脚本继续使用项目 `.venv`，因此现在运行的是3.12：

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/d/Desktop/Mujuco/start-simulation.sh
```

## 实际验证

- 完整测试：119 passed in64.82s，含真实MuJoCo动力学、感知及慢窗口回归测试；依赖检查无冲突。
- 原起点、固定顺序路线、入口坡本地场景，GPU窗口10仿真秒/13.377墙钟秒；D3D12 RTX5060，15Hz上限和快速渲染，102次窗口同步、94次相机更新。正常到时停车，无报错、跌倒或诊断障碍接触。
- 同种子10秒短测的终止物理位置与此前3.10记录一致。结果保留在本地 `output/wsl-setup/python312-viewer/`。

本轮没有重跑260秒完整串联，也未安装/验证实物厂家SDK。此前3.10的长路线证据不能改称为3.12的长路线验证。3.12可运行也不代表整个比赛任务已经完成。
