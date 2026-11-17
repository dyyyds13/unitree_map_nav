# 新仓库发布前整理

这份清单记录新仓库发布前的整理结果。实际提交、推送和远程地址以Git及GitHub上的状态为准。项目根目录是 `unitree_map_nav/`，不是它的父目录。

## 已整理内容

- README围绕Python3.12/WSL安装、便携启动、90%建图策略、仿真MPC与实物SDK的区别重写。
- 新增仓库内 `start-simulation.sh`，去除机器路径及NVIDIA型号绑定；父目录旧脚本保留供本机继续使用。
- `.gitignore` 排除输出、虚拟环境、构建产物和MuJoCo调试文件；`MJDATA.TXT` 留在本机，不删除。
- `.gitattributes` 指定Python/shell/TOML使用LF；不改变原始资源及SHA覆盖文件。
- 最新90%实跑摘要、配置、场景来源复制到 `docs/validation/mapping-search-20261117/`；大日志不发布。
- 增加只读 `scripts/check_repository.py`；保留资源NOTICE、LICENSES、MPC许可证和固定来源提交。

## 发布边界

原始 [NOTICE.md](../NOTICE.md) 明确声明：障碍和生命探测目标不构成MIT、商用或任意再分发许可，正式对外分发前须由资源所有方确认授权。此次未替资源所有方作授权判断，也未新增覆盖整个项目的开放许可证；发布可见性和资产授权需按该原文处理。

待发布文件约120MiB；最大文件是约62.58MiB的 `scene/mujoco/competition-course.mjb`。当前未使用Git LFS，检查脚本会报告≥50MiB文件及阻止≥100MiB文件。若以后采用LFS，应单独制定资源和历史迁移方案；本次不改已有资源存储或Git历史。

目前仍有误报/重复报告、后期CPU卡顿及正式协议缺失。最新单次运行虽到终点并匹配3球，严格 `local_task_pass` 仍为false，不将 `mission_complete=true` 当作完整搜救任务通过。

## 本地验证

```bash
bash -n start-simulation.sh
MUJOCO_GL=glfw .venv/bin/python -m pytest -q
.venv/bin/python -m pip check
.venv/bin/python scripts/check_repository.py
git diff --check
git status --short
```

本次检查结果：

| 项目 | 结果 |
|---|---|
| 完整软件测试 | 136项通过，62.44秒 |
| 依赖一致性 | `pip check` 通过 |
| 原始资源校验 | 769项SHA256通过 |
| 发布文件检查 | 初步检查无生成文件混入、≥100MiB单文件或凭据模式命中 |
| 可移植脚本 | 从 `/tmp` 调用仓库脚本，2仿真秒正常到时结束；D3D12 GPU渲染；未跌倒/报错 |
| shell语法与差异格式 | `bash -n`、`git diff --check` 通过 |

短仿真只检查调用链，不是完整任务验收。未重装已有环境，未在第二台电脑验证；完整环境安装仍需克隆者按README执行。凭据检查仅覆盖当前待发布文件的有限模式，不审计Git全历史。

本次提交范围包括此前未提交的Python3.12适配、死胡同参数、搜索任务、场景/评估/测试及文档，不能将整个工作树描述为“只有README变化”。新增配置、`search.py`、`test_search.py`、便携脚本和小型验证资料需一并提交。`output/`、`.venv`、`MJDATA.TXT`应保持忽略。

## 后续维护时检查远程和提交

先检查当前远程和工作树：

```bash
git remote -v
git status --short
git diff --stat
```

用户已授权创建新仓库并推送。私有仓库为 [dyyyds13/unitree_map_nav](https://github.com/dyyyds13/unitree_map_nav)，当前 `personal` 已指向该仓库；`origin` 保留原仓库 `liangjihui643-beep/unitree_map_nav`。后续提交使用 `personal`，保留原有Git历史。

```bash
git remote get-url personal
```

确认文件后再暂存和提交。以下是后续维护时的命令示例：

```bash
git add -- .
git diff --cached --stat
git diff --cached --check
git commit -m "Prepare Go2 mapping and rescue simulation for publication"
git push -u personal HEAD
```

这样保留原有Git历史。不要删除 `.git` 或强推来解决普通远程配置问题。若需要独立新历史，应在单独目录另行准备，不破坏当前历史。
