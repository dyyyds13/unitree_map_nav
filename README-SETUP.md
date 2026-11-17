# 环境与开发说明

安装与启动入口已统一到根目录 [README.md](README.md)。使用Ubuntu 24.04 / WSL2、Python3.12创建本机虚拟环境，按 `requirements-v1.lock.txt` 安装，然后在完整仓库中以可编辑方式安装项目。

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements-v1.lock.txt
.venv/bin/python -m pip install -e . --no-deps
bash start-simulation.sh
```

系统OpenGL库、WSLg和运行参数详见README。仿真需要仓库中的原始场景、机器人资源和固定版本MPC源码；仅复制Python源码或安装一个wheel不能替代完整仓库。`.venv`、输出日志和缓存不提交Git，也不跨机器复制。

开发前阅读 [AGENTS.md](AGENTS.md)、[当前状态](docs/CURRENT_STATE.md) 和相关模块。`docs/HANDOFF.md` 及较早的版本/回退记录是历史资料；实现以代码和测试为准，最新公开运行状态以README及其链接的验证资料为准。

发布新仓库参见 [发布前清单](docs/RELEASE_PREPARATION.md)。原资源NOTICE和第三方许可证保持原样，不将来源资产统一改为项目自有许可。
