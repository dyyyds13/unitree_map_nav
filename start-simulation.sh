#!/usr/bin/env bash
# Portable WSL/Linux launcher; use GO2_HEADLESS=1 for a run without a viewer.
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
if [[ ! -x .venv/bin/go2-rescue ]]; then
    echo 'Missing .venv/bin/go2-rescue. Follow README.md to install Python 3.12 dependencies.' >&2
    exit 1
fi

# WSLg uses the Windows GPU through Mesa D3D12. Honor explicit user overrides.
if [[ -e /dev/dxg && -d /usr/lib/wsl/lib ]]; then
    export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
simulation_args=(--fast-render --task-fixture --task-layout rubble-search
                 --course-policy search --seconds 1200
                 --output "output/run-$(date +%Y%m%d-%H%M%S-%N)")
if [[ "${GO2_HEADLESS:-0}" == 1 ]]; then
    # CLI otherwise defaults to EGL, which can use software rendering on WSL.
    if [[ -e /dev/dxg ]]; then
        export MUJOCO_GL="${MUJOCO_GL:-glfw}"
    fi
else
    simulation_args+=(--viewer --realtime --viewer-fps 15)
fi
exec .venv/bin/go2-rescue mujoco "${simulation_args[@]}" "$@"
