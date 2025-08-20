#!/usr/bin/env bash
set -eo pipefail

# 输出调试信息到stderr（不影响MCP协议）
echo "[run_mtc_mcp_fixed] Starting MCP server..." >&2

# Try source common ROS 2 distros
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "[run_mtc_mcp_fixed] Sourcing ROS 2 Humble..." >&2
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    echo "[run_mtc_mcp_fixed] Sourcing ROS 2 Iron..." >&2
    source /opt/ros/iron/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    echo "[run_mtc_mcp_fixed] Sourcing ROS 2 Jazzy..." >&2
    source /opt/ros/jazzy/setup.bash
else
    echo "[run_mtc_mcp_fixed] WARNING: No ROS 2 installation found" >&2
fi

# mtc_interface is now part of this workspace, no need to source separately

if [ -f /home/wenhao/uf_custom_ws/install/setup.bash ]; then
    echo "[run_mtc_mcp_fixed] Sourcing uf_custom workspace..." >&2
    source /home/wenhao/uf_custom_ws/install/setup.bash
else
    echo "[run_mtc_mcp_fixed] WARNING: uf_custom workspace not found" >&2
fi

# Avoid ROS logging to stdout; MCP uses stdout for protocol
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_COLORIZED_OUTPUT=0
export PYTHONUNBUFFERED=1

# 测试依赖是否可用
echo "[run_mtc_mcp_fixed] Testing dependencies..." >&2
python3 - <<'PY' 1>&2
import sys
try:
    import rclpy
    print(f"[run_mtc_mcp_fixed] ✓ rclpy: {rclpy.__file__}")
except Exception as e:
    print(f"[run_mtc_mcp_fixed] ✗ rclpy: {e}")

try:
    import mcp
    print(f"[run_mtc_mcp_fixed] ✓ MCP SDK: {mcp.__file__}")
except Exception as e:
    print(f"[run_mtc_mcp_fixed] ✗ MCP SDK: {e}")
    sys.exit(1)

try:
    from mtc_interface.action import ExecutePour
    print("[run_mtc_mcp_fixed] ✓ mtc_interface available")
except Exception as e:
    print(f"[run_mtc_mcp_fixed] ✗ mtc_interface: {e}")
    print("[run_mtc_mcp_fixed] MTC functionality will be limited")
PY

echo "[run_mtc_mcp_fixed] Starting MCP server..." >&2
exec python3 /home/wenhao/uf_custom_ws/src/mtc_tutorial/scripts/mtc_mcp_server.py 