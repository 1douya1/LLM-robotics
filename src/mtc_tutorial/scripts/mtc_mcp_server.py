#!/usr/bin/env python3
import os
import sys
from pathlib import Path
from typing import Optional, Dict, Any

# 确保可以导入同目录下的工具脚本
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.append(str(_THIS_DIR))

try:
    # 需要安装：pip install "mcp[stdio]"
    from mcp.server.fastmcp import FastMCP
except Exception as e:
    sys.stderr.write("[mtc_mcp_server] Missing MCP SDK. Please install: pip install 'mcp[stdio]'\n")
    raise

import ros_client_tools as tools

server = FastMCP("mtc_mcp_server")


@server.tool()
async def execute_pour(params: Dict[str, Any],action_name: str = 'execute_pour',timeout_sec: float = 180.0,cancel_after: Optional[float] = None) -> Dict[str, Any]:
    """执行一次倒水动作。params 字段与 pour_tool.DEFAULTS 一致。"""
    return tools.execute_pour(params, action_name=action_name,timeout_sec=timeout_sec, cancel_after=cancel_after)


@server.tool()
async def set_cup_pose(x: float, y: float, z: float, qx: Optional[float] = None, qy: Optional[float] = None, qz: Optional[float] = None, qw: Optional[float] = None, valid: bool = True,server_node: str = '/execute_pour_server',timeout_sec: float = 5.0) -> Dict[str, Any]:
    """设置杯子的位姿参数"""
    ok = tools.set_cup_pose(x, y, z, qx=qx, qy=qy, qz=qz, qw=qw, valid=valid,server_node=server_node, timeout_sec=timeout_sec)
    return {"ok": bool(ok)}


@server.tool()
async def set_gripper_close_ratio(ratio: float,server_node: str = '/execute_pour_server',timeout_sec: float = 5.0) -> Dict[str, Any]:
    """设置夹爪闭合比例 (0.0-1.0)"""
    ok = tools.set_gripper_close_ratio(ratio, server_node=server_node, timeout_sec=timeout_sec)
    return {"ok": bool(ok)}


# =============== Agent的辅助工具 ===============

@server.tool()
async def get_task_state(timeout_sec: float = 5.0) -> Dict[str, Any]:
    """获取当前任务状态 - Agent友好的系统状态检查工具
    
    Returns:
        详细的任务状态信息，包括：
        - stage: 当前阶段 ("idle", "planning", "executing", "completed", "error")
        - last_error: 最近的错误信息 
        - robot_pose: 机器人末端执行器位姿
        - gripper_state: 夹爪状态 ("open", "closed", "grasping")
        - action_status: Action服务器状态
        - system_ready: 系统是否准备好执行任务
        - timestamp: 状态检查时间戳
    
    这个工具帮助Agent了解当前系统状态，制定下一步决策。
    """
    return tools.get_task_state(timeout_sec=timeout_sec)


@server.tool()
async def abort_and_reset(reason: str = "Agent requested emergency stop",
                          safe_pose_joints: Optional[list] = None, 
                          timeout_sec: float = 30.0) -> Dict[str, Any]:
    """紧急中止当前任务并重置到安全状态 - Agent友好的失败恢复工具
    
    Args:
        reason: 中止原因，用于日志记录和调试
        safe_pose_joints: 安全位姿的关节角度 (可选，默认使用预设安全姿态)
        timeout_sec: 操作超时时间
    
    Returns:
        详细的执行结果，包括：
        - success: 是否成功完成中止和重置
        - reason: 中止原因
        - actions_taken: 执行的具体动作列表
        - warnings: 警告信息
        - robot_safe: 机器人是否处于安全状态
        - timestamp: 操作时间戳
    
    这个工具在任务出现问题时提供紧急恢复能力，确保机器人和环境安全。
    """
    return tools.abort_and_reset(reason=reason, safe_pose_joints=safe_pose_joints, timeout_sec=timeout_sec)


if __name__ == "__main__":
    # 通过 STDIO 运行 MCP 服务器（显式指定 stdio 以兼容不同客户端）
    server.run(transport='stdio') 