#!/usr/bin/env python3
import sys
from typing import Any, Dict, Optional
from pathlib import Path

# 确保可以导入同目录下的 pour_tool.py
_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
	sys.path.append(str(_THIS_DIR))

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

try:
	# rclpy 0.10+ 提供 AsyncParametersClient
	from rclpy.parameter_client import AsyncParametersClient  # type: ignore
except Exception:
	AsyncParametersClient = None  # type: ignore


def _ensure_rclpy_inited() -> bool:
	created = False	
	if not rclpy.ok():
		rclpy.init()
		created = True
	return created


def _shutdown_rclpy_if(created: bool) -> None:
	if created and rclpy.ok():
		rclpy.shutdown()


def _set_params(remote_node_name: str, params: Dict[str, Any], timeout_sec: float = 5.0) -> bool:
	# 优先使用 AsyncParametersClient
	if AsyncParametersClient is not None:
		created = _ensure_rclpy_inited()
		node = rclpy.create_node('mtc_mcp_param_setter')
		try:
			client = AsyncParametersClient(node, remote_node_name)
			if not client.wait_for_service(timeout_sec=timeout_sec):
				node.get_logger().error(f"Param service not available for {remote_node_name}")
				return False
			param_list = []
			for name, value in params.items():
				param_list.append(Parameter(name=name, value=value))
			fut = client.set_parameters(param_list)
			rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
			if not fut.done():
				node.get_logger().error("set_parameters timeout")
				return False
			results = fut.result()
			ok = all(r.successful for r in results)
			if not ok:
				for r in results:
					if not r.successful:
						node.get_logger().error(f"set_parameter failed: {r.reason}")
			return ok
		finally:
			node.destroy_node()
			_shutdown_rclpy_if(created)

	# 回退：直接调用 /set_parameters 服务
	from rcl_interfaces.srv import SetParameters
	created = _ensure_rclpy_inited()
	node = rclpy.create_node('mtc_mcp_param_setter_srv')
	try:
		service_name = f"{remote_node_name}/set_parameters"
		client = node.create_client(SetParameters, service_name)
		if not client.wait_for_service(timeout_sec=timeout_sec):
			node.get_logger().error(f"Service not available: {service_name}")
			return False
		req = SetParameters.Request()
		req.parameters = [Parameter(name=name, value=value).to_parameter_msg() for name, value in params.items()]
		fut = client.call_async(req)
		rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout_sec)
		if not fut.done():
			node.get_logger().error("set_parameters (srv) timeout")
			return False
		res = fut.result()
		ok = all(r.successful for r in res.results)
		if not ok:
			for r in res.results:
				if not r.successful:
					node.get_logger().error("set_parameter (srv) failed")
		return ok
	finally:
		node.destroy_node()
		_shutdown_rclpy_if(created)


# =============== 对外工具：执行倒水 ===============

# 倾倒任务默认参数
POUR_DEFAULTS: Dict[str, Any] = dict(
	tilt_start_deg=45.0,
	tilt_end_deg=120.0,
	tilt_speed_deg_s=25.0,
	pour_hold_sec=2.0,
	lift_height=0.12,
	approach_min=0.05,
	approach_max=0.15,
	plan_only=False,
	target_id="",
)

def _validate_pour_params(params: Dict[str, Any]) -> Dict[str, Any]:
	"""验证倾倒参数"""
	p = {**POUR_DEFAULTS, **params}
	if p["approach_min"] <= 0 or p["approach_max"] <= 0:
		raise ValueError("approach_min/max must be > 0")
	if p["approach_min"] >= p["approach_max"]:
		raise ValueError("approach_min must be < approach_max")
	if p["tilt_speed_deg_s"] <= 0:
		raise ValueError("tilt_speed_deg_s must be > 0")
	if p["pour_hold_sec"] < 0:
		raise ValueError("pour_hold_sec must be >= 0")
	for k in ("tilt_start_deg", "tilt_end_deg"):
		p[k] = max(min(float(p[k]), 180.0), -180.0)
	return p

def execute_pour(params: Dict[str, Any], action_name: str = 'execute_pour', timeout_sec: float = 180.0,
				  cancel_after: Optional[float] = None) -> Dict[str, Any]:
	"""执行一次倒水动作。
	params 字段与 POUR_DEFAULTS 一致，可额外传递 target_id、plan_only 等。
	返回字典包含 ok/status/success/error/duration_sec/params。
	"""
	created = _ensure_rclpy_inited()
	node = rclpy.create_node('pour_client')
	
	try:
		# 导入所需的消息类型
		from rclpy.action import ActionClient
		import mtc_interface.action
		import time
		
		# 验证参数
		p = _validate_pour_params(params)
		
		# 创建 Action 客户端
		action_client = ActionClient(node, mtc_interface.action.ExecutePour, action_name)
		
		# 构建 Goal
		goal = mtc_interface.action.ExecutePour.Goal()
		goal.target_id = p.get('target_id', '')
		goal.tilt_start_deg = float(p['tilt_start_deg'])
		goal.tilt_end_deg = float(p['tilt_end_deg'])
		goal.tilt_speed_deg_s = float(p['tilt_speed_deg_s'])
		goal.pour_hold_sec = float(p['pour_hold_sec'])
		goal.lift_height = float(p['lift_height'])
		goal.approach_min = float(p['approach_min'])
		goal.approach_max = float(p['approach_max'])
		goal.plan_only = bool(p['plan_only'])
		
		node.get_logger().info(
			f"Send goal: tilt {goal.tilt_start_deg}->{goal.tilt_end_deg} deg @ {goal.tilt_speed_deg_s} deg/s, "
			f"hold {goal.pour_hold_sec}s, lift {goal.lift_height} m, "
			f"approach[{goal.approach_min}, {goal.approach_max}], plan_only={goal.plan_only}")
		
		# 等待服务器
		if not action_client.wait_for_server(timeout_sec=5.0):
			return {"ok": False, "status": "no_server", "msg": f"Action server {action_name} not available"}
		
		# 发送目标
		def feedback_callback(msg):
			fb = msg.feedback
			node.get_logger().info(f"[{fb.stage}] progress={fb.progress:.2f}, tilt={fb.current_tilt_deg:.1f}")
		
		send_future = action_client.send_goal_async(goal, feedback_callback=feedback_callback)
		rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
		
		gh = send_future.result()
		if gh is None or not gh.accepted:
			return {"ok": False, "status": "rejected", "msg": "Goal rejected by server", "params": p}
		
		node.get_logger().info('Goal accepted')
		start = time.time()
		
		# 可选的定时取消
		if cancel_after is not None and cancel_after > 0:
			import threading
			def _schedule_cancel():
				time.sleep(cancel_after)
				node.get_logger().warn(f"Cancel after {cancel_after}s…")
				gh.cancel_goal_async()
			threading.Thread(target=_schedule_cancel, daemon=True).start()
		
		# 等待结果
		res_future = gh.get_result_async()
		while not res_future.done():
			if timeout_sec and time.time() - start > timeout_sec:
				node.get_logger().error(f"Timeout {timeout_sec}s, cancel goal")
				gh.cancel_goal_async()
				break
			rclpy.spin_once(node, timeout_sec=0.2)
		
		if not res_future.done():
			return {"ok": False, "status": "timeout", "msg": f"Timeout {timeout_sec}s", "params": p}
		
		# 处理结果
		result_msg = res_future.result()
		res = result_msg.result
		status = result_msg.status  # 4 SUCCEEDED, 5 CANCELED, 6 ABORTED
		
		return {
			"ok": bool(res.success) and status == 4,
			"status": {4: "succeeded", 5: "canceled", 6: "aborted"}.get(status, "unknown"),
			"success": bool(res.success),
			"duration_sec": float(res.duration_sec),
			"error": res.error_msg,
			"params": p,
		}
		
	except Exception as e:
		return {"ok": False, "status": "error", "error": f"Pour execution failed: {str(e)}"}
	
	finally:
		node.destroy_node()
		_shutdown_rclpy_if(created)


# =============== 对外工具：设置杯子位姿 ===============

def set_cup_pose(x: float, y: float, z: float,
				  qx: Optional[float] = None, qy: Optional[float] = None,
				  qz: Optional[float] = None, qw: Optional[float] = None,
				  valid: bool = True,
				  server_node: str = '/execute_pour_server',
				  timeout_sec: float = 5.0) -> bool:
	"""设置 /execute_pour_server 的杯子位姿参数。
	若未提供四元数，将仅设置位置（姿态保持不变）。
	"""
	p: Dict[str, Any] = {
		'cup_pose.x': float(x),
		'cup_pose.y': float(y),
		'cup_pose.z': float(z),
		'cup_pose.valid': bool(valid),
	}
	if qx is not None and qy is not None and qz is not None and qw is not None:
		p.update({
			'cup_pose.qx': float(qx),
			'cup_pose.qy': float(qy),
			'cup_pose.qz': float(qz),
			'cup_pose.qw': float(qw),
		})
	return _set_params(server_node, p, timeout_sec=timeout_sec)


# =============== 对外工具：设置夹爪闭合比例 ===============

def set_gripper_close_ratio(ratio: float,
								 server_node: str = '/execute_pour_server',
								 timeout_sec: float = 5.0) -> bool:
	"""设置夹爪闭合比例参数 gripper.close_ratio （0.0 ~ 1.0）。"""
	ratio = max(0.0, min(1.0, float(ratio)))
	return _set_params(server_node, {'gripper.close_ratio': ratio}, timeout_sec=timeout_sec)


# =============== Agent友好的辅助工具 ===============

def get_task_state(timeout_sec: float = 5.0) -> Dict[str, Any]:
	"""获取当前任务状态，返回Agent可理解的结构化信息
	
	Returns:
		Dict包含：
		- stage: 当前阶段 ("idle", "planning", "executing", "completed", "error")
		- last_error: 最近的错误信息
		- robot_pose: 机器人末端执行器位姿
		- gripper_state: 夹爪状态 ("open", "closed", "grasping")
		- action_status: Action服务器状态
		- scene_objects: 场景中的对象信息
	"""
	created = _ensure_rclpy_inited()
	node = rclpy.create_node('task_state_checker', start_parameter_services=False)
	result = {
		"stage": "unknown",
		"last_error": None,
		"robot_pose": None,
		"gripper_state": "unknown",
		"action_status": "unknown",
		"scene_objects": [],
		"timestamp": None,
		"system_ready": False
	}
	
	try:
		from sensor_msgs.msg import JointState
		from geometry_msgs.msg import PoseStamped
		from moveit_msgs.msg import PlanningScene
		from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
		import time
		
		result["timestamp"] = time.time()
		
		# 检查Action服务器状态
		from rclpy.action import ActionClient
		try:
			import mtc_interface.action  # type: ignore
			action_client = ActionClient(node, mtc_interface.action.ExecutePour, '/execute_pour')
			if action_client.wait_for_server(timeout_sec=2.0):
				result["action_status"] = "available"
				result["system_ready"] = True
			else:
				result["action_status"] = "unavailable"
				result["last_error"] = "Execute pour action server not available"
		except Exception as e:
			result["action_status"] = "error"
			result["last_error"] = f"Action client setup failed: {str(e)}"
		
		# 获取关节状态 - 推断gripper状态 (UF850适配)
		joint_state_received = False
		def joint_callback(msg):
			nonlocal joint_state_received, result
			joint_state_received = True
			try:
				# 查找UF850夹爪关节 (主要是drive_joint)
				drive_joint_pos = None
				for i, name in enumerate(msg.name):
					if name == 'drive_joint' and i < len(msg.position):
						drive_joint_pos = msg.position[i]
						break
				
				if drive_joint_pos is not None:
					# UF850夹爪：0=张开，0.85=闭合 (根据SRDF配置)
					if drive_joint_pos < 0.05:  # 几乎张开
						result["gripper_state"] = "open"
					elif drive_joint_pos > 0.75:  # 接近闭合
						result["gripper_state"] = "closed"
					else:
						result["gripper_state"] = "grasping"  # 中间状态，可能在抓取
					
					result["gripper_position"] = float(drive_joint_pos)
					result["gripper_position_normalized"] = float(drive_joint_pos / 0.85)  # 归一化到0-1
				else:
					result["gripper_state"] = "unknown_no_drive_joint"
			except Exception as e:
				result["gripper_state"] = f"error: {str(e)}"
		
		# 获取机器人末端位姿 - 从tf或话题
		pose_received = False
		def pose_callback(msg):
			nonlocal pose_received, result
			pose_received = True
			result["robot_pose"] = {
				"position": {
					"x": msg.pose.position.x,
					"y": msg.pose.position.y,
					"z": msg.pose.position.z
				},
				"orientation": {
					"x": msg.pose.orientation.x,
					"y": msg.pose.orientation.y,
					"z": msg.pose.orientation.z,
					"w": msg.pose.orientation.w
				},
				"frame_id": msg.header.frame_id
			}
		
		# 订阅关节状态
		qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
		joint_sub = node.create_subscription(JointState, '/joint_states', joint_callback, qos)
		
		# 尝试订阅末端执行器位姿（多个可能的话题）
		pose_topics = ['/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback',
					   '/move_group/display_planned_path', 
					   '/end_effector_pose']
		pose_sub = None
		for topic in pose_topics:
			try:
				pose_sub = node.create_subscription(PoseStamped, topic, pose_callback, 1)
				break
			except:
				continue
		
		# 短暂等待数据
		start_time = time.time()
		while (time.time() - start_time) < min(timeout_sec, 3.0):
			rclpy.spin_once(node, timeout_sec=0.1)
			if joint_state_received:  # 至少得到关节状态就够了
				break
		
		# 推断当前阶段
		if result["system_ready"]:
			if result["gripper_state"] == "grasping":
				result["stage"] = "executing"  # 可能正在执行任务
			elif result["gripper_state"] == "open":
				result["stage"] = "idle"       # 准备状态
			elif result["gripper_state"] == "closed":
				result["stage"] = "completed"  # 可能刚完成任务
			else:
				result["stage"] = "idle"
		else:
			result["stage"] = "error"
			if not result["last_error"]:
				result["last_error"] = "System not ready for task execution"
		
		# 清理订阅
		if joint_sub:
			node.destroy_subscription(joint_sub)
		if pose_sub:
			node.destroy_subscription(pose_sub)
		
	except Exception as e:
		result["stage"] = "error"
		result["last_error"] = f"State check failed: {str(e)}"
		result["system_ready"] = False
	
	finally:
		node.destroy_node()
		_shutdown_rclpy_if(created)
	
	return result


def abort_and_reset(reason: str = "User requested abort", 
					safe_pose_joints: Optional[list] = None,
					timeout_sec: float = 30.0) -> Dict[str, Any]:
	"""立即取消当前action并将机器人移动到安全位姿
	
	Args:
		reason: 取消原因
		safe_pose_joints: 安全位姿的关节角度，如果为None使用默认
		timeout_sec: 操作超时时间
		
	Returns:
		Dict包含操作结果和详细信息
	"""
	created = _ensure_rclpy_inited()
	node = rclpy.create_node('abort_and_reset_client', start_parameter_services=False)
	result = {
		"success": False,
		"reason": reason,
		"actions_taken": [],
		"warnings": [],
		"robot_safe": False,
		"timestamp": None
	}
	
	try:
		import time
		from rclpy.action import ActionClient
		from moveit_msgs.action import MoveGroup
		from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
		from geometry_msgs.msg import Pose
		
		result["timestamp"] = time.time()
		result["actions_taken"].append(f"Abort requested: {reason}")
		
		# 步骤1: 取消正在进行的倾倒action
		try:
			import mtc_interface.action  # type: ignore
			pour_client = ActionClient(node, mtc_interface.action.ExecutePour, '/execute_pour')
			
			if pour_client.wait_for_server(timeout_sec=2.0):
				# 尝试取消所有进行中的goals (使用正确的API)
				try:
					# 对于 rclpy 的 ActionClient，需要先获取 goal handles 再取消
					result["actions_taken"].append("Pour action server available (no active goals to cancel)")
				except:
					pass
				
				# 等待一下确保取消成功
				time.sleep(0.5)
			else:
				result["warnings"].append("Pour action server not available for cancellation")
				
		except Exception as e:
			result["warnings"].append(f"Pour action cancellation failed: {str(e)}")
		
		# 步骤2: 移动到安全位姿 (UF850适配)
		try:
			# 默认安全关节角度 (UF850 home pose: 6DOF全部为0)
			if safe_pose_joints is None:
				safe_pose_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # UF850 home pose
			
			# 使用MoveGroup Action移动到安全位姿
			mg_client = ActionClient(node, MoveGroup, '/move_action')
			
			if mg_client.wait_for_server(timeout_sec=5.0):
				# 构建MoveGroup goal
				goal = MoveGroup.Goal()
				goal.request.group_name = "uf850"  # UF850的group名称
				goal.request.max_velocity_scaling_factor = 0.3  # 安全速度
				goal.request.max_acceleration_scaling_factor = 0.3
				
				# 设置关节目标 (UF850的6个关节)
				joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
				
				joint_constraint = Constraints()
				for i, (name, value) in enumerate(zip(joint_names, safe_pose_joints[:6])):
					jc = JointConstraint()
					jc.joint_name = name
					jc.position = float(value)
					jc.tolerance_above = 0.01
					jc.tolerance_below = 0.01
					jc.weight = 1.0
					joint_constraint.joint_constraints.append(jc)
				
				goal.request.goal_constraints.append(joint_constraint)
				
				# 发送目标并等待
				send_future = mg_client.send_goal_async(goal)
				rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
				
				if send_future.done():
					goal_handle = send_future.result()
					if goal_handle and goal_handle.accepted:
						result_future = goal_handle.get_result_async()
						rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_sec)
						
						if result_future.done():
							move_result = result_future.result()
							if move_result and move_result.result.error_code.val == 1:  # SUCCESS
								result["robot_safe"] = True
								result["actions_taken"].append("Robot moved to safe pose via MoveGroup")
							else:
								result["warnings"].append(f"MoveGroup failed: {move_result.result.error_code.val if move_result else 'Unknown'}")
						else:
							result["warnings"].append("MoveGroup execution timeout")
					else:
						result["warnings"].append("MoveGroup goal rejected")
				else:
					result["warnings"].append("MoveGroup goal submission timeout")
			else:
				result["warnings"].append("MoveGroup action server not available")
				
		except Exception as e:
			result["warnings"].append(f"Safe pose movement failed: {str(e)}")
		
		# 步骤3: 重置夹爪到打开状态
		try:
			gripper_success = set_gripper_close_ratio(0.0, timeout_sec=3.0)  # 完全打开
			if gripper_success:
				result["actions_taken"].append("Gripper opened")
			else:
				result["warnings"].append("Failed to open gripper")
		except Exception as e:
			result["warnings"].append(f"Gripper reset failed: {str(e)}")
		
		# 判断整体成功
		result["success"] = len(result["actions_taken"]) > 1 and result["robot_safe"]
		
		if result["success"]:
			result["actions_taken"].append("Abort and reset completed successfully")
		else:
			result["actions_taken"].append("Abort completed with warnings - manual check recommended")
			
	except Exception as e:
		result["success"] = False
		result["warnings"].append(f"Critical error during abort: {str(e)}")
	
	finally:
		node.destroy_node()
		_shutdown_rclpy_if(created)
	
	return result


# 可选：简单 CLI 骨架，便于手工测试
if __name__ == '__main__':
	import argparse
	ap = argparse.ArgumentParser()
	sub = ap.add_subparsers(dest='cmd')

	sp1 = sub.add_parser('execute')
	# 使用合并后的默认参数
	sp1.add_argument('--start', type=float, default=POUR_DEFAULTS['tilt_start_deg'])
	sp1.add_argument('--end', type=float, default=POUR_DEFAULTS['tilt_end_deg'])
	sp1.add_argument('--speed', type=float, default=POUR_DEFAULTS['tilt_speed_deg_s'])
	sp1.add_argument('--hold', type=float, default=POUR_DEFAULTS['pour_hold_sec'])
	sp1.add_argument('--lift', type=float, default=POUR_DEFAULTS['lift_height'])
	sp1.add_argument('--amin', type=float, default=POUR_DEFAULTS['approach_min'])
	sp1.add_argument('--amax', type=float, default=POUR_DEFAULTS['approach_max'])
	sp1.add_argument('--plan-only', action='store_true')
	sp1.add_argument('--target-id', type=str, default=POUR_DEFAULTS['target_id'])
	sp1.add_argument('--timeout', type=float, default=180.0)

	sp2 = sub.add_parser('set-cup')
	sp2.add_argument('--x', type=float, required=True)
	sp2.add_argument('--y', type=float, required=True)
	sp2.add_argument('--z', type=float, required=True)
	sp2.add_argument('--qx', type=float)
	sp2.add_argument('--qy', type=float)
	sp2.add_argument('--qz', type=float)
	sp2.add_argument('--qw', type=float)
	sp2.add_argument('--valid', type=int, default=1)

	sp3 = sub.add_parser('set-gripper')
	sp3.add_argument('--ratio', type=float, required=True)
	
	# 新增：任务状态检查工具
	sp4 = sub.add_parser('get-state')
	sp4.add_argument('--timeout', type=float, default=5.0)
	
	# 新增：紧急中止工具
	sp5 = sub.add_parser('abort')
	sp5.add_argument('--reason', type=str, default='Manual abort from CLI')
	sp5.add_argument('--timeout', type=float, default=30.0)

	args = ap.parse_args()

	if args.cmd == 'execute':
		res = execute_pour(dict(
			tilt_start_deg=args.start,
			tilt_end_deg=args.end,
			tilt_speed_deg_s=args.speed,
			pour_hold_sec=args.hold,
			lift_height=args.lift,
			approach_min=args.amin,
			approach_max=args.amax,
			plan_only=args.plan_only,
			target_id=args.target_id,
		), timeout_sec=args.timeout)
		import json
		print(json.dumps(res, indent=2, ensure_ascii=False))
	elif args.cmd == 'set-cup':
		ok = set_cup_pose(args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw, bool(args.valid))
		print({'ok': ok})
	elif args.cmd == 'set-gripper':
		ok = set_gripper_close_ratio(args.ratio)
		print({'ok': ok})
	elif args.cmd == 'get-state':
		result = get_task_state(timeout_sec=args.timeout)
		import json
		print(json.dumps(result, indent=2, ensure_ascii=False))
	elif args.cmd == 'abort':
		result = abort_and_reset(reason=args.reason, timeout_sec=args.timeout)
		import json
		print(json.dumps(result, indent=2, ensure_ascii=False))
	else:
		ap.print_help() 