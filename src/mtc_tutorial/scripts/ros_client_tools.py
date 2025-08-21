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
    # 新增：直接支持杯子位姿参数
    cup_x=None,
    cup_y=None,
    cup_z=None,
    cup_qx=None,
    cup_qy=None,
    cup_qz=None,
    cup_qw=None,
    set_cup_pose_first=True,  # 是否在执行前先设置杯子位姿
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
    新增支持 cup_x/y/z, cup_qx/qy/qz/qw 直接设置杯子位姿。
    返回字典包含 ok/status/success/error/duration_sec/params。
    """
    created = _ensure_rclpy_inited()
    node = rclpy.create_node('pour_client')
    
    try:
        # 确保环境变量设置正确（用于导入本地构建的包）
        import os
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        install_path = os.path.join(workspace_root, "install")
        if os.path.exists(install_path):
            # 设置Python路径
            import sys
            python_path = os.path.join(install_path, "mtc_interface", "local", "lib", "python3.10", "dist-packages")
            if os.path.exists(python_path) and python_path not in sys.path:
                sys.path.insert(0, python_path)
            
            # 设置ROS环境变量
            if "AMENT_PREFIX_PATH" not in os.environ:
                os.environ["AMENT_PREFIX_PATH"] = install_path
            elif install_path not in os.environ["AMENT_PREFIX_PATH"]:
                os.environ["AMENT_PREFIX_PATH"] = install_path + ":" + os.environ["AMENT_PREFIX_PATH"]
        
        # 导入所需的消息类型
        from rclpy.action import ActionClient
        import mtc_interface.action
        import time
        
        # 验证参数
        p = _validate_pour_params(params)
        
        # 重构：检查杯子对象是否存在于规划场景中
        object_check = check_object_exists(object_id="object", timeout_sec=3.0)
        if not object_check["ok"] or not object_check["object_exists"]:
            node.get_logger().error("Cup object 'object' not found in planning scene!")
            return {
                "ok": False, 
                "status": "missing_object", 
                "error": "Cup object not found in planning scene. Please run setup_planning_scene first.",
                "params": p,
                "object_check": object_check
            }
        
        node.get_logger().info("Cup object found in planning scene, proceeding with pour task")
        
        # 可选功能：如果提供了杯子位姿参数，先更新杯子位姿（而非通过参数设置）
        cup_pose_updated = False
        if p.get('update_cup_pose_first', False):
            cup_x = p.get('cup_x')
            cup_y = p.get('cup_y') 
            cup_z = p.get('cup_z')
            cup_qx = p.get('cup_qx')
            cup_qy = p.get('cup_qy')
            cup_qz = p.get('cup_qz')
            cup_qw = p.get('cup_qw')
            
            if cup_x is not None and cup_y is not None and cup_z is not None:
                node.get_logger().info(f"Updating cup pose: ({cup_x}, {cup_y}, {cup_z})")
                
                update_result = update_cup_pose(
                    cup_x=cup_x, cup_y=cup_y, cup_z=cup_z,
                    cup_qx=cup_qx, cup_qy=cup_qy, cup_qz=cup_qz, cup_qw=cup_qw,
                    timeout_sec=3.0
                )
                
                if update_result["ok"]:
                    node.get_logger().info("Cup pose updated successfully")
                    cup_pose_updated = True
                else:
                    node.get_logger().warn(f"Failed to update cup pose: {update_result.get('error', 'Unknown error')}")
                    # 继续执行，使用现有位姿
                
                # 等待一下确保位姿更新生效
                time.sleep(0.3)
        
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
            f"approach[{goal.approach_min}, {goal.approach_max}], plan_only={goal.plan_only}, "
            f"cup_pose_updated={cup_pose_updated}")
        
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
            "cup_pose_updated": cup_pose_updated,
            "object_check": object_check,
        }
        
    except Exception as e:
        return {"ok": False, "status": "error", "error": f"Pour execution failed: {str(e)}"}
    
    finally:
        node.destroy_node()
        _shutdown_rclpy_if(created)


# =============== 对外工具：场景构建和管理 ===============

def setup_planning_scene(include_cup: bool = True, 
                         cup_x: float = 0.0, cup_y: float = -0.4, cup_z: float = 0.13,
                         cup_qx: Optional[float] = None, cup_qy: Optional[float] = None,
                         cup_qz: Optional[float] = None, cup_qw: Optional[float] = 1.0,
                         cup_height: float = 0.1, cup_radius: float = 0.02,
                         timeout_sec: float = 10.0) -> Dict[str, Any]:
    """设置完整的规划场景，包括桌面和杯子对象。
    
    Args:
        include_cup: 是否包含杯子对象
        cup_x, cup_y, cup_z: 杯子位置坐标
        cup_qx, cup_qy, cup_qz, cup_qw: 杯子方向四元数（可选）
        cup_height: 杯子高度（米）
        cup_radius: 杯子半径（米）
        timeout_sec: 超时时间
        
    Returns:
        包含场景设置结果的字典
    """
    created = _ensure_rclpy_inited()
    import rclpy
    node = rclpy.create_node('planning_scene_setup')
    
    try:
        # 导入必要的消息类型
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        import time
        
        # 使用ApplyPlanningScene服务，更好地模拟原始的PlanningSceneInterface
        apply_scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        # 等待服务可用
        if not apply_scene_client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().warn("ApplyPlanningScene service not available, falling back to topic publishing")
            # 备用方案：使用topic发布
            collision_object_pub = node.create_publisher(CollisionObject, '/collision_object', 10)
            time.sleep(1.0)
            use_service = False
        else:
            use_service = True
        
        collision_objects = []
        
        # 1. 添加台面碰撞对象 (OptoSigma光学面包板)
        table_surface = CollisionObject()
        table_surface.header.stamp = node.get_clock().now().to_msg()
        table_surface.header.frame_id = "link_base"
        table_surface.id = "table_surface"
        table_surface.primitives.append(SolidPrimitive())
        table_surface.primitives[0].type = SolidPrimitive.BOX
        table_surface.primitives[0].dimensions = [1.0, 1.5, 0.01]  # 长x宽x高 (米)
        
        table_pose = Pose()
        table_pose.position.x = 0.0   # 台面中心相对于机械臂基座
        table_pose.position.y = -0.25
        table_pose.position.z = -0.01  # 台面高度，低于机械臂基座
        table_pose.orientation.w = 1.0
        table_surface.primitive_poses.append(table_pose)
        table_surface.operation = CollisionObject.ADD
        collision_objects.append(table_surface)
        
        # 2. 添加台面支撑结构
        table_base = CollisionObject()
        table_base.header.stamp = node.get_clock().now().to_msg()
        table_base.header.frame_id = "link_base"
        table_base.id = "table_base"
        table_base.primitives.append(SolidPrimitive())
        table_base.primitives[0].type = SolidPrimitive.BOX
        table_base.primitives[0].dimensions = [0.7, 0.7, 0.05]  # 长x宽x高 (米)
        
        base_pose = Pose()
        base_pose.position.x = 0.0
        base_pose.position.y = -0.55
        base_pose.position.z = 0.025  # 底座位置，低于台面
        base_pose.orientation.w = 1.0
        table_base.primitive_poses.append(base_pose)
        table_base.operation = CollisionObject.ADD
        collision_objects.append(table_base)
        
        # 3. 可选：添加杯子对象
        if include_cup:
            cup_object = CollisionObject()
            cup_object.header.stamp = node.get_clock().now().to_msg()
            cup_object.header.frame_id = "link_base"
            cup_object.id = "object"  # 保持与现有代码兼容的ID
            cup_object.primitives.append(SolidPrimitive())
            cup_object.primitives[0].type = SolidPrimitive.CYLINDER
            cup_object.primitives[0].dimensions = [cup_height, cup_radius]  # 高度，半径
            
            cup_pose = Pose()
            cup_pose.position.x = cup_x
            cup_pose.position.y = cup_y
            cup_pose.position.z = cup_z
            
            # 设置方向
            if all(q is not None for q in [cup_qx, cup_qy, cup_qz, cup_qw]):
                cup_pose.orientation.x = cup_qx
                cup_pose.orientation.y = cup_qy
                cup_pose.orientation.z = cup_qz
                cup_pose.orientation.w = cup_qw
            else:
                cup_pose.orientation.w = 1.0  # 默认方向
            
            cup_object.primitive_poses.append(cup_pose)
            cup_object.operation = CollisionObject.ADD
            collision_objects.append(cup_object)
        
        # 应用碰撞对象 - 模拟原始的 psi.applyCollisionObjects(collision_objects)
        if use_service:
            # 使用服务方式，更接近原始实现
            planning_scene_msg = PlanningScene()
            planning_scene_msg.world.collision_objects = collision_objects
            planning_scene_msg.is_diff = True
            
            request = ApplyPlanningScene.Request()
            request.scene = planning_scene_msg
            
            future = apply_scene_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
            
            if future.result() is not None and future.result().success:
                node.get_logger().info("Applied planning scene via service (like original PlanningSceneInterface)")
                method = "service"
            else:
                node.get_logger().warn("Service call failed, falling back to topic publishing")
                use_service = False
        
        if not use_service:
            # 备用方案：topic发布
            if 'collision_object_pub' not in locals():
                collision_object_pub = node.create_publisher(CollisionObject, '/collision_object', 10)
                time.sleep(0.5)
            for obj in collision_objects:
                collision_object_pub.publish(obj)
                node.get_logger().info(f"Published collision object: {obj.id}")
                time.sleep(0.1)
            method = "topic"
        
        # 等待对象添加完成
        time.sleep(1.0)
        
        # 验证对象是否成功添加
        added_objects = [obj.id for obj in collision_objects]
        
        node.get_logger().info("Planning scene setup completed!")
        node.get_logger().info(f"Added objects using {method}: {added_objects}")
        if include_cup:
            node.get_logger().info(f"Cup position: x={cup_x:.3f}, y={cup_y:.3f}, z={cup_z:.3f}")
            node.get_logger().info(f"Cup dimensions: height={cup_height:.3f}m, radius={cup_radius:.3f}m")
        
        return {
            "ok": True,
            "status": "success",
            "objects_added": added_objects,
            "table_objects": ["table_surface", "table_base"],
            "cup_included": include_cup,
            "cup_position": {"x": cup_x, "y": cup_y, "z": cup_z} if include_cup else None,
            "cup_orientation": {"x": cup_qx, "y": cup_qy, "z": cup_qz, "w": cup_qw} if include_cup and all(q is not None for q in [cup_qx, cup_qy, cup_qz, cup_qw]) else None,
            "method": method,
            "msg": f"Successfully added {len(added_objects)} objects to planning scene using {method}"
        }
        
    except Exception as e:
        node.get_logger().error(f"Failed to setup planning scene: {str(e)}")
        return {
            "ok": False,
            "status": "error",
            "error": str(e),
            "msg": f"Planning scene setup failed: {str(e)}"
        }
        
    finally:
        node.destroy_node()
        _shutdown_rclpy_if(created)


def check_object_exists(object_id: str = "object", timeout_sec: float = 5.0) -> Dict[str, Any]:
    """检查指定的碰撞对象是否存在于规划场景中。
    
    Args:
        object_id: 要检查的对象ID
        timeout_sec: 超时时间
        
    Returns:
        包含对象存在状态的字典
    """
    created = _ensure_rclpy_inited()
    node = rclpy.create_node('object_checker')
    
    try:
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        import time
        
        # 使用GetPlanningScene服务查询对象
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            node.get_logger().warn("GetPlanningScene service not available, assuming object doesn't exist")
            return {
                "ok": True,
                "status": "service_unavailable",
                "object_exists": False,
                "object_id": object_id,
                "known_objects": [],
                "msg": f"Cannot verify object '{object_id}' - planning scene service unavailable"
            }
        
        # 构建请求
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        
        # 发送请求
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        
        if future.result() is not None:
            response = future.result()
            world_objects = response.scene.world.collision_objects
            known_objects = [obj.id for obj in world_objects]
            object_exists = object_id in known_objects
            
            node.get_logger().info(f"Checking for object '{object_id}': {'Found' if object_exists else 'Not found'}")
            node.get_logger().info(f"Known objects in scene: {known_objects}")
            
            return {
                "ok": True,
                "status": "success",
                "object_exists": object_exists,
                "object_id": object_id,
                "known_objects": known_objects,
                "msg": f"Object '{object_id}' {'exists' if object_exists else 'not found'} in planning scene"
            }
        else:
            node.get_logger().error("Failed to get response from planning scene service")
            return {
                "ok": False,
                "status": "service_timeout",
                "object_exists": False,
                "object_id": object_id,
                "known_objects": [],
                "error": "Service call timeout",
                "msg": f"Failed to check object existence: service timeout"
            }
        
    except Exception as e:
        node.get_logger().error(f"Failed to check object existence: {str(e)}")
        return {
            "ok": False,
            "status": "error",
            "object_exists": False,
            "error": str(e),
            "msg": f"Failed to check object existence: {str(e)}"
        }
        
    finally:
        node.destroy_node()
        _shutdown_rclpy_if(created)


def update_cup_pose(cup_x: float, cup_y: float, cup_z: float,
                    cup_qx: Optional[float] = None, cup_qy: Optional[float] = None,
                    cup_qz: Optional[float] = None, cup_qw: Optional[float] = 1.0,
                    cup_height: float = 0.1, cup_radius: float = 0.02,
                    timeout_sec: float = 5.0) -> Dict[str, Any]:
    """更新杯子对象的位姿。
    
    Args:
        cup_x, cup_y, cup_z: 新的杯子位置坐标
        cup_qx, cup_qy, cup_qz, cup_qw: 新的杯子方向四元数（可选）
        cup_height: 杯子高度（米）
        cup_radius: 杯子半径（米）
        timeout_sec: 超时时间
        
    Returns:
        包含更新结果的字典
    """
    created = _ensure_rclpy_inited()
    import rclpy
    node = rclpy.create_node('cup_pose_updater')
    
    try:
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        import time
        
        # 先检查杯子对象是否存在
        check_result = check_object_exists("object", timeout_sec=timeout_sec/2)
        if not check_result["ok"] or not check_result["object_exists"]:
            return {
                "ok": False,
                "status": "not_found",
                "msg": "Cup object 'object' not found in planning scene. Please setup planning scene first."
            }
        
        # 使用ApplyPlanningScene服务来更新对象，与setup_planning_scene保持一致
        apply_scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        if not apply_scene_client.wait_for_service(timeout_sec=timeout_sec/2):
            # 备用方案：使用topic发布
            collision_object_pub = node.create_publisher(CollisionObject, '/collision_object', 10)
            time.sleep(0.5)
            use_service = False
        else:
            use_service = True
        
        # 创建新的杯子对象来替换现有的
        cup_object = CollisionObject()
        cup_object.header.stamp = node.get_clock().now().to_msg()
        cup_object.header.frame_id = "link_base"
        cup_object.id = "object"
        cup_object.primitives.append(SolidPrimitive())
        cup_object.primitives[0].type = SolidPrimitive.CYLINDER
        cup_object.primitives[0].dimensions = [cup_height, cup_radius]
        
        cup_pose = Pose()
        cup_pose.position.x = cup_x
        cup_pose.position.y = cup_y
        cup_pose.position.z = cup_z
        
        # 设置方向
        if all(q is not None for q in [cup_qx, cup_qy, cup_qz, cup_qw]):
            cup_pose.orientation.x = cup_qx
            cup_pose.orientation.y = cup_qy
            cup_pose.orientation.z = cup_qz
            cup_pose.orientation.w = cup_qw
        else:
            cup_pose.orientation.w = 1.0
        
        cup_object.primitive_poses.append(cup_pose)
        cup_object.operation = CollisionObject.ADD  # 使用ADD操作替换对象（MoveIt会自动覆盖同名对象）
        
        # 更新对象 - 使用先删除再添加的可靠方法
        if use_service:
            # 方式1：先删除现有对象
            remove_object = CollisionObject()
            remove_object.header.stamp = node.get_clock().now().to_msg()
            remove_object.header.frame_id = "link_base"
            remove_object.id = "object"
            remove_object.operation = CollisionObject.REMOVE
            
            planning_scene_remove = PlanningScene()
            planning_scene_remove.world.collision_objects = [remove_object]
            planning_scene_remove.is_diff = True
            
            request_remove = ApplyPlanningScene.Request()
            request_remove.scene = planning_scene_remove
            
            future_remove = apply_scene_client.call_async(request_remove)
            rclpy.spin_until_future_complete(node, future_remove, timeout_sec=timeout_sec/4)
            
            # 短暂等待删除完成
            time.sleep(0.2)
            
            # 方式2：重新添加对象到新位置
            planning_scene_add = PlanningScene()
            planning_scene_add.world.collision_objects = [cup_object]
            planning_scene_add.is_diff = True
            
            request_add = ApplyPlanningScene.Request()
            request_add.scene = planning_scene_add
            
            future_add = apply_scene_client.call_async(request_add)
            rclpy.spin_until_future_complete(node, future_add, timeout_sec=timeout_sec/2)
            
            if future_add.result() is not None and future_add.result().success:
                method = "service (remove+add)"
                node.get_logger().info("Successfully removed and re-added cup object at new position")
            else:
                use_service = False
        
        if not use_service:
            # 备用方案：topic发布（先删除再添加）
            if 'collision_object_pub' not in locals():
                collision_object_pub = node.create_publisher(CollisionObject, '/collision_object', 10)
                time.sleep(0.5)
            
            # 删除旧对象
            remove_object = CollisionObject()
            remove_object.header.stamp = node.get_clock().now().to_msg()
            remove_object.header.frame_id = "link_base"
            remove_object.id = "object"
            remove_object.operation = CollisionObject.REMOVE
            collision_object_pub.publish(remove_object)
            time.sleep(0.2)
            
            # 添加新对象
            collision_object_pub.publish(cup_object)
            method = "topic (remove+add)"
        
        time.sleep(0.5)  # 等待更新生效
        
        node.get_logger().info(f"Updated cup pose using {method} to: x={cup_x:.3f}, y={cup_y:.3f}, z={cup_z:.3f}")
        
        return {
            "ok": True,
            "status": "success",
            "cup_position": {"x": cup_x, "y": cup_y, "z": cup_z},
            "cup_orientation": {"x": cup_qx, "y": cup_qy, "z": cup_qz, "w": cup_qw} if all(q is not None for q in [cup_qx, cup_qy, cup_qz, cup_qw]) else None,
            "method": method,
            "msg": "Cup pose updated successfully"
        }
        
    except Exception as e:
        node.get_logger().error(f"Failed to update cup pose: {str(e)}")
        return {
            "ok": False,
            "status": "error",
            "error": str(e),
            "msg": f"Failed to update cup pose: {str(e)}"
        }
        
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


# =============== 模块化任务执行工具 ===============

def _execute_real_mtc_task(task_type: str, params_dict: Dict[str, Any], task_name: str, 
                          action_name: str = 'execute_modular_task', timeout_sec: float = 180.0) -> Dict[str, Any]:
    """使用真实的模块化任务服务器执行MTC任务
    
    Args:
        task_type: 任务类型 ("pick", "pour", "place", "return")
        params_dict: 任务参数字典
        task_name: 任务名称（用于日志）
        action_name: Action服务器名称
        timeout_sec: 超时时间
        
    Returns:
        执行结果字典
    """
    created = _ensure_rclpy_inited()
    node = rclpy.create_node('real_mtc_client')
    
    try:
        import time
        
        node.get_logger().info(f"🚀 开始执行{task_name}任务 (类型: {task_type}) - 使用真实模块化任务服务器")
        
        # 确保环境变量设置正确
        import os
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        install_path = os.path.join(workspace_root, "install")
        if os.path.exists(install_path):
            import sys
            python_path = os.path.join(install_path, "mtc_interface", "local", "lib", "python3.10", "dist-packages")
            if os.path.exists(python_path) and python_path not in sys.path:
                sys.path.insert(0, python_path)
            
            if "AMENT_PREFIX_PATH" not in os.environ:
                os.environ["AMENT_PREFIX_PATH"] = install_path
            elif install_path not in os.environ["AMENT_PREFIX_PATH"]:
                os.environ["AMENT_PREFIX_PATH"] = install_path + ":" + os.environ["AMENT_PREFIX_PATH"]
        
        # 导入Action客户端
        from rclpy.action import ActionClient
        import mtc_interface.action
        
        # 创建Action客户端
        action_client = ActionClient(node, mtc_interface.action.ExecutePour, action_name)
        
        # 针对 pick 任务：在发送 goal 之前，将参数写入模块化服务器的参数表
        if task_type == "pick":
            server_node = '/modular_task_server'
            param_map: Dict[str, Any] = {}
            if 'safe_approach_height' in params_dict:
                param_map['pick.safe_approach_height'] = float(params_dict['safe_approach_height'])
            # 可选：后方约束
            if 'use_back_constraint' in params_dict:
                param_map['pick.use_back_constraint'] = bool(params_dict['use_back_constraint'])
            # 区域尺寸参数（如有）
            for k_src, k_dst in [
                ('back_region_center_y', 'pick.back_region_center_y'),
                ('back_region_size_x', 'pick.back_region_size_x'),
                ('back_region_size_y', 'pick.back_region_size_y'),
                ('back_region_size_z', 'pick.back_region_size_z'),
            ]:
                if k_src in params_dict:
                    param_map[k_dst] = float(params_dict[k_src])
            if param_map:
                node.get_logger().info(f"设置模块化服务器参数: {param_map}")
                _set_params(server_node, param_map, timeout_sec=2.0)
        
        # 等待服务器
        if not action_client.wait_for_server(timeout_sec=10.0):
            return {
                "ok": False,
                "status": "no_server",
                "error": f"模块化任务服务器不可用: {action_name}",
                "params": params_dict,
                "task_name": task_name
            }
        
        # 构建Goal (复用ExecutePour的消息格式，通过target_id指定任务类型)
        goal = mtc_interface.action.ExecutePour.Goal()
        goal.target_id = task_type  # 关键：通过target_id指定任务类型
        
        # 映射参数到ExecutePour格式
        goal.tilt_start_deg = float(params_dict.get("tilt_start_deg", 45.0))
        goal.tilt_end_deg = float(params_dict.get("tilt_end_deg", 120.0)) 
        goal.tilt_speed_deg_s = float(params_dict.get("tilt_speed_deg_s", 25.0))
        goal.pour_hold_sec = float(params_dict.get("pour_hold_sec", 2.0))
        goal.lift_height = float(params_dict.get("lift_height", 0.12))
        goal.approach_min = float(params_dict.get("approach_min", 0.05))
        goal.approach_max = float(params_dict.get("approach_max", 0.15))
        goal.plan_only = bool(params_dict.get("plan_only", False))
        
        node.get_logger().info(f"🎯 发送{task_name}任务到模块化服务器")
        node.get_logger().info(f"📋 任务类型: {task_type} (通过target_id传递)")
        node.get_logger().info(f"⚙️ 参数: approach=[{goal.approach_min:.2f}, {goal.approach_max:.2f}], lift={goal.lift_height:.2f}")
        
        # 反馈处理
        def feedback_callback(feedback_msg):
            fb = feedback_msg.feedback
            node.get_logger().info(f"[{fb.stage}] 进度: {fb.progress:.1%}")
        
        # 发送目标
        send_future = action_client.send_goal_async(goal, feedback_callback=feedback_callback)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)
        
        gh = send_future.result()
        if gh is None or not gh.accepted:
            return {
                "ok": False,
                "status": "rejected",
                "error": "任务被模块化服务器拒绝",
                "params": params_dict,
                "task_name": task_name
            }
        
        node.get_logger().info(f'✅ {task_name}任务已被模块化服务器接受，开始执行...')
        start_time = time.time()
        
        # 等待结果
        result_future = gh.get_result_async()
        while not result_future.done():
            if timeout_sec and time.time() - start_time > timeout_sec:
                node.get_logger().error(f"任务超时 {timeout_sec}s，取消任务")
                gh.cancel_goal_async()
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        
        if not result_future.done():
            return {
                "ok": False,
                "status": "timeout",
                "error": f"任务超时 {timeout_sec}s",
                "params": params_dict,
                "task_name": task_name
            }
        
        # 处理结果
        result_msg = result_future.result()
        result = result_msg.result
        status_code = result_msg.status
        
        status_map = {4: "succeeded", 5: "canceled", 6: "aborted"}
        status_str = status_map.get(status_code, "unknown")
        
        node.get_logger().info(f'🎉 {task_name}任务执行完成: {status_str}, 成功={result.success}, 耗时={result.duration_sec:.2f}s')
        
        success_note = ""
        if result.success:
            success_note = f"✅ 真实的模块化MTC任务执行成功！使用了{task_type}任务构建器"
        
        return {
            "ok": bool(result.success) and status_code == 4,
            "status": status_str,
            "success": bool(result.success),
            "duration_sec": float(result.duration_sec),
            "error": result.error_msg,
            "params": params_dict,
            "task_name": task_name,
            "plan_only": params_dict.get('plan_only', False),
            "note": success_note,
            "used_modular_server": True,
            "task_type": task_type
            }
        
    except Exception as e:
        node.get_logger().error(f"{task_name}任务执行异常: {str(e)}")
        return {
            "ok": False,
            "status": "error",
            "success": False,
            "error": f"客户端执行异常: {str(e)}",
            "params": params_dict,
            "task_name": task_name
        }
        
    finally:
        node.destroy_node()
        _shutdown_rclpy_if(created)


def pick_container(source_pose: Dict[str, float], 
                   grasp_hint: Optional[Dict[str, float]] = None,
                   object_id: str = "object", 
                   plan_only: bool = False,
                   timeout_sec: float = 180.0) -> Dict[str, Any]:
    """抓取容器
    
    Args:
        source_pose: 源位姿 {"x": float, "y": float, "z": float, 可选: "qx", "qy", "qz", "qw"}
        grasp_hint: 抓取提示 {"approach_min": float, "approach_max": float, "lift_height": float,
                           可选: "safe_approach_height": float,
                           可选: "use_back_constraint": bool,
                           可选: "back_region_center_y"/"back_region_size_x"/"back_region_size_y"/"back_region_size_z"}
        object_id: 目标对象ID
        plan_only: 仅规划不执行
        timeout_sec: 超时时间
        
    Returns:
        执行结果字典
    """
    params = {
        "source_x": source_pose.get("x", 0.0),
        "source_y": source_pose.get("y", -0.4),
        "source_z": source_pose.get("z", 0.13),
        "source_qx": source_pose.get("qx", 0.0),
        "source_qy": source_pose.get("qy", 0.0), 
        "source_qz": source_pose.get("qz", 0.0),
        "source_qw": source_pose.get("qw", 1.0),
        "object_id": object_id,
        "plan_only": plan_only
    }
    
    if grasp_hint:
        params.update({
            "approach_min": grasp_hint.get("approach_min", 0.05),
            "approach_max": grasp_hint.get("approach_max", 0.15),
            "lift_height": grasp_hint.get("lift_height", 0.12)
        })
        # 新增：安全高度与后方约束（如提供）
        if "safe_approach_height" in grasp_hint:
            params["safe_approach_height"] = grasp_hint["safe_approach_height"]
        for k in ("use_back_constraint", "back_region_center_y", "back_region_size_x", "back_region_size_y", "back_region_size_z"):
            if k in grasp_hint:
                params[k] = grasp_hint[k]
    else:
        params.update({
            "approach_min": 0.05,
            "approach_max": 0.15,
            "lift_height": 0.12
        })
    
    # 首先尝试使用真实的模块化任务服务器
    try:
        return _execute_real_mtc_task("pick", params, "抓取容器", 'execute_modular_task', timeout_sec)
    except Exception as e:
        # 如果模块化服务器不可用，回退到模拟版本并记录
        result = _execute_mtc_task("pick", params, "抓取容器", timeout_sec)
        result["fallback_reason"] = f"模块化服务器不可用: {str(e)}"
        return result


def pour_to_target(target_pose: Optional[Dict[str, float]] = None,
                   tilt_deg: Dict[str, float] = {"start": 45.0, "end": 120.0},
                   speed: float = 25.0,
                   stop_condition: Dict[str, float] = {"hold_time": 2.0},
                   move_distance: Optional[Dict[str, float]] = None,
                   plan_only: bool = False,
                   timeout_sec: float = 180.0) -> Dict[str, Any]:
    """执行倾倒动作
    
    Args:
        target_pose: 目标倾倒位置（可选，将使用移动相对距离）
        tilt_deg: 倾斜角度 {"start": float, "end": float}
        speed: 倾斜速度（度/秒）
        stop_condition: 停止条件 {"hold_time": float}
        move_distance: 移动距离 {"min": float, "max": float}
        plan_only: 仅规划不执行
        timeout_sec: 超时时间
        
    Returns:
        执行结果字典
    """
    params = {
        "tilt_start_deg": tilt_deg.get("start", 45.0),
        "tilt_end_deg": tilt_deg.get("end", 120.0),
        "tilt_speed_deg_s": speed,
        "pour_hold_sec": stop_condition.get("hold_time", 2.0),
        "plan_only": plan_only
    }
    
    if target_pose:
        params.update({
            "target_x": target_pose.get("x", 0.1),
            "target_y": target_pose.get("y", -0.5),
            "target_z": target_pose.get("z", 0.13)
        })
    
    if move_distance:
        params.update({
            "move_to_pour_min": move_distance.get("min", 0.08),
            "move_to_pour_max": move_distance.get("max", 0.15)
        })
    else:
        params.update({
            "move_to_pour_min": 0.08,
            "move_to_pour_max": 0.15
        })
    
    # 首先尝试使用真实的模块化任务服务器
    try:
        return _execute_real_mtc_task("pour", params, "倾倒液体", 'execute_modular_task', timeout_sec)
    except Exception as e:
        # 如果模块化服务器不可用，回退到原实现
        result = _execute_mtc_task("pour", params, "倾倒液体", timeout_sec)
        result["fallback_reason"] = f"模块化服务器不可用: {str(e)}"
        return result


def place_container(target_pose: Optional[Dict[str, float]] = None,
                    object_id: str = "object",
                    placement_params: Optional[Dict[str, float]] = None,
                    plan_only: bool = False,
                    timeout_sec: float = 180.0) -> Dict[str, Any]:
    """放置容器
    
    Args:
        target_pose: 目标放置位置（可选，使用默认位置）
        object_id: 目标对象ID
        placement_params: 放置参数 {"lower_min", "lower_max", "retreat_min", "retreat_max"}
        plan_only: 仅规划不执行
        timeout_sec: 超时时间
        
    Returns:
        执行结果字典
    """
    params = {
        "object_id": object_id,
        "plan_only": plan_only
    }
    
    if target_pose:
        params["target_pose"] = {
            "position": {
                "x": target_pose.get("x", 0.0),
                "y": target_pose.get("y", -0.45),
                "z": target_pose.get("z", 0.18)
            },
            "orientation": {
                "x": target_pose.get("qx", 0.0),
                "y": target_pose.get("qy", 0.0),
                "z": target_pose.get("qz", 0.0),
                "w": target_pose.get("qw", 1.0)
            }
        }
    
    if placement_params:
        params.update({
            "lower_min": placement_params.get("lower_min", 0.03),
            "lower_max": placement_params.get("lower_max", 0.2),
            "retreat_min": placement_params.get("retreat_min", 0.05),
            "retreat_max": placement_params.get("retreat_max", 0.1)
        })
    else:
        params.update({
            "lower_min": 0.03,
            "lower_max": 0.2,
            "retreat_min": 0.05,
            "retreat_max": 0.1
        })
    
    # 首先尝试使用真实的模块化任务服务器
    try:
        return _execute_real_mtc_task("place", params, "放置容器", 'execute_modular_task', timeout_sec)
    except Exception as e:
        # 如果模块化服务器不可用，回退到模拟版本
        result = _execute_mtc_task("place", params, "放置容器", timeout_sec)
        result["fallback_reason"] = f"模块化服务器不可用: {str(e)}"
        return result


def return_to_home(target_joints: Optional[Dict[str, float]] = None,
                   plan_only: bool = False,
                   timeout_sec: float = 60.0) -> Dict[str, Any]:
    """返回初始/安全位置
    
    Args:
        target_joints: 目标关节配置（可选，使用"home"配置）
        plan_only: 仅规划不执行
        timeout_sec: 超时时间
        
    Returns:
        执行结果字典
    """
    params = {
        "plan_only": plan_only,
        "timeout_sec": timeout_sec
    }
    
    if target_joints:
        params["target_joints"] = target_joints
    
    # 首先尝试使用真实的模块化任务服务器
    try:
        return _execute_real_mtc_task("return", params, "返回初始位置", 'execute_modular_task', timeout_sec)
    except Exception as e:
        # 如果模块化服务器不可用，回退到模拟版本
        result = _execute_mtc_task("return", params, "返回初始位置", timeout_sec)
        result["fallback_reason"] = f"模块化服务器不可用: {str(e)}"
        return result


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
                    timeout_sec: float = 30.0,
                    force_stop: bool = True) -> Dict[str, Any]:
    """立即取消当前action并将机器人移动到安全位姿
    
    Args:
        reason: 取消原因
        safe_pose_joints: 安全位姿的关节角度，如果为None使用默认
        timeout_sec: 操作超时时间
        force_stop: 是否强制停止（会尝试多种方式确保停止）
        
    Returns:
        Dict包含操作结果和详细信息
        
    注意：这个函数设计为即使在其他任务运行时也能工作，
         通过直接与ROS action服务器通信来强制取消任务。
    """
    created = _ensure_rclpy_inited()
    node = rclpy.create_node('abort_and_reset_client', start_parameter_services=False)
    result = {
        "success": False,
        "reason": reason,
        "actions_taken": [],
        "warnings": [],
        "robot_safe": False,
        "timestamp": None,
        "force_stop": force_stop
    }
    
    try:
        import time
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
        from geometry_msgs.msg import Pose
        
        result["timestamp"] = time.time()
        result["actions_taken"].append(f"强制中止请求: {reason}")
        
        # 步骤1: 强制取消正在进行的倾倒action
        cancel_success = False
        try:
            import mtc_interface.action  # type: ignore
            pour_client = ActionClient(node, mtc_interface.action.ExecutePour, '/execute_pour')
            
            if pour_client.wait_for_server(timeout_sec=3.0):
                # 尝试取消所有可能的goals
                # 注意：由于rclpy ActionClient的限制，我们无法直接获取和取消特定的goal handles
                # 但是我们可以通过发送一个立即失败的goal来打断正在执行的任务
                
                if force_stop:
                    # 强制停止策略：发送一个无效的goal来打断当前执行
                    try:
                        invalid_goal = mtc_interface.action.ExecutePour.Goal()
                        invalid_goal.plan_only = True  # 仅规划，不执行
                        invalid_goal.tilt_start_deg = 0.0
                        invalid_goal.tilt_end_deg = 0.0
                        invalid_goal.tilt_speed_deg_s = 1.0
                        invalid_goal.pour_hold_sec = 0.0
                        invalid_goal.lift_height = 0.0
                        invalid_goal.approach_min = 0.01
                        invalid_goal.approach_max = 0.02
                        invalid_goal.target_id = "ABORT_REQUEST"
                        
                        # 发送这个goal会导致服务器处理新的请求，可能中断当前任务
                        send_future = pour_client.send_goal_async(invalid_goal)
                        rclpy.spin_until_future_complete(node, send_future, timeout_sec=2.0)
                        
                        if send_future.done():
                            gh = send_future.result()
                            if gh:
                                # 立即取消这个goal
                                gh.cancel_goal_async()
                                result["actions_taken"].append("发送中断信号到pour action server")
                                cancel_success = True
                            
                    except Exception as e:
                        result["warnings"].append(f"强制中断策略失败: {str(e)}")
                
                result["actions_taken"].append("Pour action server可用，尝试了中断操作")
            else:
                result["warnings"].append("Pour action server不可用，可能没有正在运行的任务")
                cancel_success = True  # 如果服务器不存在，认为取消成功
                
        except Exception as e:
            result["warnings"].append(f"Pour action取消操作失败: {str(e)}")
        
        # 等待一下确保取消操作生效
        if cancel_success:
            time.sleep(0.5)
        
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
                                result["actions_taken"].append("机器人已移动到安全位姿")
                            else:
                                result["warnings"].append(f"MoveGroup失败: 错误代码 {move_result.result.error_code.val if move_result else '未知'}")
                        else:
                            result["warnings"].append("MoveGroup执行超时")
                    else:
                        result["warnings"].append("MoveGroup目标被拒绝")
                else:
                    result["warnings"].append("MoveGroup目标提交超时")
            else:
                result["warnings"].append("MoveGroup action server不可用")
                
        except Exception as e:
            result["warnings"].append(f"安全位姿移动失败: {str(e)}")
        
        # 步骤3: 重置夹爪到打开状态
        try:
            gripper_success = set_gripper_close_ratio(0.0, timeout_sec=3.0)  # 完全打开
            if gripper_success:
                result["actions_taken"].append("夹爪已打开")
            else:
                result["warnings"].append("夹爪打开失败")
        except Exception as e:
            result["warnings"].append(f"夹爪重置失败: {str(e)}")
        
        # 步骤4: 重置cup_pose参数为无效状态
        try:
            reset_params = {'cup_pose.valid': False}
            param_success = _set_params('/execute_pour_server', reset_params, timeout_sec=2.0)
            if param_success:
                result["actions_taken"].append("杯子位姿参数已重置")
            else:
                result["warnings"].append("杯子位姿参数重置失败")
        except Exception as e:
            result["warnings"].append(f"参数重置失败: {str(e)}")
        
        # 判断整体成功
        critical_actions = len([a for a in result["actions_taken"] if "机器人" in a or "夹爪" in a])
        result["success"] = critical_actions > 0 or len(result["actions_taken"]) > 2
        
        if result["success"]:
            result["actions_taken"].append("中止和重置操作成功完成")
        else:
            result["actions_taken"].append("中止操作完成，但有警告 - 建议手动检查")
            
    except Exception as e:
        result["success"] = False
        result["warnings"].append(f"中止操作过程中发生严重错误: {str(e)}")
    
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