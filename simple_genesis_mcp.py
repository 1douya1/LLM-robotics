"""
持久化Genesis MCP服务器
实现单场景、单窗口的持续交互模式
"""

from mcp.server.fastmcp import FastMCP
import logging
import sys
import json
import numpy as np
import os 
from genesis.utils.geom import quat_to_xyz, euler_to_quat, quat_to_R

# 导入Genesis
try:
    import genesis as gs
    GENESIS_AVAILABLE = True
    print("✅ Genesis可用")
except ImportError:
    GENESIS_AVAILABLE = False
    print("❌ Genesis不可用")

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 创建MCP服务器
mcp = FastMCP("Genesis Robot Simulator")

# ==================== 全局状态管理 ====================
class GenesisSession:
    """Genesis会话管理器 - 实现持久化状态"""
    
    def __init__(self):
        self.initialized = False
        self.scene = None
        self.robot = None
        self.entities = {}  # 记录所有实体
        self.cameras = {}   # 记录所有相机
        self.stats = {"scenes_created": 0, "actions_executed": 0, "entities_added": 0}
    
    def ensure_genesis_init(self):
        """确保Genesis只初始化一次"""
        if not self.initialized and GENESIS_AVAILABLE:
            try:
                gs.init(backend=gs.gpu)
                self.initialized = True
                logger.info("✅ Genesis初始化成功")
                return True
            except Exception as e:
                logger.error(f"❌ Genesis初始化失败: {e}")
                return False
        return self.initialized
    
    def create_scene(self, show_viewer=True):
        """创建或重置场景"""
        if not self.ensure_genesis_init():
            return False
        
        try:
            # 如果已有场景，先清理
            if self.scene is not None:
                logger.info("🔄 重置现有场景")
                # 可以选择重置或创建新场景
                
            # 创建新场景
            self.scene = gs.Scene(
                viewer_options=gs.options.ViewerOptions(
                    camera_pos=(3, -1, 1.5),
                    camera_lookat=(0.0, 0.0, 0.5),
                    camera_fov=30,
                    max_FPS=60,
                ),
                sim_options=gs.options.SimOptions(
                    dt=0.005,
                    substeps=10,
                ),
                sph_options=gs.options.SPHOptions(
                    lower_bound=(-1.0, -1.0, 0.0),
                    upper_bound=(1.0, 1.0, 2.0),
                    particle_size=0.005,
                ),
                vis_options=gs.options.VisOptions(
                    show_world_frame=True,
                    world_frame_size=1.0,
                    plane_reflection=True,
                    ambient_light=(0.1, 0.1, 0.1),
                    visualize_sph_boundary=False,
                ),
                renderer=gs.renderers.Rasterizer(),
                show_viewer=show_viewer,  # 控制是否显示窗口
            )
            
            # 添加基础地面
            plane = self.scene.add_entity(gs.morphs.Plane())
            self.entities['ground_plane'] = plane
            
            # 添加机器人
            robot = self.scene.add_entity(
                gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml')
            )
            self.entities['franka_robot'] = robot
            self.robot = robot
            
            # ⚠️ 重要：暂时不构建场景，等待添加更多实体
            self.built = False
            
            self.stats["scenes_created"] += 1
            logger.info("🎬 Genesis场景创建成功（未构建，等待添加实体）")
            return True
            
        except Exception as e:
            logger.error(f"场景创建失败: {e}")
            return False
    
    def build_scene_if_needed(self):
        """构建场景（只能调用一次）"""
        if self.scene is None:
            return False
            
        if not hasattr(self, 'built') or not self.built:
            try:
                self.scene.build()
                self.built = True
                
                # 设置机器人控制参数（只在构建后设置）
                if self.robot:
                    self.robot.set_dofs_kp(np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 300, 300]))
                    self.robot.set_dofs_kv(np.array([450, 450, 350, 350, 200, 200, 200, 30, 30]))
                    self.robot.set_dofs_force_range(
                        np.array([-87, -87, -87, -87, -12, -12, -12, -200, -200]),
                        np.array([87, 87, 87, 87, 12, 12, 12, 200, 200])
                    )
                
                logger.info("🔧 Genesis场景已构建")
                return True
            except Exception as e:
                logger.error(f"❌ 场景构建失败: {e}")
                return False
        else:
            logger.info("ℹ️ 场景已经构建过了")
            return True
    
    # 信息函数调试
    def get_object_grasp_info(self, obj_name='cup', grasp_height_ratio=0.5):
        """
        获取物体的抓取信息
        
        Args:
            obj_name: 实体名称（在self.entities中的键）
            grasp_height_ratio: 抓取高度比例（0-1），0.5表示抓取物体中部
        
        Returns:
            dict: 包含抓取位置、朝向、物体尺寸等信息，失败返回None
        """
        if obj_name not in self.entities:
            logger.error(f"物体 '{obj_name}' 不存在于场景中")
            return None
            
        try:
            obj_entity = self.entities[obj_name]
            
            # 获取物体位置和朝向（转换为numpy数组）
            obj_pos = obj_entity.get_pos()
            if hasattr(obj_pos, 'cpu'):  # 如果是tensor
                obj_pos = obj_pos.cpu().numpy()
            if len(obj_pos.shape) > 1:  # 如果是批量环境
                obj_pos = obj_pos[0]
            
            obj_quat = obj_entity.get_quat()
            if hasattr(obj_quat, 'cpu'):  # 如果是tensor
                obj_quat = obj_quat.cpu().numpy()
            if len(obj_quat.shape) > 1:  # 如果是批量环境
                obj_quat = obj_quat[0]
            
            # 获取物体的AABB包围盒来估算尺寸
            aabb = obj_entity.get_AABB()
            if hasattr(aabb, 'cpu'):
                aabb = aabb.cpu().numpy()
            if len(aabb.shape) > 2:
                aabb = aabb[0]
            
            # 计算物体尺寸
            obj_size = aabb[1] - aabb[0]  # [width, depth, height]
            obj_height = obj_size[2]
            obj_width = obj_size[0]
            obj_depth = obj_size[1]
            
            # 计算抓取高度
            grasp_height = aabb[0][2] + obj_height * grasp_height_ratio
            
            # 计算抓取位置（考虑物体中心）
            grasp_pos = np.array([
                obj_pos[0],
                obj_pos[1], 
                grasp_height
            ])
            
            grasp_info = {
                'position': grasp_pos,
                'quaternion': obj_quat,
                'size': obj_size,
                'width': obj_width,
                'depth': obj_depth,
                'height': obj_height,
                'aabb': aabb
            }
            
            logger.info(f"✅ 获取到 '{obj_name}' 的抓取信息")
            logger.debug(f"  位置: {grasp_pos}")
            logger.debug(f"  尺寸: W={obj_width:.3f}, D={obj_depth:.3f}, H={obj_height:.3f}")
            
            return grasp_info
            
        except Exception as e:
            logger.error(f"获取物体抓取信息失败: {e}")
            return None
    
    def calculate_grasp_approach(self, grasp_info, approach_distance=0.15, 
                               grasp_margin=1.2, finger_length=0.08, 
                               vertical_offset=0.02):
        """
        计算抓取接近点和手指位置
        
        Args:
            grasp_info: get_object_grasp_info返回的信息 
            approach_distance: 接近距离
            grasp_margin: 抓取余量系数（1.2表示手指张开比物体宽20%）
            finger_length: 手指长度（从hand到指尖）
            vertical_offset: 垂直偏移量，避免手掌碰到物体顶部
        
        Returns:
            dict: 包含预抓取位置、手指位置等
        """
        if grasp_info is None:
            return None
            
        try:
            grasp_pos = grasp_info['position']
            obj_width = grasp_info['width']
            obj_depth = grasp_info['depth']
            
            # 修正抓取位置：考虑手指长度和垂直偏移
            corrected_grasp_pos = grasp_pos.copy()
            corrected_grasp_pos[2] += vertical_offset  # 向上偏移，避免碰撞
            
            # 预抓取位置（在物体上方）
            pre_grasp_pos = corrected_grasp_pos.copy()
            pre_grasp_pos[2] += approach_distance
            
            # 计算手指张开宽度
            finger_spread = obj_width * grasp_margin
            
            # 横向偏移
            lateral_offset = 0.11  # 固定偏移11cm，确保手掌不会碰到物体
            
            # 计算实际的手掌位置
            hand_pos = corrected_grasp_pos.copy()
            hand_pos[1] = grasp_pos[1] - lateral_offset  # 在Y方向上后退，给手指留空间
            
            grasp_plan = {
                'pre_grasp_pos': pre_grasp_pos,
                'grasp_pos': hand_pos,  # 使用修正后的手掌位置
                'original_grasp_pos': grasp_pos,  # 保留原始抓取点
                'finger_spread': finger_spread,
                'grasp_quat': gs.euler_to_quat((-np.pi/2, 0, np.pi/2)),  # 垂直向下抓取
                'lateral_offset': lateral_offset,
                'vertical_offset': vertical_offset
            }
            
            logger.info("✅ 计算抓取策略成功")
            logger.debug(f"  预抓取位置: {pre_grasp_pos}")
            logger.debug(f"  手掌位置: {hand_pos}")
            logger.debug(f"  手指张开: {finger_spread:.3f}")
            
            return grasp_plan
            
        except Exception as e:
            logger.error(f"计算抓取策略失败: {e}")
            return None
    
    
    
    def step_simulation(self, steps=1):
        """运行仿真步骤"""
        if self.scene is None:
            return False
        
        try:
            for _ in range(steps):
                self.scene.step()
            return True
        except Exception as e:
            logger.error(f"❌ 仿真步骤失败: {e}")
            return False
    
    def reset_session(self):
        """重置整个会话"""
        try:
            # 如果有场景，尝试关闭viewer
            if self.scene is not None:
                try:
                    # 尝试关闭viewer（如果支持）
                    if hasattr(self.scene, 'viewer') and self.scene.viewer is not None:
                        self.scene.viewer.close()
                        logger.info("🔌 Genesis窗口已关闭")
                except Exception as e:
                    logger.warning(f"关闭窗口失败（这是正常的）: {e}")
            
            if GENESIS_AVAILABLE and self.initialized:
                # 不调用gs.reset_world()，因为可能导致问题
                # 直接清理Python对象引用
                pass
            
            self.scene = None
            self.robot = None
            self.entities = {}
            self.cameras = {}
            self.built = False  # 重置构建状态
            logger.info("🔄 Genesis会话已重置")
            return True
        except Exception as e:
            logger.error(f"❌ 重置失败: {e}")
            return False

# 全局会话实例
session = GenesisSession()

# ==================== MCP工具定义 ====================

@mcp.tool()
def create_simulation_scene(show_viewer: bool = True) -> dict:
    """创建Genesis机器人仿真场景（持久化模式）
    
    Args:
        show_viewer: 是否显示3D窗口，默认True
    """
    if not GENESIS_AVAILABLE:
        return {"error": "Genesis未安装，请在包含Genesis的虚拟环境中运行"}
    
    success = session.create_scene(show_viewer=show_viewer)
    
    if success:
        return {
            "success": True,
            "message": f"🎬 持久化仿真场景已创建（窗口{'开启' if show_viewer else '关闭'}）",
            "entities": list(session.entities.keys()),
            "persistent": True,
            "session_stats": session.stats
        }
    else:
        return {"error": "场景创建失败，请查看日志"}

@mcp.tool()
def add_cup_with_liquid() -> dict:
    """向现有场景添加装有液体的杯子"""
    if session.scene is None:
        return {"error": "请先调用 create_simulation_scene() 创建场景"}
    
    # 检查是否已经构建
    if hasattr(session, 'built') and session.built:
        return {"error": "场景已构建，无法添加新实体。请先调用 reset_simulation() 重置"}
    
    try:
        # 杯子配置（完全按照grasp-optimization.py）
        cup_pos = (0.55, 0.55, 0.01)
        cup_scale = 0.03
        cup_euler = (-np.pi/2, 0, 0)
        
        # 创建杯子实体 - 使用原始的Mesh配置
        cup_entity = session.scene.add_entity(
            morph=gs.morphs.Mesh(
                file='/home/dd/Genesis/Genesis/project/object/newscene.obj',  # 使用绝对路径
                scale=cup_scale,
                pos=cup_pos,
                euler=cup_euler,
                convexify=False,
                decompose_nonconvex=True,
            ),
            material=gs.materials.Rigid(
                rho=800.0,
                friction=0.8,
            ),
            surface=gs.surfaces.Default(
                color=(0.9, 0.9, 0.9),
                vis_mode='visual',
            ),
        )
        
        # 创建液体实体 - 使用原始的SPH配置
        liquid_pos = (0.55, 0.55, 0.05)
        liquid_size = (0.03, 0.03, 0.06)
        
        liquid_entity = session.scene.add_entity(
            material=gs.materials.SPH.Liquid(
                sampler='pbs',
            ),
            morph=gs.morphs.Box(
                pos=liquid_pos,
                size=liquid_size,
            ),
            surface=gs.surfaces.Default(
                color=(0.4, 0.8, 1.0),
                vis_mode='particle',
            ),
        )
        
        # 现在构建场景（只能调用一次）
        if not session.build_scene_if_needed():
            return {"error": "场景构建失败"}
        
        # 修正杯子方向（在构建后）
        cup_entity.set_quat(gs.euler_to_quat((np.pi/2, 0, 0)))
        
        # 记录实体
        session.entities['cup'] = cup_entity
        session.entities['liquid'] = liquid_entity
        session.stats["entities_added"] += 2
        
        logger.info("➕ 杯子和液体已添加（使用原始配置）")
        
        return {
            "success": True,
            "message": "➕ 杯子和液体已添加到现有场景(使用mesh文件和SPH液体)",
            "cup_position": cup_pos,
            "liquid_position": liquid_pos,
            "cup_scale": cup_scale,
            "liquid_size": liquid_size,
            "scene_built": True,
            "total_entities": len(session.entities)
        }
        
    except Exception as e:
        logger.error(f"❌ 添加杯子失败: {e}")
        return {"error": f"杯子添加失败: {str(e)}"}


@mcp.tool()
def add_test_sphere(
    sphere_radius: float = 0.015,
    sphere_position: tuple = (0.55, 0.55, 0.08),
    sphere_color: tuple = (1.0, 0.2, 0.2),
    sphere_density: float = 1000.0,
    sphere_friction: float = 0.5
) -> dict:
    """向现有场景添加测试球体（代替液体用于倾倒测试）
    
    Args:
        sphere_radius: 球体半径，默认0.015m（1.5cm）
        sphere_position: 球体初始位置 (x, y, z)，默认在杯子内部稍高位置
        sphere_color: 球体颜色 (r, g, b)，默认红色
        sphere_density: 球体密度 (kg/m³)，默认1000（水的密度）
        sphere_friction: 球体摩擦系数，默认0.5
    """
    if session.scene is None:
        return {"error": "请先调用 create_simulation_scene() 创建场景"}
    
    # 检查是否已经构建
    if hasattr(session, 'built') and session.built:
        return {"error": "场景已构建，无法添加新实体。请先调用 reset_simulation() 重置"}
    
    try:
        # 创建球体实体
        sphere_entity = session.scene.add_entity(
            morph=gs.morphs.Sphere(
                radius=sphere_radius,
                pos=sphere_position,
            ),
            material=gs.materials.Rigid(
                rho=sphere_density,
                friction=sphere_friction,
            ),
            surface=gs.surfaces.Default(
                color=sphere_color,
                vis_mode='visual',
            ),
        )
        
        # 记录实体
        session.entities['test_sphere'] = sphere_entity
        session.stats["entities_added"] += 1
        
        logger.info(f"➕ 测试球体已添加到位置 {sphere_position}")
        
        return {
            "success": True,
            "message": f"➕ 测试球体已添加（半径: {sphere_radius}m, 位置: {sphere_position}）",
            "sphere_details": {
                "radius": sphere_radius,
                "position": sphere_position,
                "color": sphere_color,
                "density": sphere_density,
                "friction": sphere_friction,
                "mass_estimate": 4/3 * 3.14159 * sphere_radius**3 * sphere_density
            },
            "usage_tip": "球体将在场景构建后受重力影响落入杯子中",
            "total_entities": len(session.entities)
        }
        
    except Exception as e:
        logger.error(f"❌ 添加测试球体失败: {e}")
        return {"error": f"球体添加失败: {str(e)}"}


@mcp.tool()
def create_complete_scene_with_sphere(show_viewer: bool = True) -> dict:
    """创建包含球体的完整场景（一次性创建，避免构建后无法添加实体的问题）
    
    Args:
        show_viewer: 是否显示3D窗口，默认True
    """
    if not GENESIS_AVAILABLE:
        return {"error": "Genesis未安装，请在genesis310环境中运行"}
    
    # 重置现有会话
    session.reset_session()
    
    if not session.ensure_genesis_init():
        return {"error": "Genesis初始化失败"}
    
    try:
        # 创建场景
        session.scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3, -1, 1.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=30,
                max_FPS=60,
            ),
            sim_options=gs.options.SimOptions(
                dt=0.003,
                substeps=5,
            ),
            sph_options=gs.options.SPHOptions(
                lower_bound=(-1.0, -1.0, 0.0),
                upper_bound=(1.0, 1.0, 2.0),
                particle_size=0.005,
            ),
            vis_options=gs.options.VisOptions(
                show_world_frame=True,
                world_frame_size=1.0,
                plane_reflection=True,
                ambient_light=(0.1, 0.1, 0.1),
                visualize_sph_boundary=False,
            ),
            renderer=gs.renderers.Rasterizer(),
            show_viewer=show_viewer,
        )
        
        # 一次性添加所有实体
        
        # 1. 地面
        plane = session.scene.add_entity(gs.morphs.Plane())
        session.entities['ground_plane'] = plane
        
        # 2. 机器人
        robot = session.scene.add_entity(
            gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml')
        )
        session.entities['franka_robot'] = robot
        session.robot = robot
        
        # 3. 杯子（源杯子）
        cup_pos = (0.55, 0.55, 0.01)
        cup_entity = session.scene.add_entity(
            morph=gs.morphs.Mesh(
                file='/home/dd/Genesis/Genesis/project/object/newscene.obj',
                scale=0.03,
                pos=cup_pos,
                # convexify=False,
                # decompose_nonconvex=True,
            ),
            material=gs.materials.Rigid(),
            surface=gs.surfaces.Default(
                color=(0.9, 0.8, 0.9),
                vis_mode='visual',
            ),
        )
        session.entities['cup'] = cup_entity
        
        # 4. 测试球体（位于杯子内部）
        sphere_pos = (0.55, 0.55, 0.05)  # 稍微高于杯子底部
        sphere_entity = session.scene.add_entity(
            morph=gs.morphs.Sphere(
                radius=0.010,  # 1.2cm半径，适合杯子大小
                pos=sphere_pos,
            ),
            material=gs.materials.Rigid(
            ),
            surface=gs.surfaces.Default(
                color=(1.0, 0.2, 0.2),  # 红色球体
                vis_mode='visual',
            ),
        )
        session.entities['test_sphere'] = sphere_entity
        
        
        cube = session.scene.add_entity(
            gs.morphs.Box(
                size = (0.01, 0.01, 0.01),
                pos  = (0.55, 0.55, 0.08),
            )
        )
        session.entities['cube'] = cube

        
        
        # 5. 目标容器
        container_pos = (0.48, -0.25, 0.01)
        container = session.scene.add_entity(
            morph=gs.morphs.Mesh(
                file="/home/dd/Genesis/Genesis/project/object/Glass_Cup.obj",
                scale=1.3,
                pos=container_pos,
                euler=(0, 0, 0), 
                convexify=False,
                decompose_nonconvex=True,
            ),
            material=gs.materials.Rigid(
            ),
            surface=gs.surfaces.Default(
                color=(0.8, 0.4, 0.2),
                vis_mode='visual',
            ),
        )
        session.entities['target_container'] = container
        
        # 一次性构建场景
        session.scene.build()
        session.built = True
        
        # 修正杯子和容器方向
        cup_entity.set_quat(gs.euler_to_quat((np.pi/2, 0, 0)))
        container.set_quat(gs.euler_to_quat((np.pi/2, 0, 0)))
        
        # 设置机器人控制参数
        robot.set_dofs_kp(np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 300, 300]))
        robot.set_dofs_kv(np.array([450, 450, 350, 350, 200, 200, 200, 30, 30]))
        # 修正力矩限制：前4个关节±87N，后3个手腕关节±12N（符合panda.xml定义），夹爪±200N
        robot.set_dofs_force_range(
            np.array([-87, -87, -87, -87, -12, -12, -12, -200, -200]),
            np.array([87, 87, 87, 87, 12, 12, 12, 200, 200])
        )
        
        session.stats["scenes_created"] += 1
        session.stats["entities_added"] += 5
        
        logger.info("🎬 完整Genesis场景创建成功（包含球体测试）")
        
        return {
            "success": True,
            "message": "🎬 完整仿真场景已创建（包含机器人、杯子、球体、目标容器）",
            "entities": list(session.entities.keys()),
            "total_entities": len(session.entities),
            "scene_built": True,
            "ready_for_actions": True,
            "test_setup": {
                "cup_position": cup_pos,
                "sphere_position": sphere_pos,
                "container_position": container_pos,
                "sphere_radius": 0.012,
                "test_ready": True
            },
            "usage_tip": "球体会在重力作用下落入杯子中，可以测试倾倒动作将球体倒入目标容器"
        }
        
    except Exception as e:
        logger.error(f"❌ 完整场景创建失败: {e}")
        return {"error": f"完整场景创建失败: {str(e)}"}


@mcp.tool()
def execute_robot_action(
    action_type: str = "safe_position",
    *,
    steps: int = 450, 
    approach_distance: float = 0.20,
    grasp_margin: float = 1.15,
    grasp_force: float = 15.0,
    lift_height: float = 0.25,
    use_path_planning: bool = True,
) -> dict:
    """在现有场景中执行机器人动作(优化版)
    Parameters
    ----------
    action_type : str
        动作类型：
        - "safe_position": 移动到安全位置
        - "reach_cup": 仅移动到杯子上方
        - "side_approach_grasp": 仅执行侧向抓取
        - "full_grasp_sequence": 完整抓取序列（reach + side_approach + grasp）
    steps : int
        每个阶段默认 step 数 (scene.step() 次数)，默认 450
    approach_distance : float
        预抓取高度(距离物体表面的 z 偏移)
    grasp_margin : float
        指间张开宽度系数
    grasp_force : float
        夹爪闭合施加的力(N)
    lift_height : float
        抬举高度(m)
    use_path_planning : bool
        是否优先使用 plan_path, 失败再直接控制
    """
    # -------- 前置检查 ----------
    if session.robot is None or session.scene is None:
        return {"error": "请先调用 create_*_scene 创建场景和机器人"}

    if not session.build_scene_if_needed():
        return {"error": "场景尚未 build, 无法执行动作"}

    robot = session.robot
    scene = session.scene
    end_effector = robot.get_link("hand")
    motors_dof = np.arange(7)
    fingers_dof = np.arange(7, 9)

    log: list[str] = []

    # ------------- SAFE_POSITION (原 DEMO) -----------------
    if action_type == "safe_position":
        logger.info("🛡️ 移动到安全位置...")
        target_qpos = np.array([0, -0.5, 0, -2.4, 0, 2.0, 0.8])
        robot.control_dofs_position(target_qpos, motors_dof)
        robot.control_dofs_position(np.array([0.04, 0.04]), fingers_dof)  # 打开夹爪
        for _ in range(150):
            scene.step()
        log.append("机器人已移动到安全位置")

    # ------------- REACH_CUP (单独执行) ---------------------
    elif action_type == "reach_cup":
        logger.info("📍 移动到杯子上方...")
        # 通过 session 内置方法动态获取杯子高度
        grasp_info = session.get_object_grasp_info("cup", grasp_height_ratio=0.8)
        if grasp_info is None:
            return {"error": "无法获取杯子信息, 请确认已添加 cup"}

        hover_pos   = grasp_info["position"] + np.array([0, 0, approach_distance])
        hover_quat  = gs.euler_to_quat((-np.pi/2, 0, np.pi/2))
        q_hover     = robot.inverse_kinematics(link=end_effector, pos=hover_pos, quat=hover_quat)
        q_hover[-2:] = 0.04  # 张开夹爪

        success = _move_with_fallback(robot, q_hover, motors_dof, scene, use_path_planning, log, steps)
        if not success:
            return {"error": "到达杯子上方失败"}
        log.append("已到达杯子上方")

    # --------- SIDE APPROACH + GRASP (单独执行) -------------
    elif action_type == "side_approach_grasp":
        logger.info("🔄 执行侧向接近并抓取...")
        # A. 计算抓取计划Q
        g_info = session.get_object_grasp_info("cup", 0.5)
        if g_info is None:
            return {"error": "获取杯子信息失败"}
        g_plan = session.calculate_grasp_approach(g_info, approach_distance, grasp_margin)
        if g_plan is None:
            return {"error": "计算抓取策略失败"}
        
        side_pre = g_plan["pre_grasp_pos"].copy()
        side_pre[1] -= 0.10
        
        # B. 移动到预抓取位
        q_pre = robot.inverse_kinematics(link=end_effector,
                                         pos=side_pre,
                                         quat=g_plan["grasp_quat"])
        q_pre[-2:] = 0.04
        if not _move_with_fallback(robot, q_pre, motors_dof, scene, use_path_planning, log, steps//3):
            return {"error": "移动到预抓取位失败"}
        log.append("到达侧面预抓取位")

        # C. 水平侧向靠近抓取位
        q_grasp = robot.inverse_kinematics(link=end_effector,
                                           pos=g_plan["grasp_pos"],
                                           quat=g_plan["grasp_quat"])
        robot.control_dofs_position(q_grasp[:-2], motors_dof)
        for _ in range(steps//3):
            scene.step()
        log.append("水平侧向靠近完成")

        # D. 闭合夹爪抓取
        robot.control_dofs_force(np.array([-grasp_force, -grasp_force]), fingers_dof)
        for _ in range(steps//3):
            scene.step()
        log.append(f"夹爪闭合, 施加 {grasp_force}N")

        # E. 抬起
        lift_pos = g_plan["grasp_pos"].copy()
        lift_pos[2] += lift_height
        q_lift = robot.inverse_kinematics(link=end_effector, pos=lift_pos, quat=g_plan["grasp_quat"])
        robot.control_dofs_position(q_lift[:-2], motors_dof)
        robot.control_dofs_force(np.array([-grasp_force, -grasp_force]), fingers_dof)
        for _ in range(steps//3):
            scene.step()
        log.append(f"已抬升 {lift_height}m")

    # --------- FULL GRASP SEQUENCE (完整抓取序列) -------------
    elif action_type == "full_grasp_sequence":
        logger.info("🎯 执行完整抓取序列...")
        
        # ===== 阶段1: REACH CUP =====
        log.append("【阶段1: 移动到杯子上方】")
        grasp_info = session.get_object_grasp_info("cup", grasp_height_ratio=0.8)
        if grasp_info is None:
            return {"error": "无法获取杯子信息"}
        
        hover_pos = grasp_info["position"] + np.array([0, 0, approach_distance])
        hover_quat = gs.euler_to_quat((-np.pi/2, 0, np.pi/2))
        q_hover = robot.inverse_kinematics(link=end_effector, pos=hover_pos, quat=hover_quat)
        q_hover[-2:] = 0.04
        
        if not _move_with_fallback(robot, q_hover, motors_dof, scene, use_path_planning, log, steps//4):
            return {"error": "到达杯子上方失败"}
        log.append("已到达杯子上方")
        
        # 稳定等待
        for _ in range(50):
            scene.step()
        
        # ===== 阶段2: SIDE APPROACH + GRASP =====
        log.append("【阶段2: 侧向接近并抓取】")
        g_info = session.get_object_grasp_info("cup", 0.5)
        g_plan = session.calculate_grasp_approach(g_info, approach_distance, grasp_margin)
        if g_plan is None:
            return {"error": "计算抓取策略失败"}
        
        # 2.1 侧面预抓取位
        side_pre = g_plan["pre_grasp_pos"].copy()
        side_pre[1] -= 0.10
        q_pre = robot.inverse_kinematics(link=end_effector, pos=side_pre, quat=g_plan["grasp_quat"])
        q_pre[-2:] = 0.04
        
        if not _move_with_fallback(robot, q_pre, motors_dof, scene, use_path_planning, log, steps//4):
            return {"error": "移动到预抓取位失败"}
        log.append("到达侧面预抓取位")
        
        # 2.2 水平接近
        q_grasp = robot.inverse_kinematics(link=end_effector, 
                                           pos=g_plan["grasp_pos"], 
                                           quat=g_plan["grasp_quat"])
        robot.control_dofs_position(q_grasp[:-2], motors_dof)
        for _ in range(steps//2):
            scene.step()
        log.append("水平接近抓取位置")
        
        # 2.3 闭合夹爪
        robot.control_dofs_force(np.array([-grasp_force, -grasp_force]), fingers_dof)
        for _ in range(steps//2):
            scene.step()
        log.append(f"执行抓取 (力: {grasp_force}N)")
        
        # 2.4 抬起
        lift_pos = g_plan["grasp_pos"].copy()
        lift_pos[2] += lift_height
        q_lift = robot.inverse_kinematics(link=end_effector, pos=lift_pos, quat=g_plan["grasp_quat"])
        robot.control_dofs_position(q_lift[:-2], motors_dof)
        robot.control_dofs_force(np.array([-grasp_force, -grasp_force]), fingers_dof)
        for _ in range(steps//4):
            scene.step()
        log.append(f"抬起完成 (高度: {lift_height}m)")
        
        log.append("完整抓取序列执行成功！")

    else:
        return {"error": f"未知动作类型: {action_type}"}

    session.stats["actions_executed"] += 1
    return {
        "success": True,
        "action_type": action_type,
        "steps_executed": steps,
        "execution_log": log,
        "path_planning_used": use_path_planning,
        "timestamp": str(np.datetime64('now'))
    }
def _move_with_fallback(robot, q_goal, motors_dof, scene, use_pp, log, steps=450):
    """优先 plan_path,失败则直接 control_dofs_position
    
    Parameters
    ----------
    steps : int
        直接控制时的步数
    """
    if use_pp:
        try:
            path = robot.plan_path(qpos_goal=q_goal, num_waypoints=120)
            for i, pt in enumerate(path):
                robot.control_dofs_position(pt)
                # 每个路径点执行多步以确保到达
                for _ in range(3):
                    scene.step()
                if i % 30 == 0:
                    logger.debug(f"路径执行进度: {i}/{len(path)}")
            log.append(f"✅ 路径规划成功 ({len(path)} 路径点)")
            return True
        except Exception as e:
            log.append(f"⚠️ 路径规划失败，切换直接控制: {str(e)}")

    # 直接控制回退方案
    robot.control_dofs_position(q_goal[:-2], motors_dof)
    robot.control_dofs_position(q_goal[-2:], fingers_dof)
    for _ in range(steps):
        scene.step()
    log.append("✅ 直接控制完成")
    return True

##########second version##########
@mcp.tool()
def execute_pour_sequence(
    target_position: tuple = (0.57,-0.39,0.31),  
    pour_angle: float = -2.5,  # 倾倒角度（弧度）- 增加倾倒角度
    horizontal_move_steps: int = 350,  # 水平移动步数
    pour_duration_steps: int = 400,    # 倾倒持续步数
    stabilize_steps: int = 150,         # 每阶段间的稳定步数
    maintain_grasp_force: float = 15.0,  # 增强抓取力
    use_wrist_only: bool = True        # 仅使用手腕倾倒
) -> dict:
    """
    优化的倾倒序列：使用End Effector IK实现稳定的姿态控制
    阶段1：保持垂直姿态水平移动到目标位置
    阶段2：执行倾倒动作（手腕旋转或姿态改变）
    阶段3：恢复垂直姿态
    """
    
    if session.robot is None or session.scene is None:
        return {"error": "请先创建场景和机器人"}
    
    robot = session.robot
    scene = session.scene
    end_effector = robot.get_link("hand")
    motors_dof = np.arange(7)
    fingers_dof = np.arange(7, 9)
    
    log = []
    
    try:
        # 获取当前末端执行器状态
        current_pos = end_effector.get_pos()
        current_quat = end_effector.get_quat()
        
        # 处理张量格式
        if hasattr(current_pos, 'cpu'):
            current_pos = current_pos.cpu().numpy()
        if len(current_pos.shape) > 1:
            current_pos = current_pos[0]
            
        if hasattr(current_quat, 'cpu'):
            current_quat = current_quat.cpu().numpy()
        if len(current_quat.shape) > 1:
            current_quat = current_quat[0]
        
        # 定义垂直向下的抓取姿态（保持杯子直立）
        # 这是关键！确保整个运输过程中保持这个姿态
        vertical_grasp_quat = euler_to_quat((-np.pi/2, 0, np.pi/2))
        
        log.append(f"📍 当前位置: {current_pos}")
        log.append(f"🎯 目标位置: {target_position}")
        log.append(f"📐 垂直抓取姿态: {vertical_grasp_quat}")
        
        # ====== 阶段1: 水平移动到目标位置 ======
        log.append("【阶段1: 水平移动到预倾倒位置】")
        
        # 构建目标位置
        if target_position[2] is None:
            # 保持当前高度
            target_pos = np.array([
                target_position[0],
                target_position[1],
                current_pos[2]
            ])
        else:
            target_pos = np.array(target_position)
        
        log.append(f"🎯 目标位置（含高度）: {target_pos}")
        
        # 使用End Effector IK，明确指定垂直姿态
        try:
            q_horizontal = robot.inverse_kinematics(
                link=end_effector,
                pos=target_pos,
                quat=vertical_grasp_quat  # 关键：始终保持垂直姿态
            )
            
            log.append("✅ IK计算成功（保持垂直姿态）")
            
            # 执行水平移动
            success = _move_with_fixed_orientation(
                robot, q_horizontal, motors_dof, scene, 
                True, log, horizontal_move_steps,
                maintain_grasp_force, fingers_dof,
                end_effector, vertical_grasp_quat
            )
            
            if not success:
                return {"error": "水平移动失败"}
            
            log.append(f"✅ 水平移动完成，到达位置: {target_pos}")
            
        except Exception as e:
            log.append(f"❌ 水平移动IK失败: {str(e)}")
            return {"error": f"水平移动失败: {str(e)}"}
        
        # 稳定等待
        for _ in range(stabilize_steps):
            robot.control_dofs_force(
                np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                fingers_dof
            )
            scene.step()
        
        # ====== 阶段2: 执行倾倒动作 ======
        log.append("【阶段2: 执行倾倒动作】")
        
        if use_wrist_only:
            # 方法A: 一次性手腕关节倾倒（优化版）
            log.append("🔄 使用手腕关节一次性倾倒模式")
            
            # 获取当前关节角度
            current_qpos = robot.get_dofs_position()
            if hasattr(current_qpos, 'cpu'):
                current_qpos = current_qpos.cpu().numpy()
            if len(current_qpos.shape) > 1:
                current_qpos = current_qpos[0]
            
            # 保存倾倒前的关节角度（用于恢复）
            pre_pour_qpos = current_qpos.copy()
            
            # 一次性倾倒：直接旋转到目标角度
            pour_qpos = pre_pour_qpos.copy()
            pour_qpos[6] += pour_angle  # 手腕旋转关节直接到目标角度
            
            log.append(f"🎯 一次性倾倒到目标角度: {np.degrees(pour_angle):.1f}°")
            
            # 控制机械臂关节到倾倒位置
            robot.control_dofs_position(pour_qpos[:-2], motors_dof)
            
            # 短暂等待到达目标位置
            approach_steps = 100  # 到达目标角度的步数
            for step in range(approach_steps):
                robot.control_dofs_force(
                    np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                    fingers_dof
                )
                scene.step()
                
                # 进度反馈
                if step % 25 == 0:
                    log.append(f"   到达倾倒位置进度: {step}/{approach_steps}")
            
            log.append("✅ 已到达倾倒位置")
            
            # 保持倾倒状态
            log.append(f"⏳ 保持倾倒状态 {pour_duration_steps} 步...")
            for step in range(pour_duration_steps):
                robot.control_dofs_force(
                    np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                    fingers_dof
                )
                scene.step()
                
                # 定期反馈
                if step % 100 == 0 and step > 0:
                    progress = (step / pour_duration_steps) * 100
                    log.append(f"   倾倒进度: {progress:.1f}%")
            
            log.append("✅ 倾倒保持阶段完成")
            
        else:
            # 方法B: 使用End Effector姿态控制一次性倾倒
            log.append("🔄 使用End Effector姿态一次性倾倒模式")
            
            # 计算倾倒姿态（在垂直基础上增加倾斜）
            tilt_quat = euler_to_quat((-np.pi/2 - pour_angle, 0, np.pi/2))
            
            try:
                # 一次性改变姿态到倾倒角度
                q_tilt = robot.inverse_kinematics(
                    link=end_effector,
                    pos=target_pos,
                    quat=tilt_quat
                )
                
                log.append(f"🎯 一次性倾倒到姿态角度: {np.degrees(pour_angle):.1f}°")
                
                # 控制到倾倒姿态
                robot.control_dofs_position(q_tilt[:-2], motors_dof)
                
                # 等待到达倾倒姿态
                approach_steps = 150  # 姿态控制需要更多步数
                for step in range(approach_steps):
                    robot.control_dofs_force(
                        np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                        fingers_dof
                    )
                    scene.step()
                    
                    if step % 30 == 0:
                        log.append(f"   到达倾倒姿态进度: {step}/{approach_steps}")
                
                log.append("✅ 已到达倾倒姿态")
                
                # 保持倾倒姿态
                log.append(f"⏳ 保持倾倒姿态 {pour_duration_steps} 步...")
                for step in range(pour_duration_steps):
                    robot.control_dofs_force(
                        np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                        fingers_dof
                    )
                    scene.step()
                    
                    if step % 100 == 0 and step > 0:
                        progress = (step / pour_duration_steps) * 100
                        log.append(f"   倾倒进度: {progress:.1f}%")
                
                log.append("✅ 倾倒姿态保持完成")
                    
            except Exception as e:
                log.append(f"❌ 姿态倾倒失败: {str(e)}")
                return {"error": f"倾倒执行失败: {str(e)}"}
        
        log.append(f"✅ 一次性倾倒完成（角度: {np.degrees(pour_angle):.1f}°）")
        
        # ====== 阶段3: 恢复垂直姿态 ======
        log.append("【阶段3: 一次性恢复垂直姿态】")
        
        if use_wrist_only:
            # 一次性恢复手腕角度到原始位置
            log.append("🔄 一次性恢复手腕角度")
            
            # 直接恢复到原始角度
            robot.control_dofs_position(pre_pour_qpos[:-2], motors_dof)
            
            # 等待恢复完成
            restore_steps = 120  # 恢复时间
            for step in range(restore_steps):
                robot.control_dofs_force(
                    np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                    fingers_dof
                )
                scene.step()
                
                # 进度反馈
                if step % 30 == 0:
                    log.append(f"   恢复进度: {step}/{restore_steps}")
            
            log.append("✅ 手腕角度已恢复")
            
        else:
            # 一次性恢复End Effector垂直姿态
            log.append("🔄 一次性恢复End Effector垂直姿态")
            
            try:
                # 直接计算恢复到垂直姿态的逆运动学
                q_restore = robot.inverse_kinematics(
                    link=end_effector,
                    pos=target_pos,
                    quat=vertical_grasp_quat  # 恢复到垂直姿态
                )
                
                # 一次性控制到恢复姿态
                robot.control_dofs_position(q_restore[:-2], motors_dof)
                
                # 等待恢复完成
                restore_steps = 150  # 姿态恢复需要更多时间
                for step in range(restore_steps):
                    robot.control_dofs_force(
                        np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                        fingers_dof
                    )
                    scene.step()
                    
                    if step % 30 == 0:
                        log.append(f"   姿态恢复进度: {step}/{restore_steps}")
                
                log.append("✅ End Effector姿态已恢复")
                        
            except Exception as e:
                log.append(f"⚠️ 恢复姿态失败: {str(e)}")
        
        # 最终稳定
        for _ in range(stabilize_steps):
            robot.control_dofs_force(
                np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                fingers_dof
            )
            scene.step()
        
        log.append("✅ 已恢复垂直姿态")
        
        # 验证最终姿态
        final_quat = end_effector.get_quat()
        if hasattr(final_quat, 'cpu'):
            final_quat = final_quat.cpu().numpy()
        if len(final_quat.shape) > 1:
            final_quat = final_quat[0]
        
        quat_error = 1.0 - abs(np.dot(final_quat, vertical_grasp_quat))
        log.append(f"📊 最终姿态误差: {quat_error:.6f}")
        
        session.stats["actions_executed"] += 1
        
        return {
            "success": True,
            "message": "🌊 优化的倾倒序列执行成功",
            "method": "wrist_only" if use_wrist_only else "end_effector",
            "execution_details": {
                "horizontal_target": target_pos.tolist(),
                "pour_angle_rad": pour_angle,
                "pour_angle_deg": np.degrees(pour_angle),
                "horizontal_steps": horizontal_move_steps,
                "pour_steps": pour_duration_steps,
                "maintain_grasp_force": maintain_grasp_force,
                "final_orientation_error": float(quat_error),
                "used_fixed_orientation": True
            },
            "execution_log": log,
            "timestamp": str(np.datetime64('now'))
        }
        
    except Exception as e:
        logger.error(f"❌ 倾倒序列失败: {e}")
        return {
            "error": f"倾倒执行失败: {str(e)}",
            "partial_log": log
        }

def _move_with_fixed_orientation(robot, q_goal, motors_dof, scene, use_pp, 
                                log, steps, grasp_force, fingers_dof,
                                end_effector, target_quat):
    """
    增强版移动函数：在移动过程中维持抓取力和姿态验证
    
    Args:
        robot: 机器人对象
        q_goal: 目标关节角度
        motors_dof: 机械臂关节索引
        scene: 场景对象
        use_pp: 是否使用路径规划
        log: 日志列表
        steps: 执行步数
        grasp_force: 抓取力
        fingers_dof: 夹爪关节索引
        end_effector: 末端执行器链接
        target_quat: 目标姿态四元数（用于验证）
    """
    if use_pp:
        try:
            # 使用路径规划生成平滑轨迹
            path = robot.plan_path(qpos_goal=q_goal, num_waypoints=100)
            
            orientation_errors = []
            
            for i, pt in enumerate(path):
                # 控制机械臂
                robot.control_dofs_position(pt[:-2], motors_dof)
                
                # 维持抓取力
                robot.control_dofs_force(
                    np.array([-grasp_force, -grasp_force]), 
                    fingers_dof
                )
                
                # 执行仿真步
                for _ in range(3):
                    scene.step()
                
                # 定期检查姿态保持情况
                if i % 20 == 0:
                    current_quat = end_effector.get_quat()
                    if hasattr(current_quat, 'cpu'):
                        current_quat = current_quat.cpu().numpy()
                    if len(current_quat.shape) > 1:
                        current_quat = current_quat[0]
                    
                    quat_error = 1.0 - abs(np.dot(current_quat, target_quat))
                    orientation_errors.append(quat_error)
                    
                    if i % 40 == 0:
                        log.append(f"   路径点 {i}/{len(path)}, 姿态误差: {quat_error:.6f}")
            
            # 统计姿态保持情况
            if orientation_errors:
                mean_error = np.mean(orientation_errors)
                max_error = np.max(orientation_errors)
                log.append(f"✅ 路径规划移动完成 - 平均姿态误差: {mean_error:.6f}, 最大误差: {max_error:.6f}")
            else:
                log.append(f"✅ 路径规划移动完成 ({len(path)} 路径点)")
                
            return True
            
        except Exception as e:
            log.append(f"⚠️ 路径规划失败，使用直接控制: {str(e)}")

    # 直接控制回退方案
    robot.control_dofs_position(q_goal[:-2], motors_dof)
    
    # 渐进式移动，持续维持抓取力
    for step in range(steps):
        robot.control_dofs_force(
            np.array([-grasp_force, -grasp_force]), 
            fingers_dof
        )
        scene.step()
        
        # 定期检查姿态
        if step % 50 == 0 and step > 0:
            current_quat = end_effector.get_quat()
            if hasattr(current_quat, 'cpu'):
                current_quat = current_quat.cpu().numpy()
            if len(current_quat.shape) > 1:
                current_quat = current_quat[0]
            
            quat_error = 1.0 - abs(np.dot(current_quat, target_quat))
            log.append(f"   步骤 {step}/{steps}, 姿态误差: {quat_error:.6f}")
    
    log.append("✅ 直接控制移动完成")
    return True

# @mcp.tool()
# def add_target_container(
#     position: tuple = (0.15, 0.55, 0.01),
#     container_scale: float = 1.3,
#     container_color: tuple = (0.8, 0.4, 0.2),
#     container_obj_file: str = "/home/dd/Genesis/Genesis/project/object/Glass_Cup.obj"  # 可以换成你的容器obj文件
# ) -> dict:
#     """
#     添加目标容器（使用固定obj模型，用于接收倾倒的液体）
    
#     Parameters:
#     -----------
#     position : tuple
#         容器位置 (x, y, z)
#     container_scale : float
#         容器缩放比例，默认0.04
#     container_color : tuple
#         容器颜色 (r, g, b)，默认橙色
#     container_obj_file : str
#         容器obj文件路径，如果文件不存在会回退到简单box
#     """
    
#     if session.scene is None:
#         return {"error": "请先创建场景"}
    
#     if hasattr(session, 'built') and session.built:
#         return {"error": "场景已构建，无法添加新实体"}
    
#     try:
#         # 尝试使用obj文件加载容器
#         try:
#             container = session.scene.add_entity(
#                 morph=gs.morphs.Mesh(
#                     file=container_obj_file,
#                     scale=container_scale,
#                     pos=position,
#                     euler=(0, 0, 0),  # 容器通常不需要旋转
#                     convexify=False,
#                     decompose_nonconvex=True,
#                 ),
#                 material=gs.materials.Rigid(
#                     rho=2000.0,
#                     friction=0.8,
#                     needs_coup=True,        # 关键！
#                     coup_friction=0.8,      # 液体-容器摩擦
#                 ),
#                 surface=gs.surfaces.Default(
#                     color=container_color,
#                     vis_mode='visual',
#                 ),
#             )
#             # container.set_quat(gs.euler_to_quat((np.pi/2, 0, 0)))
#             container_type = "mesh_obj"
#             success_msg = f"✅ 目标容器已添加（使用obj模型）"
            
#         except Exception as obj_error:
#             # 如果obj文件加载失败，回退到简单box
#             container = session.scene.add_entity(
#                 morph=gs.morphs.Box(
#                     pos=position,
#                     size=(container_scale*2, container_scale*2, container_scale*1.5)  # 基于scale计算size
#                 ),
#                 material=gs.materials.Rigid(
#                     rho=500.0,
#                     friction=0.7
#                 ),
#                 surface=gs.surfaces.Default(
#                     color=container_color,
#                     vis_mode='visual'
#                 )
#             )
            
#             container_type = "box_fallback"
#             success_msg = f"⚠️ obj文件加载失败，使用box容器作为备选"
            
#         # 记录容器实体
#         session.entities['target_container'] = container
#         session.stats["entities_added"] += 1
        
#         return {
#             "success": True,
#             "message": success_msg,
#             "container_details": {
#                 "type": container_type,
#                 "position": position,
#                 "scale": container_scale,
#                 "color": container_color,
#                 "obj_file": container_obj_file if container_type == "mesh_obj" else None
#             },
#             "fallback_used": container_type == "box_fallback",
#             "entity_name": "target_container"
#         }
        
#     except Exception as e:
#         return {"error": f"添加目标容器失败: {str(e)}"}
    
@mcp.tool()
def debug_robot_grasp_status() -> dict:
    """
    调试机器人抓取状态和当前配置
    """
    if session.robot is None:
        return {"error": "机器人未初始化"}
    
    robot = session.robot
    
    try:
        # 获取当前状态
        current_qpos = robot.get_dofs_position()
        if hasattr(current_qpos, 'cpu'):
            current_qpos = current_qpos.cpu().numpy()
        if len(current_qpos.shape) > 1:
            current_qpos = current_qpos[0]
        
        end_effector = robot.get_link("hand")
        current_pos = end_effector.get_pos()
        current_quat = end_effector.get_quat()
        
        if hasattr(current_pos, 'cpu'):
            current_pos = current_pos.cpu().numpy()
        if len(current_pos.shape) > 1:
            current_pos = current_pos[0]
            
        if hasattr(current_quat, 'cpu'):
            current_quat = current_quat.cpu().numpy()
        if len(current_quat.shape) > 1:
            current_quat = current_quat[0]
        
        return {
            "success": True,
            "robot_status": {
                "joint_positions": current_qpos.tolist(),
                "end_effector_pos": current_pos.tolist(),
                "end_effector_quat": current_quat.tolist(),
                "gripper_positions": [current_qpos[7], current_qpos[8]],
                "wrist_angle_deg": np.degrees(current_qpos[6]),
                "estimated_grasp_strength": "strong" if current_qpos[7] < 0.02 else "weak"
            },
            "available_links": [
                "link0", "link1", "link2", "link3", "link4", 
                "link5", "link6", "link7", "hand", 
                "left_finger", "right_finger"
            ],
            "key_joint_info": {
                "joint_6_wrist": f"{np.degrees(current_qpos[6]):.1f}°",
                "joint_7_gripper_left": f"{current_qpos[7]:.3f}",
                "joint_8_gripper_right": f"{current_qpos[8]:.3f}"
            }
        }
        
    except Exception as e:
        return {"error": f"获取机器人状态失败: {str(e)}"}


@mcp.tool()
def test_wrist_rotation(
    rotation_angle: float = 0.5,
    steps: int = 100,
    maintain_grasp_force: float = 8.0
) -> dict:
    """
    测试手腕旋转功能（用于验证倾倒控制）
    """
    if session.robot is None:
        return {"error": "机器人未初始化"}
    
    robot = session.robot
    scene = session.scene
    motors_dof = np.arange(7)
    fingers_dof = np.arange(7, 9)
    
    try:
        # 获取当前关节状态
        current_qpos = robot.get_dofs_position()
        if hasattr(current_qpos, 'cpu'):
            current_qpos = current_qpos.cpu().numpy()
        if len(current_qpos.shape) > 1:
            current_qpos = current_qpos[0]
        
        original_wrist_angle = current_qpos[6]
        
        # 执行旋转
        rotated_qpos = current_qpos.copy()
        rotated_qpos[6] += rotation_angle
        
        robot.control_dofs_position(rotated_qpos[:-2], motors_dof)
        
        for _ in range(steps):
            robot.control_dofs_force(
                np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                fingers_dof
            )
            scene.step()
        
        # 恢复原始角度
        robot.control_dofs_position(current_qpos[:-2], motors_dof)
        
        for _ in range(steps):
            robot.control_dofs_force(
                np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                fingers_dof
            )
            scene.step()
        
        return {
            "success": True,
            "message": "🔄 手腕旋转测试完成",
            "rotation_details": {
                "original_angle_deg": np.degrees(original_wrist_angle),
                "rotation_angle_deg": np.degrees(rotation_angle),
                "final_angle_deg": np.degrees(original_wrist_angle + rotation_angle),
                "steps": steps,
                "grasp_force": maintain_grasp_force
            }
        }
        
    except Exception as e:
        return {"error": f"手腕旋转测试失败: {str(e)}"}


@mcp.tool()
def test_extreme_pour(
    pour_angle: float = 5.5,  # 极端倾倒角度（约315度）
    steps_per_increment: int = 60,  # 每个角度增量的步数
    total_increments: int = 25,  # 总的角度增量数
    hold_duration: int = 600,  # 保持倾倒状态的时间
    maintain_grasp_force: float = 20.0  # 更强的抓取力
) -> dict:
    """
    测试极端倾倒角度功能（用于验证大角度倾倒效果）
    
    Args:
        pour_angle: 极端倾倒角度（弧度）
        steps_per_increment: 每个角度增量的仿真步数
        total_increments: 总的角度增量数
        hold_duration: 保持倾倒状态的时间
        maintain_grasp_force: 抓取力
    """
    if session.robot is None or session.scene is None:
        return {"error": "请先创建场景和机器人"}
    
    robot = session.robot
    scene = session.scene
    motors_dof = np.arange(7)
    fingers_dof = np.arange(7, 9)
    
    try:
        # 获取当前关节状态
        current_qpos = robot.get_dofs_position()
        if hasattr(current_qpos, 'cpu'):
            current_qpos = current_qpos.cpu().numpy()
        if len(current_qpos.shape) > 1:
            current_qpos = current_qpos[0]
        
        original_wrist_angle = current_qpos[6]
        pre_pour_qpos = current_qpos.copy()
        
        log = []
        log.append(f"🔥 开始极端倾倒测试 - 角度: {np.degrees(pour_angle):.1f}°")
        log.append(f"📊 参数: {total_increments}步增量, 每步{steps_per_increment}帧, 保持{hold_duration}帧")
        
        # 渐进式倾倒到极端角度
        angle_step = pour_angle / total_increments
        
        for i in range(total_increments):
            current_pour_angle = angle_step * (i + 1)
            incremental_qpos = pre_pour_qpos.copy()
            incremental_qpos[6] += current_pour_angle  # 手腕旋转关节
            
            # 控制机械臂关节
            robot.control_dofs_position(incremental_qpos[:-2], motors_dof)
            
            # 维持强抓取力
            robot.control_dofs_force(
                np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                fingers_dof
            )
            
            # 每个角度增量保持更长时间
            for _ in range(steps_per_increment):
                scene.step()
            
            # 更详细的进度报告
            if i % 5 == 0:
                degrees = np.degrees(current_pour_angle)
                progress = (i + 1) / total_increments * 100
                log.append(f"📐 倾倒进度: {degrees:.1f}° ({progress:.1f}%)")
        
        log.append(f"⏳ 保持极端倾倒状态 {hold_duration} 步...")
        # 保持极端倾倒状态
        for _ in range(hold_duration):
            robot.control_dofs_force(
                np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                fingers_dof
            )
            scene.step()
        
        log.append("🔄 开始恢复到原始位置...")
        # 渐进式恢复原始角度
        for i in range(total_increments):
            current_restore_angle = pour_angle * (1 - (i + 1) / total_increments)
            restore_qpos = pre_pour_qpos.copy()
            restore_qpos[6] += current_restore_angle
            
            robot.control_dofs_position(restore_qpos[:-2], motors_dof)
            robot.control_dofs_force(
                np.array([-maintain_grasp_force, -maintain_grasp_force]), 
                fingers_dof
            )
            
            for _ in range(steps_per_increment // 2):  # 恢复稍快一些
                scene.step()
            
            if i % 5 == 0:
                degrees = np.degrees(current_restore_angle)
                progress = (i + 1) / total_increments * 100
                log.append(f"🔙 恢复进度: {degrees:.1f}° ({progress:.1f}%)")
        
        log.append("✅ 极端倾倒测试完成")
        
        return {
            "success": True,
            "message": f"🔥 极端倾倒测试完成 - 最大角度: {np.degrees(pour_angle):.1f}°",
            "test_details": {
                "max_angle_deg": np.degrees(pour_angle),
                "max_angle_rad": pour_angle,
                "original_angle_deg": np.degrees(original_wrist_angle),
                "total_increments": total_increments,
                "steps_per_increment": steps_per_increment,
                "hold_duration": hold_duration,
                "grasp_force": maintain_grasp_force,
                "total_simulation_steps": total_increments * steps_per_increment * 2 + hold_duration
            },
            "execution_log": log
        }
        
    except Exception as e:
        return {"error": f"极端倾倒测试失败: {str(e)}"}



@mcp.tool()
def get_simulation_status() -> dict:
    """获取当前仿真状态"""
    return {
        "genesis_available": GENESIS_AVAILABLE,
        "genesis_initialized": session.initialized,
        "scene_created": session.scene is not None,
        "robot_ready": session.robot is not None,
        "entities": list(session.entities.keys()),
        "entity_count": len(session.entities),
        "statistics": session.stats,
        "environment": "genesis310",
        "mode": "persistent_session"
    }

@mcp.tool()
def reset_simulation() -> dict:
    """重置整个仿真会话"""
    success = session.reset_session()
    
    if success:
        return {
            "success": True,
            "message": "🔄 仿真会话已重置，可重新开始",
            "reset_complete": True
        }
    else:
        return {"error": "重置失败"}

# ==================== 资源定义 ====================

@mcp.resource("genesis://status/session")
def get_session_status() -> str:
    """获取详细的会话状态"""
    status_icon = "✅" if session.initialized else "❌"
    scene_icon = "🎬" if session.scene else "⭕"
    robot_icon = "🤖" if session.robot else "⭕"
    
    return f"""
Genesis持久化会话状态:
========================
🔧 Genesis初始化: {status_icon} {'已完成' if session.initialized else '未完成'}
{scene_icon} 场景状态: {'已创建' if session.scene else '未创建'}
{robot_icon} 机器人: {'已加载' if session.robot else '未加载'}

📦 场景实体 ({len(session.entities)}):
{chr(10).join([f"  • {name}" for name in session.entities.keys()])}

📊 统计信息:
{json.dumps(session.stats, indent=2, ensure_ascii=False)}

🎯 持久化模式: 单场景、单窗口、多轮交互
🔄 重置命令: reset_simulation()
"""

@mcp.resource("genesis://help/persistent")
def get_persistent_help() -> str:
    """获取持久化模式帮助"""
    return """
Genesis持久化MCP使用指南:
=========================

🎯 核心理念: 一个Genesis窗口，多轮交互

📋 推荐工作流程（方式1 - 一次性创建）:
1. create_complete_scene()            # 一次性创建完整场景（推荐）
2. execute_robot_action("demo")       # 机器人演示动作
3. execute_robot_action("reach_cup")  # 移动到杯子附近
4. execute_robot_action("grasp_cup")  # 抓取杯子
5. step_simulation(50)               # 推进仿真观察
6. reset_simulation()                # 重新开始

📋 分步工作流程（方式2 - 分步创建）:
1. create_simulation_scene()         # 创建基础场景（机器人+地面）
2. add_cup_with_liquid()            # 添加杯子和液体（触发构建）
3. execute_robot_action(...)        # 执行机器人动作
⚠️  注意：构建后无法再添加新实体（这是Genesis限制）

🤖 机器人动作类型:
- "demo"      : 简单演示动作（移动到安全位置）
- "reach_cup" : 移动到杯子附近（预抓取位置）
- "grasp_cup" : 完整的抓取序列（移动→抓取→抬起）

🔧 调试工具:
- debug_robot_status() : 查看机器人当前状态
- get_simulation_status() : 查看整体仿真状态

🚨 已知问题和解决方案:
- reach_cup无反应 → 使用debug_robot_status()检查状态
- 构建后无法添加实体 → 这是正常的,使用create_complete_scene()
- 窗口需手动关闭 → reset_simulation()会尝试自动关闭

💡 最佳实践:
- 优先使用create_complete_scene()避免构建限制
- 使用debug工具排查机器人动作问题
- 定期调用step_simulation()观察仿真进展
"""

# ==================== 服务器启动 ====================

def main():
    """启动持久化MCP服务器"""
    print("🤖 Genesis持久化MCP服务器启动中...")
    print(f"📁 工作目录: {os.getcwd()}")
    print(f"🐍 Python环境: {sys.executable}")
    print(f"🧬 Genesis状态: {'✅ 可用' if GENESIS_AVAILABLE else '❌ 不可用'}")
    print("🎯 模式: 持久化会话 (单场景、单窗口、多轮交互)")
    
    if not GENESIS_AVAILABLE:
        print("⚠️  Genesis未找到，请确保在genesis310环境中运行")
    
    print("🚀 MCP服务器就绪，等待连接...")
    print("💡 首次使用请调用: create_simulation_scene()")
    
    try:
        mcp.run(transport="stdio")
    except KeyboardInterrupt:
        print("\n🛑 服务器已停止")
        if session.scene:
            print("🔄 正在清理Genesis资源...")
            session.reset_session()
    except Exception as e:
        print(f"💥 服务器错误: {e}")

if __name__ == "__main__":
    main()