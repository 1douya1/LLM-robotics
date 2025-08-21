#include <mtc_tutorial/modular_task_builders.hpp>
#include <mtc_tutorial/pour_task_builder.hpp>  // 复用通用配置函数

#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/node.hpp>
#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace mtc = moveit::task_constructor;

namespace mtc_tutorial {

static double clamp(double v, double lo, double hi) { 
    return std::max(lo, std::min(hi, v)); 
}

// =============================================================================
// 1. 抓取任务构建器 (pick_container)
// =============================================================================

mtc::Task build_pick_task(const rclcpp::Node::SharedPtr& node, const PickTaskParams& p) {
    configure_moveit_params(node);
    sync_robot_model_params(node);

    mtc::Task task;
    task.stages()->setName("uf850 pick container task");
    task.loadRobotModel(node);

    // 读取夹爪闭合比例参数
    double gripper_close_ratio = 0.30;
    node->get_parameter_or<double>("gripper.close_ratio", gripper_close_ratio, 0.30);

    const std::string arm_group_name = "uf850";
    const std::string hand_group_name = "uf850_gripper"; 
    const std::string hand_frame = "link_tcp";

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // 创建规划器
    auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    pipeline_planner->setTimeout(3.0);
    pipeline_planner->setMaxVelocityScalingFactor(0.3);
    pipeline_planner->setMaxAccelerationScalingFactor(0.5);
    pipeline_planner->setPlannerId("RRTConnect");

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.3);
    cartesian_planner->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner->setStepSize(0.008);
    cartesian_planner->setJumpThreshold(0.0);

    // 阶段1：获取当前状态
    mtc::Stage* current_state_ptr = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        current_state_ptr = stage.get();
        task.add(std::move(stage));
    }

    // 阶段2：打开夹爪
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", 
                                      std::make_shared<mtc::solvers::JointInterpolationPlanner>());
        stage->setGroup(hand_group_name);
        stage->setGoal("open");
        task.add(std::move(stage));
    }

    // 阶段3：连接到抓取位置
    {
        auto stage = std::make_unique<mtc::stages::Connect>("move to pick location",
            mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, pipeline_planner } });
        stage->setTimeout(15.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    // 阶段4：抓取序列
    {
        auto grasp = std::make_unique<mtc::SerialContainer>("grasp container");
        task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // 4.1 接近物体（使用安全高度的垂直接近）
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach container", cartesian_planner);
            stage->properties().set("marker_ns", "approach_container");
            stage->properties().set("link", hand_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(p.safe_approach_height, p.safe_approach_height);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "link_base";  // 使用世界坐标系
            vec.vector.z = -1.0;                 // 垂直向下
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // 4.2 生成抓取姿态
        {
            auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose("open");
            stage->setObject(p.object_id);
            stage->setAngleDelta(M_PI / 12.0);
            stage->setMonitoredStage(current_state_ptr);

            // 设置抓取变换（侧向抓取）
            Eigen::Isometry3d grasp_frame_transform;
            grasp_frame_transform.setIdentity();
            Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
            grasp_frame_transform.linear() = q.matrix();
            grasp_frame_transform.translation().z() = 0.02;

            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(16);
            wrapper->setMinSolutionDistance(0.3);
            wrapper->setIKFrame(grasp_frame_transform, hand_frame);
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            wrapper->setTimeout(8.0);
            grasp->insert(std::move(wrapper));
        }

        // 4.3 允许手与物体的碰撞
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,container)");
            stage->allowCollisions(p.object_id,
                task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

        // 4.4 微插入提高抓取稳定性
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("pre-close insert", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setIKFrame(hand_frame);
            stage->setMinMaxDistance(0.01, 0.03);
            stage->properties().set("marker_ns", "pre_close_insert");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // 4.5 关闭夹爪
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", 
                                          std::make_shared<mtc::solvers::JointInterpolationPlanner>());
            stage->setGroup(hand_group_name);
            std::map<std::string, double> close_goal;
            close_goal["drive_joint"] = gripper_close_ratio;
            stage->setGoal(close_goal);
            grasp->insert(std::move(stage));
        }

        // 4.6 附着对象到夹爪
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach container");
            stage->attachObject(p.object_id, hand_frame);
            grasp->insert(std::move(stage));
        }

        // 4.7 抬升物体
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lift container", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lift_container");
            stage->setMinMaxDistance(p.lift_height, p.lift_height + 0.05);
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "link_base";
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        task.add(std::move(grasp));
    }

    return task;
}

// =============================================================================
// 2. 纯倾倒任务构建器 (pour_to_target)
// =============================================================================

mtc::Task build_pour_only_task(const rclcpp::Node::SharedPtr& node, const PourOnlyTaskParams& p) {
    configure_moveit_params(node);
    sync_robot_model_params(node);

    mtc::Task task;
    task.stages()->setName("uf850 pour only task");
    task.loadRobotModel(node);

    const std::string arm_group_name = "uf850";
    const std::string hand_group_name = "uf850_gripper";
    const std::string hand_frame = "link_tcp";

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // 创建规划器
    auto slow_cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    slow_cartesian_planner->setMaxVelocityScalingFactor(0.15);
    slow_cartesian_planner->setMaxAccelerationScalingFactor(0.3);
    slow_cartesian_planner->setStepSize(0.008);
    slow_cartesian_planner->setJumpThreshold(0.0);

    auto slow_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    const double joint6_max_rad_s = 2.0;
    double scaling_from_deg = (p.tilt_speed_deg_s * M_PI / 180.0) / joint6_max_rad_s;
    slow_interpolation_planner->setMaxVelocityScalingFactor(clamp(scaling_from_deg, 0.05, 1.0));
    slow_interpolation_planner->setMaxAccelerationScalingFactor(0.3);

    // 阶段1：获取当前状态
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        task.add(std::move(stage));
    }

    // 阶段2：倾倒序列
    {
        auto pour = std::make_unique<mtc::SerialContainer>("pour water");
        task.properties().exposeTo(pour->properties(), { "eef", "group", "ik_frame" });
        pour->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // 2.1 移动到倾倒位置
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("move to pour position", slow_cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(p.move_to_pour_min, p.move_to_pour_max);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "move_to_pour");
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "link_base";
            vec.vector.y = -1.0;  // 沿-Y方向移动
            stage->setDirection(vec);
            pour->insert(std::move(stage));
        }

        // 2.2 倾斜开始
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("tilt start", slow_interpolation_planner);
            stage->setGroup(arm_group_name);
            std::map<std::string, double> joint_goal;
            joint_goal["joint6"] = p.tilt_start_deg * M_PI / 180.0;
            stage->setGoal(joint_goal);
            pour->insert(std::move(stage));
        }

        // 2.3 倾斜到结束角度
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("tilt to end", slow_interpolation_planner);
            stage->setGroup(arm_group_name);
            std::map<std::string, double> joint_goal;
            joint_goal["joint6"] = p.tilt_end_deg * M_PI / 180.0;
            stage->setGoal(joint_goal);
            pour->insert(std::move(stage));
        }

        // 2.4 保持倾倒位置
        if (p.pour_hold_sec > 0.0) {
            auto stage = std::make_unique<mtc::stages::MoveTo>("hold pour position", slow_interpolation_planner);
            stage->setGroup(arm_group_name);
            std::map<std::string, double> joint_goal;
            joint_goal["joint6"] = p.tilt_end_deg * M_PI / 180.0;
            stage->setGoal(joint_goal);
            stage->setTimeout(p.pour_hold_sec);
            pour->insert(std::move(stage));
        }

        // 2.5 从倾倒位置返回
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("return from pour", slow_interpolation_planner);
            stage->setGroup(arm_group_name);
            std::map<std::string, double> joint_goal;
            joint_goal["joint6"] = p.tilt_start_deg * M_PI / 180.0;
            stage->setGoal(joint_goal);
            pour->insert(std::move(stage));
        }

        task.add(std::move(pour));
    }

    return task;
}

// =============================================================================
// 3. 放置任务构建器 (place_container)
// =============================================================================

mtc::Task build_place_task(const rclcpp::Node::SharedPtr& node, const PlaceTaskParams& p) {
    configure_moveit_params(node);
    sync_robot_model_params(node);

    mtc::Task task;
    task.stages()->setName("uf850 place container task");
    task.loadRobotModel(node);

    const std::string arm_group_name = "uf850";
    const std::string hand_group_name = "uf850_gripper";
    const std::string hand_frame = "link_tcp";

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame);

    // 创建规划器
    auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    pipeline_planner->setTimeout(3.0);
    pipeline_planner->setMaxVelocityScalingFactor(0.3);
    pipeline_planner->setMaxAccelerationScalingFactor(0.5);
    pipeline_planner->setPlannerId("RRTConnect");

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    interpolation_planner->setMaxVelocityScalingFactor(0.5);
    interpolation_planner->setMaxAccelerationScalingFactor(0.6);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.3);
    cartesian_planner->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner->setStepSize(0.008);
    cartesian_planner->setJumpThreshold(0.0);

    // 阶段1：获取当前状态
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        task.add(std::move(stage));
    }

    // 阶段2：附着对象到夹爪（模拟已经抓住的状态）- 这是关键的状态管理阶段
    mtc::Stage* attach_object_stage_ptr = nullptr;
    {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object for place");
        stage->attachObject(p.object_id, hand_frame);
        attach_object_stage_ptr = stage.get();  // 保存指针用于监控
        task.add(std::move(stage));
    }

    // 阶段3：连接到放置位置
    {
        auto stage = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, interpolation_planner } });
        stage->setTimeout(15.0);
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stage));
    }

    // 阶段4：放置容器序列（完整版本，按照mtc_tutorial.cpp）
    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
        place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

        // 4.1 生成放置姿态 - 使用完整的GeneratePlacePose
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject(p.object_id);

            // 设置放置目标位置
            geometry_msgs::msg::PoseStamped target_pose_msg;
            target_pose_msg.header.frame_id = "link_base";
            if (p.target_pose.has_value()) {
                target_pose_msg.pose = p.target_pose.value();
            } else {
                // 使用默认放置位置
                target_pose_msg.pose.position.x = 0.0;
                target_pose_msg.pose.position.y = -0.45;
                target_pose_msg.pose.position.z = 0.18;
                target_pose_msg.pose.orientation.w = 1.0;
            }
            stage->setPose(target_pose_msg);
            
            // 关键：设置监控阶段 - 这是GeneratePlacePose正常工作的必要条件
            if (attach_object_stage_ptr) {
                stage->setMonitoredStage(attach_object_stage_ptr);
            }

            // 计算IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(3);
            wrapper->setMinSolutionDistance(0.5);
            
            // 定义放置时的物体框架 - 让物体保持垂直
            Eigen::Isometry3d place_frame_transform;
            place_frame_transform.setIdentity();
            wrapper->setIKFrame(place_frame_transform, p.object_id);
            
            wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
            wrapper->setTimeout(5.0);
            place->insert(std::move(wrapper));
        }

        // 4.2 降低物体到放置位置
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(p.lower_min, p.lower_max);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "lower_object");
            
            // 设置向下的方向
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = "link_base";
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        // 4.3 打开夹爪释放物体
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("release object", interpolation_planner);
            stage->setGroup(hand_group_name);
            stage->setGoal("open");
            place->insert(std::move(stage));
        }

        // 4.4 禁止手和物体之间的碰撞
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions(p.object_id,
                task.getRobotModel()
                    ->getJointModelGroup(hand_group_name)
                    ->getLinkModelNamesWithCollisionGeometry(),
                false);
            place->insert(std::move(stage));
        }

        // 4.5 分离物体
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
            stage->detachObject(p.object_id, hand_frame);
            place->insert(std::move(stage));
        }

        // 4.6 后退
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(p.retreat_min, p.retreat_max);
            stage->setIKFrame(hand_frame);
            stage->properties().set("marker_ns", "retreat");

            // 设置后退方向
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = hand_frame;
            vec.vector.z = -1.0;  // 向后退
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }
        
        task.add(std::move(place));
    }

    return task;
}

// =============================================================================
// 4. 返回任务构建器 (return_to_home)
// =============================================================================

mtc::Task build_return_task(const rclcpp::Node::SharedPtr& node, const ReturnTaskParams& p) {
    configure_moveit_params(node);
    sync_robot_model_params(node);

    mtc::Task task;
    task.stages()->setName("uf850 return home task");
    task.loadRobotModel(node);

    const std::string arm_group_name = "uf850";

    task.setProperty("group", arm_group_name);

    // 创建规划器
    auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    pipeline_planner->setTimeout(3.0);
    pipeline_planner->setMaxVelocityScalingFactor(0.3);
    pipeline_planner->setMaxAccelerationScalingFactor(0.5);
    pipeline_planner->setPlannerId("RRTConnect");

    // 阶段1：获取当前状态
    {
        auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
        task.add(std::move(stage));
    }

    // 阶段2：返回初始位置
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>("return to home", pipeline_planner);
        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGroup(arm_group_name);
        
        if (p.target_joints.has_value()) {
            // 使用指定的关节目标
            stage->setGoal(p.target_joints.value());
        } else {
            // 使用"home"配置
            stage->setGoal("home");
        }
        
        stage->setTimeout(p.timeout_sec);
        task.add(std::move(stage));
    }

    return task;
}

} // namespace mtc_tutorial 