#include <mtc_tutorial/pour_task_builder.hpp>

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

namespace mtc = moveit::task_constructor;

namespace mtc_tutorial {

static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

void configure_moveit_params(const rclcpp::Node::SharedPtr& node) {
  auto declare_if = [&](const std::string& name, const rclcpp::ParameterValue& def){
    try { if (!node->has_parameter(name)) node->declare_parameter(name, def); } catch (...) {}
  };
  declare_if("robot_description", rclcpp::ParameterValue(""));
  declare_if("robot_description_semantic", rclcpp::ParameterValue(""));
  declare_if("default_planning_pipeline", rclcpp::ParameterValue("ompl"));
  declare_if("planning_pipelines", rclcpp::ParameterValue(std::vector<std::string>{"ompl"}));
  declare_if("ompl.planning_plugin", rclcpp::ParameterValue("ompl_interface/OMPLPlanner"));
  declare_if("ompl.request_adapters", rclcpp::ParameterValue(
    std::string("default_planner_request_adapters/AddTimeOptimalParameterization ") +
    "default_planner_request_adapters/FixWorkspaceBounds "
    "default_planner_request_adapters/FixStartStateBounds "
    "default_planner_request_adapters/FixStartStateCollision "
    "default_planner_request_adapters/FixStartStatePathConstraints"));
  declare_if("trajectory_execution.allowed_execution_duration_scaling", rclcpp::ParameterValue(5.0));
  declare_if("trajectory_execution.allowed_goal_duration_margin", rclcpp::ParameterValue(2.0));
  declare_if("trajectory_execution.allowed_start_tolerance", rclcpp::ParameterValue(0.05));
  declare_if("moveit_controller_manager", rclcpp::ParameterValue("moveit_simple_controller_manager/MoveItSimpleControllerManager"));
  declare_if("robot_description_kinematics.uf850.kinematics_solver", rclcpp::ParameterValue(""));
  declare_if("robot_description_kinematics.uf850.kinematics_solver_search_resolution", rclcpp::ParameterValue(0.005));
  declare_if("robot_description_kinematics.uf850.kinematics_solver_timeout", rclcpp::ParameterValue(0.005));
  declare_if("robot_description_kinematics.uf850.kinematics_solver_attempts", rclcpp::ParameterValue(3));
  for (int i = 1; i <= 6; ++i) {
    const std::string j = "robot_description_planning.joint_limits.joint" + std::to_string(i);
    declare_if(j + ".has_acceleration_limits", rclcpp::ParameterValue(true));
    declare_if(j + ".max_acceleration", rclcpp::ParameterValue(5.0));
    declare_if(j + ".has_velocity_limits", rclcpp::ParameterValue(true));
    declare_if(j + ".max_velocity", rclcpp::ParameterValue(1.0));
  }

  // 如果已存在非空 robot_description / SRDF，则不强制覆盖
  std::string urdf_existing, srdf_existing;
  node->get_parameter_or<std::string>("robot_description", urdf_existing, "");
  node->get_parameter_or<std::string>("robot_description_semantic", srdf_existing, "");

  // 使用独立 probe 节点避免 executor 冲突
  auto probe = std::make_shared<rclcpp::Node>(
      "pour_param_probe", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto client = std::make_shared<rclcpp::SyncParametersClient>(probe, "/move_group");
  if (client->wait_for_service(std::chrono::seconds(15))) {
    try {
      auto base = client->get_parameters({"robot_description", "robot_description_semantic"});
      if (base.size() >= 2) {
        if (urdf_existing.empty() && base[0].get_type() == rclcpp::PARAMETER_STRING)
          node->set_parameter(rclcpp::Parameter("robot_description", base[0].as_string()));
        if (srdf_existing.empty() && base[1].get_type() == rclcpp::PARAMETER_STRING)
          node->set_parameter(rclcpp::Parameter("robot_description_semantic", base[1].as_string()));
      }
      auto planning = client->get_parameters({"default_planning_pipeline","planning_pipelines","ompl.planning_plugin","ompl.request_adapters"});
      if (planning.size() == 4 && planning[3].get_type() == rclcpp::PARAMETER_STRING) {
        std::string adapters = planning[3].as_string();
        if (adapters.find("AddTimeOptimalParameterization") == std::string::npos)
          adapters = std::string("default_planner_request_adapters/AddTimeOptimalParameterization ") + adapters;
        node->set_parameter(rclcpp::Parameter("default_planning_pipeline", planning[0].as_string()));
        node->set_parameter(rclcpp::Parameter("planning_pipelines", planning[1].as_string_array()));
        node->set_parameter(rclcpp::Parameter("ompl.planning_plugin", planning[2].as_string()));
        node->set_parameter(rclcpp::Parameter("ompl.request_adapters", adapters));
      }
      auto execp = client->get_parameters({"trajectory_execution.allowed_execution_duration_scaling","trajectory_execution.allowed_goal_duration_margin","trajectory_execution.allowed_start_tolerance","moveit_controller_manager"});
      if (execp.size() == 4) {
        node->set_parameter(rclcpp::Parameter("trajectory_execution.allowed_execution_duration_scaling", 5.0));
        node->set_parameter(rclcpp::Parameter("trajectory_execution.allowed_goal_duration_margin", 2.0));
        node->set_parameter(rclcpp::Parameter("trajectory_execution.allowed_start_tolerance", 0.05));
        if (execp[3].get_type() == rclcpp::PARAMETER_STRING)
          node->set_parameter(rclcpp::Parameter("moveit_controller_manager", execp[3].as_string()));
      }
      auto kin = client->get_parameters({"robot_description_kinematics.uf850.kinematics_solver","robot_description_kinematics.uf850.kinematics_solver_search_resolution","robot_description_kinematics.uf850.kinematics_solver_timeout","robot_description_kinematics.uf850.kinematics_solver_attempts"});
      if (kin.size() == 4 && kin[0].get_type() == rclcpp::PARAMETER_STRING && !kin[0].as_string().empty()) {
        node->set_parameter(rclcpp::Parameter("robot_description_kinematics.uf850.kinematics_solver", kin[0].as_string()));
        node->set_parameter(rclcpp::Parameter("robot_description_kinematics.uf850.kinematics_solver_search_resolution", kin[1].as_double()));
        node->set_parameter(rclcpp::Parameter("robot_description_kinematics.uf850.kinematics_solver_timeout", kin[2].as_double()));
        node->set_parameter(rclcpp::Parameter("robot_description_kinematics.uf850.kinematics_solver_attempts", kin[3].as_int()));
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(node->get_logger(), "configure_moveit_params: exception: %s", e.what());
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "configure_moveit_params: /move_group param service not available, using declared defaults");
  }
}

static double clamp(double v, double lo, double hi);

void sync_robot_model_params(const rclcpp::Node::SharedPtr& node) {
  std::string urdf_existing, srdf_existing;
  node->get_parameter_or<std::string>("robot_description", urdf_existing, "");
  node->get_parameter_or<std::string>("robot_description_semantic", srdf_existing, "");
  if (!urdf_existing.empty() && !srdf_existing.empty()) return;  // 已有

  auto probe = std::make_shared<rclcpp::Node>(
      "pour_param_probe2", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto client = std::make_shared<rclcpp::SyncParametersClient>(probe, "/move_group");
  if (!client->wait_for_service(std::chrono::seconds(20))) {
    RCLCPP_WARN(node->get_logger(), "wait /move_group param service timeout; robot model params may be missing");
    return;
  }
  try {
    auto params = client->get_parameters({"robot_description", "robot_description_semantic"});
    if (params.size() >= 2 && params[0].get_type() == rclcpp::PARAMETER_STRING && params[1].get_type() == rclcpp::PARAMETER_STRING) {
      const auto& urdf = params[0].as_string();
      const auto& srdf = params[1].as_string();
      if (!urdf.empty()) node->set_parameter(rclcpp::Parameter("robot_description", urdf));
      if (!srdf.empty()) node->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf));
      RCLCPP_INFO(node->get_logger(), "synced robot_description + SRDF from /move_group");
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(), "exception fetching params from /move_group: %s", e.what());
  }
}

mtc::Task build_pour_task(const rclcpp::Node::SharedPtr& node, const PourTaskParams& p) {
  configure_moveit_params(node);
  sync_robot_model_params(node);

  mtc::Task task;
  task.stages()->setName("uf850 execute pour task");
  task.loadRobotModel(node);

  // 简化场景
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<moveit_msgs::msg::CollisionObject> objs;
  {
    moveit_msgs::msg::CollisionObject table_surface; table_surface.id = "table_surface"; table_surface.header.frame_id = "link_base";
    table_surface.primitives.resize(1);
    table_surface.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table_surface.primitives[0].dimensions = {1.0, 1.5, 0.01};
    geometry_msgs::msg::Pose pz; pz.orientation.w = 1.0; pz.position.y = -0.25; pz.position.z = -0.01; table_surface.pose = pz;
    table_surface.operation = moveit_msgs::msg::CollisionObject::ADD; objs.push_back(table_surface);
  }
  {
    moveit_msgs::msg::CollisionObject object; object.id = "object"; object.header.frame_id = "link_base";
    object.primitives.resize(1); object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.1, 0.02};
    geometry_msgs::msg::Pose pose; pose.orientation.w = 1.0; pose.position.y = -0.40; pose.position.z = 0.13; object.pose = pose;

    // 如果上游注入了 cup_pose.* 参数，则覆盖默认位置
    double cx, cy, cz, qx, qy, qz, qw;
    if (node->has_parameter("cup_pose.x") &&
        node->get_parameter("cup_pose.x", cx) &&
        node->get_parameter("cup_pose.y", cy) &&
        node->get_parameter("cup_pose.z", cz)) {
      // 姿态可选
      if (node->get_parameter("cup_pose.qx", qx) &&
          node->get_parameter("cup_pose.qy", qy) &&
          node->get_parameter("cup_pose.qz", qz) &&
          node->get_parameter("cup_pose.qw", qw)) {
        pose.orientation.x = qx; pose.orientation.y = qy; pose.orientation.z = qz; pose.orientation.w = qw;
      }
      pose.position.x = cx; pose.position.y = cy; pose.position.z = cz;
      object.pose = pose;
      RCLCPP_INFO(node->get_logger(), "Using cup_pose from parameters: (%.3f, %.3f, %.3f)", cx, cy, cz);
    }

    object.operation = moveit_msgs::msg::CollisionObject::ADD; objs.push_back(object);
  }
  psi.applyCollisionObjects(objs);

  const std::string arm_group_name = "uf850";
  const std::string hand_group_name = "uf850_gripper";
  const std::string hand_frame = "link_tcp";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

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

  auto slow_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  const double joint6_max_rad_s = 2.0;
  double scaling_from_deg = (p.tilt_speed_deg_s * M_PI / 180.0) / joint6_max_rad_s;
  slow_interpolation_planner->setMaxVelocityScalingFactor(clamp(scaling_from_deg, 0.05, 1.0));
  slow_interpolation_planner->setMaxAccelerationScalingFactor(0.3);

  // 开始
  mtc::Stage* current_state_ptr = nullptr;
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));
  }
  // 打开夹爪
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", std::make_shared<mtc::solvers::JointInterpolationPlanner>());
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }
  // 连接
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, pipeline_planner } });
    stage->setTimeout(15.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // 抓取
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(p.approach_min, p.approach_max);
      geometry_msgs::msg::Vector3Stamped vec; vec.header.frame_id = hand_frame; vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12.0);
      stage->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform; grasp_frame_transform.setIdentity();
      // 与 mtc_tutorial 对齐的侧向抓取朝向
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.02;  // 端点到物体中心的微距
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(16);
      wrapper->setMinSolutionDistance(0.3);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      wrapper->setTimeout(8.0);
      grasp->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
        task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }

    // 在关爪前进行微插入，提高抓取稳定性
    {
      auto insert = std::make_unique<mtc::stages::MoveRelative>("pre-close insert", cartesian_planner);
      insert->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      insert->setIKFrame(hand_frame);
      insert->setMinMaxDistance(0.01, 0.03);
      insert->properties().set("marker_ns", "pre_close_insert");
      geometry_msgs::msg::Vector3Stamped vec_ins; vec_ins.header.frame_id = hand_frame; vec_ins.vector.z = 1.0;
      insert->setDirection(vec_ins);
      grasp->insert(std::move(insert));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", std::make_shared<mtc::solvers::JointInterpolationPlanner>());
      stage->setGroup(hand_group_name);
      std::map<std::string,double> close; close["drive_joint"] = 0.85;
      stage->setGoal(close);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");
      stage->setMinMaxDistance(p.lift_height, p.lift_height + 0.05);
      geometry_msgs::msg::Vector3Stamped vec; vec.header.frame_id = "link_base"; vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // 倒水
  {
    auto pour = std::make_unique<mtc::SerialContainer>("pour water");
    task.properties().exposeTo(pour->properties(), { "eef", "group", "ik_frame" });
    pour->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    {
      auto s = std::make_unique<mtc::stages::MoveTo>("tilt start", slow_interpolation_planner);
      s->setGroup(arm_group_name);
      std::map<std::string,double> j; j["joint6"] = p.tilt_start_deg * M_PI / 180.0;
      s->setGoal(j);
      pour->insert(std::move(s));
    }
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("tilt to end", slow_interpolation_planner);
      s->setGroup(arm_group_name);
      std::map<std::string,double> j; j["joint6"] = p.tilt_end_deg * M_PI / 180.0;
      s->setGoal(j);
      pour->insert(std::move(s));
    }
    if (p.pour_hold_sec > 0.0) {
      auto s = std::make_unique<mtc::stages::MoveTo>("hold pour position", slow_interpolation_planner);
      s->setGroup(arm_group_name);
      std::map<std::string,double> j; j["joint6"] = p.tilt_end_deg * M_PI / 180.0;
      s->setGoal(j);
      s->setTimeout(p.pour_hold_sec);
      pour->insert(std::move(s));
    }
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("return from pour", slow_interpolation_planner);
      s->setGroup(arm_group_name);
      std::map<std::string,double> j; j["joint6"] = p.tilt_start_deg * M_PI / 180.0;
      s->setGoal(j);
      pour->insert(std::move(s));
    }

    task.add(std::move(pour));
  }

  return task;
}

} // namespace mtc_tutorial 