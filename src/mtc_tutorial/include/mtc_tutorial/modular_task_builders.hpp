#pragma once

#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>

namespace mtc_tutorial {

// 抓取任务参数
struct PickTaskParams {
  // 目标物体位置
  double source_x = 0.0;
  double source_y = -0.4; 
  double source_z = 0.13;
  double source_qx = 0.0;
  double source_qy = 0.0;
  double source_qz = 0.0;
  double source_qw = 1.0;
  
  // 抓取提示
  double approach_min = 0.05;
  double approach_max = 0.15;
  double lift_height = 0.12;
  
  // 安全高度（先到达抓取点上方的高度，再垂直下降同样距离）
  double safe_approach_height = 0.10;
  
  // 其他选项
  bool plan_only = false;
  std::string object_id = "object";
};

// 纯倾倒任务参数（不包含抓取和放置）
struct PourOnlyTaskParams {
  // 目标倾倒位置
  double target_x = 0.1;
  double target_y = -0.5;
  double target_z = 0.13;
  
  // 倾斜参数
  double tilt_start_deg = 45.0;
  double tilt_end_deg = 120.0;
  double tilt_speed_deg_s = 25.0;
  double pour_hold_sec = 2.0;
  
  // 移动参数
  double move_to_pour_min = 0.08;
  double move_to_pour_max = 0.15;
  
  bool plan_only = false;
};

// 放置任务参数
struct PlaceTaskParams {
  // 放置目标位置（可选，使用默认位置）
  std::optional<geometry_msgs::msg::Pose> target_pose = std::nullopt;
  
  // 放置参数
  double lower_min = 0.03;
  double lower_max = 0.2;
  double retreat_min = 0.05;
  double retreat_max = 0.1;
  
  bool plan_only = false;
  std::string object_id = "object";
};

// 返回任务参数
struct ReturnTaskParams {
  // 目标关节配置（可选，使用"home"配置）
  std::optional<std::map<std::string, double>> target_joints = std::nullopt;
  
  bool plan_only = false;
  double timeout_sec = 15.0;
};

// 任务构建器函数声明
moveit::task_constructor::Task build_pick_task(const rclcpp::Node::SharedPtr& node, 
                                                const PickTaskParams& params);

moveit::task_constructor::Task build_pour_only_task(const rclcpp::Node::SharedPtr& node,
                                                     const PourOnlyTaskParams& params);

moveit::task_constructor::Task build_place_task(const rclcpp::Node::SharedPtr& node,
                                                 const PlaceTaskParams& params);

moveit::task_constructor::Task build_return_task(const rclcpp::Node::SharedPtr& node,
                                                  const ReturnTaskParams& params);

// 通用配置函数（从pour_task_builder.cpp中提取）
void configure_moveit_params(const rclcpp::Node::SharedPtr& node);
void sync_robot_model_params(const rclcpp::Node::SharedPtr& node);

} // namespace mtc_tutorial 