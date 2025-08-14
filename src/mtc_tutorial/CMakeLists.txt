cmake_minimum_required(VERSION 3.8)
project(mtc_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mtc_interface REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(mtc_tutorial src/mtc_tutorial.cpp)
ament_target_dependencies(mtc_tutorial 
  moveit_task_constructor_core 
  moveit_ros_planning_interface
  moveit_ros_planning
  rclcpp 
  tf2_eigen
  tf2_geometry_msgs
)


target_include_directories(mtc_tutorial PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(mtc_tutorial PUBLIC c_std_99 cxx_std_17)


install(TARGETS mtc_tutorial
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/check_moveit_config.py
  scripts/pour_client.py
  scripts/pour_tool.py
  DESTINATION lib/${PROJECT_NAME}
)
  

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# 新增可复用的任务构建库
add_library(pour_task_builder
  src/pour_task_builder.cpp
)
target_include_directories(pour_task_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pour_task_builder
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_ros_planning
  rclcpp
  tf2_eigen
  tf2_geometry_msgs
)

# 服务器可执行体链接到库
add_executable(execute_pour_server src/execute_pour_server.cpp)
ament_target_dependencies(execute_pour_server
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_ros_planning
  rclcpp
  rclcpp_action
  tf2_eigen
  tf2_geometry_msgs
  mtc_interface
)
target_include_directories(execute_pour_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(execute_pour_server pour_task_builder)

target_compile_features(execute_pour_server PUBLIC c_std_99 cxx_std_17)

install(TARGETS execute_pour_server pour_task_builder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)