#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <mtc_interface/action/execute_pour.hpp>  // 复用现有的Action接口

#include <moveit/task_constructor/task.h>
#include <mtc_tutorial/modular_task_builders.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace mtc = moveit::task_constructor;

using ExecutePour = mtc_interface::action::ExecutePour;
using GoalHandleEP = rclcpp_action::ServerGoalHandle<ExecutePour>;

class ModularTaskServer : public rclcpp::Node {
public:
    ModularTaskServer() : Node("modular_task_server") {
        // 创建Action服务器 - 使用不同的命名空间来避免与原服务器冲突
        server_ = rclcpp_action::create_server<ExecutePour>(
            this, "execute_modular_task",
            std::bind(&ModularTaskServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ModularTaskServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ModularTaskServer::handle_accepted, this, std::placeholders::_1));
        
        // 声明任务类型参数
        this->declare_parameter<std::string>("task_type", "pick");
        
        // 声明：抓取相关参数（可由客户端动态设置）
        this->declare_parameter<double>("pick.safe_approach_height", 0.10);
        this->declare_parameter<bool>("pick.use_back_constraint", true);
        this->declare_parameter<double>("pick.back_region_center_y", -0.6);
        this->declare_parameter<double>("pick.back_region_size_x", 2.0);
        this->declare_parameter<double>("pick.back_region_size_y", 1.2);
        this->declare_parameter<double>("pick.back_region_size_z", 2.0);
        
        RCLCPP_INFO(this->get_logger(), "🚀 模块化MTC任务服务器已启动");
        RCLCPP_INFO(this->get_logger(), "📋 支持的任务类型: pick, pour, place, return");
        RCLCPP_INFO(this->get_logger(), "🔧 Action接口: /execute_modular_task");
    }

private:
    rclcpp_action::Server<ExecutePour>::SharedPtr server_;
    bool cancel_requested_ = false;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecutePour::Goal> goal) {
        
        // 通过target_id来指定任务类型
        std::string task_type = goal->target_id.empty() ? "pour" : goal->target_id;
        
        RCLCPP_INFO(this->get_logger(), "收到模块化任务执行请求: 类型=%s", task_type.c_str());
        
        // 验证任务类型
        if (task_type != "pick" && task_type != "pour" && 
            task_type != "place" && task_type != "return") {
            RCLCPP_ERROR(this->get_logger(), "不支持的任务类型: %s", task_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleEP> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "收到取消任务请求");
        cancel_requested_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(std::shared_ptr<GoalHandleEP> goal_handle) {
        std::thread([this, goal_handle]() {
            auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<ExecutePour::Feedback>();
            auto result = std::make_shared<ExecutePour::Result>();
            
            cancel_requested_ = false;
            
            try {
                // 通过target_id来指定任务类型
                std::string task_type = goal->target_id.empty() ? "pour" : goal->target_id;
                
                RCLCPP_INFO(this->get_logger(), "开始执行模块化任务: %s", task_type.c_str());
                
                // 构建任务
                mtc::Task task;
                if (!build_modular_task(task_type, goal, task)) {
                    result->success = false;
                    result->error_msg = "任务构建失败";
                    goal_handle->abort(result);
                    return;
                }
                
                // 发布反馈：开始规划
                feedback->stage = "planning";
                feedback->progress = 0.1f;
                feedback->current_tilt_deg = 0.0f;
                goal_handle->publish_feedback(feedback);
                
                // 初始化和规划任务
                task.init();
                
                feedback->stage = "planning";
                feedback->progress = 0.3f;
                goal_handle->publish_feedback(feedback);
                
                if (!task.plan(4)) {
                    result->success = false;
                    result->error_msg = "任务规划失败";
                    goal_handle->abort(result);
                    return;
                }
                
                feedback->stage = "planned";
                feedback->progress = 0.5f;
                goal_handle->publish_feedback(feedback);
                
                // 检查是否仅规划
                if (goal->plan_only) {
                    result->success = true;
                    result->error_msg = "";
                    result->duration_sec = 0.0;
                    goal_handle->succeed(result);
                    return;
                }
                
                // 检查取消请求
                if (cancel_requested_) {
                    result->success = false;
                    result->error_msg = "任务在执行前被取消";
                    goal_handle->canceled(result);
                    return;
                }
                
                // 执行任务
                feedback->stage = "executing";
                feedback->progress = 0.7f;
                goal_handle->publish_feedback(feedback);
                
                auto start_time = std::chrono::steady_clock::now();
                const auto& solution = *task.solutions().front();
                auto exec_result = task.execute(solution);
                auto end_time = std::chrono::steady_clock::now();
                
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                result->duration_sec = duration.count() / 1000.0;
                
                // 检查执行结果
                if (exec_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    result->success = false;
                    result->error_msg = "任务执行失败，错误代码: " + std::to_string(exec_result.val);
                    goal_handle->abort(result);
                    return;
                }
                
                // 成功完成
                result->success = true;
                result->error_msg = "";
                feedback->stage = "completed";
                feedback->progress = 1.0f;
                goal_handle->publish_feedback(feedback);
                goal_handle->succeed(result);
                
                RCLCPP_INFO(this->get_logger(), "✅ %s任务执行完成，耗时: %.2fs", 
                           task_type.c_str(), result->duration_sec);
                
            } catch (const std::exception& e) {
                result->success = false;
                result->error_msg = std::string("任务执行异常: ") + e.what();
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "❌ 任务执行异常: %s", e.what());
            }
        }).detach();
    }
    
    bool build_modular_task(const std::string& task_type, 
                            std::shared_ptr<const ExecutePour::Goal> goal,
                            mtc::Task& task) {
        auto node = this->shared_from_this();
        
        try {
            if (task_type == "pick") {
                RCLCPP_INFO(this->get_logger(), "🔧 构建Pick任务...");
                mtc_tutorial::PickTaskParams pick_params;
                
                // 从ExecutePour Goal映射参数
                pick_params.approach_min = goal->approach_min;
                pick_params.approach_max = goal->approach_max;
                pick_params.lift_height = goal->lift_height;
                pick_params.plan_only = goal->plan_only;
                pick_params.object_id = "object";
                
                // 使用默认源位置（可以通过参数服务器设置）
                pick_params.source_x = 0.0;
                pick_params.source_y = -0.4;
                pick_params.source_z = 0.13;

                // 读取安全高度参数（支持客户端通过参数服务器设置）
                double safe_h = 0.08;
                this->get_parameter_or<double>("pick.safe_approach_height", safe_h, 0.08);
                pick_params.safe_approach_height = safe_h;
                
                task = mtc_tutorial::build_pick_task(node, pick_params);
                
            } else if (task_type == "pour") {
                RCLCPP_INFO(this->get_logger(), "🔧 构建Pour任务...");
                mtc_tutorial::PourOnlyTaskParams pour_params;
                
                pour_params.tilt_start_deg = goal->tilt_start_deg;
                pour_params.tilt_end_deg = goal->tilt_end_deg;
                pour_params.tilt_speed_deg_s = goal->tilt_speed_deg_s;
                pour_params.pour_hold_sec = goal->pour_hold_sec;
                pour_params.move_to_pour_min = 0.08;
                pour_params.move_to_pour_max = 0.15;
                pour_params.plan_only = goal->plan_only;
                
                task = mtc_tutorial::build_pour_only_task(node, pour_params);
                
            } else if (task_type == "place") {
                RCLCPP_INFO(this->get_logger(), "🔧 构建Place任务...");
                mtc_tutorial::PlaceTaskParams place_params;
                
                place_params.plan_only = goal->plan_only;
                place_params.object_id = "object";
                
                // 使用默认放置位置
                // place_params.target_pose = std::nullopt; // 使用默认
                
                task = mtc_tutorial::build_place_task(node, place_params);
                
            } else if (task_type == "return") {
                RCLCPP_INFO(this->get_logger(), "🔧 构建Return任务...");
                mtc_tutorial::ReturnTaskParams return_params;
                
                return_params.plan_only = goal->plan_only;
                return_params.timeout_sec = 15.0;
                
                task = mtc_tutorial::build_return_task(node, return_params);
                
            } else {
                RCLCPP_ERROR(this->get_logger(), "未知的任务类型: %s", task_type.c_str());
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "✅ %s任务构建成功: %s", 
                       task_type.c_str(), task.stages()->name().c_str());
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "❌ 构建%s任务时发生异常: %s", task_type.c_str(), e.what());
            return false;
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto server = std::make_shared<ModularTaskServer>();
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
} 