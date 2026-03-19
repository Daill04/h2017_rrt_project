#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <thread>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("basic_motion_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

    RCLCPP_INFO(node->get_logger(), "=== 开始执行基础目标点运动 (Basic Motion) ===");

    std::vector<double> target_joints = {1.57, 0.52, 1.04, 0.0, 1.57, 0.0};
    move_group.setJointValueTarget(target_joints);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(node->get_logger(), "正在请求底层算法规划路径...");
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "✓ 规划成功！正在进行轨迹平滑处理...");
        
        robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), "manipulator");
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), my_plan.trajectory_);
        
        trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.01, 0.01);

        if (totg.computeTimeStamps(rt, 0.8, 0.2)) {
            rt.getRobotTrajectoryMsg(my_plan.trajectory_);
            RCLCPP_INFO(node->get_logger(), "平滑处理完成，准备发送给底层驱动...");
        }

        move_group.execute(my_plan);
        RCLCPP_INFO(node->get_logger(), "机械臂已成功平滑到达目标位置！");
        
    } else {
        RCLCPP_ERROR(node->get_logger(), "规划失败！");
    }

    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::shutdown();
    return 0;
}