#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>
#include <vector>
#include <map>
#include <cmath>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>


const double PI = 3.14159265359;
const std::string PLANNING_GROUP = "manipulator";  // 你的全局规划组名

double degreesToRadians(double degrees) {
    return degrees * PI / 180.0;
}

std::vector<double> degreesToRadians(const std::vector<double>& degrees_array) {
    std::vector<double> radians_array;
    for (double deg : degrees_array) {
        radians_array.push_back(degreesToRadians(deg));
    }
    return radians_array;
}

// 障碍物配置
struct ObstacleConfig {
    std::string id;
    geometry_msgs::msg::Pose pose;
    std::vector<double> dimensions;  // [X, Y, Z]
};


void addObstacle(
    moveit::planning_interface::PlanningSceneInterface& psi,
    std::string frame_id,
    const ObstacleConfig& config)
{
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.header.frame_id = frame_id;
    collision_obj.id = config.id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = config.dimensions[0];
    primitive.dimensions[1] = config.dimensions[1];
    primitive.dimensions[2] = config.dimensions[2];

    collision_obj.primitives.push_back(primitive);
    collision_obj.primitive_poses.push_back(config.pose);
    collision_obj.operation = collision_obj.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> objs;
    objs.push_back(collision_obj);
    psi.addCollisionObjects(objs);
}

void removeObstacle(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::string& obstacle_id)
{
    moveit_msgs::msg::CollisionObject collision_obj;
    collision_obj.id = obstacle_id;
    collision_obj.operation = collision_obj.REMOVE;
    
    std::vector<moveit_msgs::msg::CollisionObject> objs;
    objs.push_back(collision_obj);
    psi.addCollisionObjects(objs);
    RCLCPP_INFO(rclcpp::get_logger("rrt"), "障碍物 '%s' 已移除", obstacle_id.c_str());
}

void removeObstacles(
    moveit::planning_interface::PlanningSceneInterface& psi,
    const std::vector<std::string>& obstacle_ids)
{
    for (const auto& id : obstacle_ids) {
        removeObstacle(psi, id);
    }
}

void addObstacles(
    moveit::planning_interface::PlanningSceneInterface& psi,
    std::string frame_id,
    const std::vector<ObstacleConfig>& obstacles)
{
    for (const auto& obstacle : obstacles) {
        addObstacle(psi, frame_id, obstacle);
        RCLCPP_INFO(rclcpp::get_logger("rrt"), "障碍物 '%s' 已添加", obstacle.id.c_str());
    }
}

bool planAndExecute(
    moveit::planning_interface::MoveGroupInterface& move_group,
    rclcpp::Node::SharedPtr node,
    const std::vector<double>& target_joints,
    const std::string& target_name)
{
    RCLCPP_INFO(node->get_logger(), "设置目标点: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        target_joints[0], target_joints[1], target_joints[2],
        target_joints[3], target_joints[4], target_joints[5]);
    
    move_group.setJointValueTarget(target_joints);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    RCLCPP_INFO(node->get_logger(), "开始求解...");
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "✓ 规划成功 → %s", target_name.c_str());
        RCLCPP_INFO(node->get_logger(), ">> 原始轨迹点数: %lu (稀疏易卡顿)", my_plan.trajectory_.joint_trajectory.points.size());
        
        robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), PLANNING_GROUP);
        
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), my_plan.trajectory_);

        trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.01, 0.01);
        
        bool time_param_success = totg.computeTimeStamps(rt, 1.0, 0.2);
        
        if (time_param_success) {
            rt.getRobotTrajectoryMsg(my_plan.trajectory_);
            RCLCPP_INFO(node->get_logger(), ">> ✓ 轨迹已高频加密！新轨迹点数: %lu (平滑执行)", my_plan.trajectory_.joint_trajectory.points.size());
        } else {
            RCLCPP_WARN(node->get_logger(), "轨迹加密失败，将使用原始粗糙轨迹。");
        }
     
        RCLCPP_INFO(node->get_logger(), "开始执行轨迹...");
        moveit::core::MoveItErrorCode execute_result = move_group.execute(my_plan);
        
        if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "✓ 轨迹执行成功");
            return true;
        } else {
            RCLCPP_ERROR(node->get_logger(), "✗ 轨迹执行失败，错误码: %d", execute_result.val);
            return false;
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "✗ 规划失败 → %s", target_name.c_str());
        return false;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("rrt_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface psi;

    RCLCPP_INFO(node->get_logger(), "=== 工业机械臂高密度平滑避障规划测试 ===");

    std::vector<ObstacleConfig> obstacles;
    std::vector<std::string> old_obstacles = {"assembly_box", "left_support", "right_tool_stand"};
    // removeObstacles(psi, old_obstacles);
    rclcpp::sleep_for(std::chrono::seconds(1));

    ObstacleConfig obstacle1;
    obstacle1.id = "assembly_box";
    obstacle1.dimensions = {0.5, 0.5, 1.0};
    obstacle1.pose.orientation.w = 1.0;
    obstacle1.pose.position.x = 0.7;
    obstacle1.pose.position.y = 0.0;
    obstacle1.pose.position.z = 0.4;
    obstacles.push_back(obstacle1);

    ObstacleConfig obstacle2;
    obstacle2.id = "left_support";
    obstacle2.dimensions = {0.3, 0.2, 0.8};
    obstacle2.pose.orientation.w = 1.0;
    obstacle2.pose.position.x = 0.0;
    obstacle2.pose.position.y = 0.8;
    obstacle2.pose.position.z = 1.0;
    obstacles.push_back(obstacle2);

    ObstacleConfig obstacle3;
    obstacle3.id = "right_tool_stand";
    obstacle3.dimensions = {0.2, 0.3, 0.6};
    obstacle3.pose.orientation.w = 1.0;
    obstacle3.pose.position.x = -0.1;
    obstacle3.pose.position.y = -0.35;
    obstacle3.pose.position.z = 0.0;
    obstacles.push_back(obstacle3);

    addObstacles(psi, move_group.getPlanningFrame(), obstacles);
    rclcpp::sleep_for(std::chrono::seconds(1));

    move_group.setPlannerId("RRTstarkConfigDefault");
    move_group.setPlanningTime(5.0); 
    move_group.setNumPlanningAttempts(5);
  
    move_group.setMaxVelocityScalingFactor(1.0);  
    move_group.setMaxAccelerationScalingFactor(0.2); 
    
    std::vector<double> current_joints = move_group.getCurrentJointValues();
    RCLCPP_INFO(node->get_logger(), "✓ 当前关节状态获取成功");

    std::vector<double> start_position_degrees = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> start_position = degreesToRadians(start_position_degrees);
    
    std::vector<double> target_position_degrees = {60.0, 23.0, 91.0, 77.0, -63.0, 108.0};
    std::vector<double> target_position = degreesToRadians(target_position_degrees);

    RCLCPP_INFO(node->get_logger(), "\n[步骤 1/2] 机械臂回归初始位置...");
    if (!planAndExecute(move_group, node, start_position, "home")) {
        RCLCPP_WARN(node->get_logger(), "回归初始位置可能受阻，继续尝试目标点规划");
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    
    RCLCPP_INFO(node->get_logger(), "\n[步骤 2/2] 执行避障目标规划...");
    planAndExecute(move_group, node, target_position, "target");

    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::shutdown();
    return 0;
}