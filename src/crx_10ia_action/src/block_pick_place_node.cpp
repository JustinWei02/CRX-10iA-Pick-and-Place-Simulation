#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <random>
#include <cmath>

class BlockPickPlaceNode : public rclcpp::Node
{
public:
    BlockPickPlaceNode()
        : Node("block_pick_place_node"), goal_received_(false), generator_(std::random_device{}())
    {
        // Declare and retrieve parameters
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("robot_description_semantic", "");
        this->declare_parameter<std::string>("robot_description_kinematics", "");

        robot_description_ = this->get_parameter("robot_description").as_string();
        robot_description_semantic_ = this->get_parameter("robot_description_semantic").as_string();
        robot_description_kinematics_ = this->get_parameter("robot_description_kinematics").as_string();

        RCLCPP_INFO(this->get_logger(), "Robot description size: %zu", robot_description_.size());
        RCLCPP_INFO(this->get_logger(), "Semantic description: %s", robot_description_semantic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Kinematics config: %s", robot_description_kinematics_.c_str());

        if (robot_description_.empty() || robot_description_semantic_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Missing robot_description or robot_description_semantic parameters.");
            rclcpp::shutdown();
            return;
        }

        // Subscriber for block position
        block_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/block_position", 10, std::bind(&BlockPickPlaceNode::blockPositionCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Block Pick and Place Node Initialized");
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr block_position_subscriber_;

    std::string robot_description_;
    std::string robot_description_semantic_;
    std::string robot_description_kinematics_;
    std::mt19937 generator_;

    bool goal_received_;

    void blockPositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr block_position_msg)
    {
        if (goal_received_) {
            RCLCPP_INFO(this->get_logger(), "Goal already received, ignoring subsequent messages.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received block position: [%.2f, %.2f, %.2f]",
                    block_position_msg->point.x, block_position_msg->point.y, block_position_msg->point.z);

        // MoveIt interface for the arm and gripper
        auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
        auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

        // Configure tolerance
        arm_move_group.setPlanningTime(10.0);       
        arm_move_group.allowReplanning(true);       
        arm_move_group.setGoalPositionTolerance(0.001);      
        arm_move_group.setGoalOrientationTolerance(0.03);
        gripper_move_group.setGoalPositionTolerance(0.005); 

        // Plan and move to a position above the block
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = block_position_msg->point.x + 0.024; // Offset for block depth
        target_pose.position.y = block_position_msg->point.y;
        target_pose.position.z = block_position_msg->point.z + 0.124; // Offset above the block

        target_pose.orientation.x = 0.022838;
        target_pose.orientation.y = 0.70926;
        target_pose.orientation.z = -0.0209184;
        target_pose.orientation.w = 0.704269;

        arm_move_group.setStartStateToCurrentState();
        gripper_move_group.setStartStateToCurrentState();
        arm_move_group.setPoseTarget(target_pose);

        if (planAndExecute(arm_move_group)) {
            RCLCPP_INFO(this->get_logger(), "Successfully moved above the block position.");
            goal_received_ = true; 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to move above the block position.");
        }

        gripper_move_group.setJointValueTarget("joint_7", -0.011); // Close left finger
        gripper_move_group.setJointValueTarget("joint_8", 0.011);  // Close right finger

        if (planAndExecute(gripper_move_group)) {
            RCLCPP_INFO(this->get_logger(), "Gripper closed successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to close gripper.");
        }

        // Move up with the block
        target_pose.position.z += 0.3; // Move up by 0.3 meters
        arm_move_group.setPoseTarget(target_pose);

        if (planAndExecute(arm_move_group)) {
            RCLCPP_INFO(this->get_logger(), "Successfully moved up with the block.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to move up with the block.");
            return;
        }

        // Generate a random target position within the semi-annular workspace
        std::uniform_real_distribution<double> radius_distribution(0.45, 0.85);
        std::uniform_real_distribution<double> angle_distribution(0.0, 1.0 * M_PI);

        double radius = radius_distribution(generator_);
        double angle = angle_distribution(generator_);
        double target_x = radius * cos(angle);
        double target_y = radius * sin(angle);

        // Place the block at the random position on the ground
        target_pose.position.x = target_x;
        target_pose.position.y = target_y;
        target_pose.position.z = 0.144; // Height of the ground
        arm_move_group.setPoseTarget(target_pose);

        if (planAndExecute(arm_move_group)) {
            RCLCPP_INFO(this->get_logger(), "Successfully placed the block at a random position.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to place the block at a random position.");
            return;
        }

        // Open the gripper
        gripper_move_group.setJointValueTarget("joint_7", 0.0); 
        gripper_move_group.setJointValueTarget("joint_8", 0.0);
        if (planAndExecute(gripper_move_group)) {
            RCLCPP_INFO(this->get_logger(), "Gripper opened successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the gripper.");
            return;
        }

        // Move up with the block
        target_pose.position.z += 0.3; // Move up by 0.3 meters
        arm_move_group.setPoseTarget(target_pose);

        if (planAndExecute(arm_move_group)) {
            RCLCPP_INFO(this->get_logger(), "Successfully moved up with the block.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to move up with the block.");
            return;
        }

        // Return to home position
        arm_move_group.setNamedTarget("home_arm");
        if (planAndExecute(arm_move_group)) {
            RCLCPP_INFO(this->get_logger(), "Successfully returned to home position.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to return to home position.");
        }
    }

    

    bool planAndExecute(moveit::planning_interface::MoveGroupInterface &move_group)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        }
        return false;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlockPickPlaceNode>());
    rclcpp::shutdown();
    return 0;
}
