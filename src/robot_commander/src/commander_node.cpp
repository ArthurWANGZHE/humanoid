#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <robot_interfaces/msg/pose_command.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = robot_interfaces::msg::PoseCommand; 

using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;

        right_arm_ = std::make_shared<MoveGroupInterface>(node_, "right_arm");
        left_arm_ = std::make_shared<MoveGroupInterface>(node_, "left_arm");
        right_gripper_ = std::make_shared<MoveGroupInterface>(node_, "right_gripper");
        left_gripper_ = std::make_shared<MoveGroupInterface>(node_, "left_gripper");
        neck_= std::make_shared<MoveGroupInterface>(node_, "neck");

        setScalingFactors(right_arm_, 1.0, 1.0);
        setScalingFactors(left_arm_, 1.0, 1.0);
        setScalingFactors(right_gripper_, 1.0, 1.0);
        setScalingFactors(left_gripper_, 1.0, 1.0);
        setScalingFactors(neck_, 1.0, 1.0);

        open_right_gripper_sub_ = node_->create_subscription<Bool>(
            "open_right_gripper", 10, std::bind(&Commander::openRightGripperCallback, this, _1));
        open_left_gripper_sub_ = node_->create_subscription<Bool>(
            "open_left_gripper", 10, std::bind(&Commander::openLeftGripperCallback, this, _1));
        right_joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "right_joint_command", 10, std::bind(&Commander::rightjointCmdCallback, this, _1));
        left_joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "left_joint_command", 10, std::bind(&Commander::leftjointCmdCallback, this, _1));
        right_pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "right_pose_command", 10, std::bind(&Commander::rightposeCmdCallback, this, _1));
        left_pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "left_pose_command", 10, std::bind(&Commander::leftposeCmdCallback, this, _1));
        neck_joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "neck_joint_command", 10, std::bind(&Commander::neckjointCmdCallback, this, _1));
    }

    void goToNamedTarget(const std::shared_ptr<MoveGroupInterface> &interfaces, const std::string &name)
    {
        interfaces->setStartStateToCurrentState();
        interfaces->setNamedTarget(name);
        planAndExecute(interfaces);
    }

    void goToJointTarget(const std::shared_ptr<MoveGroupInterface> &interfaces, const std::vector<double> &joints)
    {
        interfaces->setStartStateToCurrentState();
        interfaces->setJointValueTarget(joints);
        planAndExecute(interfaces);
    }

    void gotoPoseTargert(const std::shared_ptr<MoveGroupInterface> &arm, double x, double y, double z, 
                        double roll, double pitch, double yaw, bool cartesion_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped  target_pose;
        target_pose.header.frame_id = "link00";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x =q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm->setStartStateToCurrentState();
        
        if (!cartesion_path){
            arm->setPoseTarget(target_pose);
            planAndExecute(arm);
        }else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;
            double fraction = arm->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, true, &error_code);

            if(fraction == 1){
                arm->execute(trajectory);
            }else{
                RCLCPP_ERROR(node_->get_logger(), "笛卡尔路径规划失败，只完成了 %.2f%%", fraction * 100.0);
            }
        }
    }

    void gotoPoseTargert(const std::shared_ptr<MoveGroupInterface> &arm, double x, double y, double z, 
                        double roll, double pitch, double yaw, bool cartesion_path = false, bool relative = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";

        if(!relative){
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            target_pose.pose.orientation.x =q.getX();
            target_pose.pose.orientation.y = q.getY();
            target_pose.pose.orientation.z = q.getZ();
            target_pose.pose.orientation.w = q.getW();
        }else{
            target_pose.pose = arm->getCurrentPose().pose;
            target_pose.pose.position.x += x;
            target_pose.pose.position.y += y;
            target_pose.pose.position.z += z;
            target_pose.pose.orientation.x +=q.getX();
            target_pose.pose.orientation.y += q.getY();
            target_pose.pose.orientation.z += q.getZ();
            target_pose.pose.orientation.w += q.getW();
        }

        arm->setStartStateToCurrentState();
        
        if (!cartesion_path){
            arm->setPoseTarget(target_pose);
            planAndExecute(arm);
        }else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;
            double fraction = arm->computeCartesianPath(waypoints, 0.01, 0.0, trajectory, true, &error_code);

            if(fraction == 1){
                arm->execute(trajectory);
            }else{
                RCLCPP_ERROR(node_->get_logger(), "笛卡尔路径规划失败，只完成了 %.2f%%", fraction * 100.0);
            }
        }
    }

    void openGripper(const std::shared_ptr<MoveGroupInterface> &gripper_)
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("Gripper_open");
        planAndExecute(gripper_);
    }

    void closeGripper(const std::shared_ptr<MoveGroupInterface> &gripper_)
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("Gripper_closed");
        planAndExecute(gripper_);
    }
private:

    void setScalingFactors(const std::shared_ptr<MoveGroupInterface>& interfaces, float VelocityScalingFactor, float AccelerationScalingFactor)
    {
        interfaces->setMaxVelocityScalingFactor(VelocityScalingFactor);
        interfaces->setMaxAccelerationScalingFactor(AccelerationScalingFactor);
    }

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interfaces)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interfaces->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            interfaces->execute(plan);
        }
    }

    void openRightGripperCallback(const Bool &msg)
    {
        if (msg.data) {
            openGripper(right_gripper_);
        } else {
            closeGripper(right_gripper_);
        }
    }

    void openLeftGripperCallback(const Bool &msg)
    {
        if (msg.data) {
            openGripper(left_gripper_);
        } else {
            closeGripper(left_gripper_);
        }
    }

    void leftjointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;

        if(joints.size() == 6) {
            goToJointTarget(left_arm_, joints);
        }
    }

    void rightjointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;

        if(joints.size() == 6) {
            goToJointTarget(right_arm_, joints);
        }
    }

    void rightposeCmdCallback(const PoseCmd &msg)
    {
        gotoPoseTargert(right_arm_, msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, 
                    msg.cartesian_path, msg.relative);
    }

    void leftposeCmdCallback(const PoseCmd &msg)
    {
        gotoPoseTargert(left_arm_, msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, 
                    msg.cartesian_path, msg.relative);
    }

    void neckjointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;

        if(joints.size() == 2) {
            goToJointTarget(neck_, joints);
        }       
    }

private:

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> right_arm_;
    std::shared_ptr<MoveGroupInterface> left_arm_;
    std::shared_ptr<MoveGroupInterface> right_gripper_;
    std::shared_ptr<MoveGroupInterface> left_gripper_;
    std::shared_ptr<MoveGroupInterface> neck_;
    std::shared_ptr<MoveGroupInterface> dual_arm_;
    
    rclcpp::Subscription<Bool>::SharedPtr open_right_gripper_sub_;
    rclcpp::Subscription<Bool>::SharedPtr open_left_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr right_joint_cmd_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr left_joint_cmd_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr neck_joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr right_pose_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr left_pose_cmd_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Commander");
    auto commander = Commander(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}