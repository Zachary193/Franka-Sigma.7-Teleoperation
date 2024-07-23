#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <cmath>

geometry_msgs::Pose current_target_pose;
bool target_pose_updated = false;

float current_gripper_gap = 0.0;
bool gripper_gap_updated = false;

void targetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    current_target_pose = *msg;
    target_pose_updated = true;
}

void targetGripperGapCallback(const std_msgs::Float32::ConstPtr& msg) {
    current_gripper_gap = msg->data;
    gripper_gap_updated = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "franka_pose");
    ros::NodeHandle nh;

    ros::Subscriber pose_subscriber = nh.subscribe("target_pose", 10, targetPoseCallback);
    ros::Subscriber gripper_subscriber = nh.subscribe("target_gripper_gap", 10, targetGripperGapCallback);

    ros::Publisher end_effector_pose_publisher = nh.advertise<geometry_msgs::Pose>("current_end_effector_pose", 10);

    moveit::planning_interface::MoveGroupInterface arm_move_group("panda_arm");

    actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_client("franka_gripper/move", true);

    ROS_INFO("Waiting for action server to start...");
    gripper_client.waitForServer();
    ROS_INFO("Action server started.");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    geometry_msgs::Pose initial_pose;
    initial_pose.position.x = 0.4;
    initial_pose.position.y = 0;
    initial_pose.position.z = 0.4;
    initial_pose.orientation.w = 0;
    initial_pose.orientation.x = cos(M_PI / 8);
    initial_pose.orientation.y = -sin(M_PI / 8);
    initial_pose.orientation.z = 0;

    arm_move_group.setPoseTarget(initial_pose);

    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
    bool success = (arm_move_group.plan(initial_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        arm_move_group.execute(initial_plan);
        ROS_INFO("Initial position reached.");
    } else {
        ROS_WARN("Failed to reach initial position.");
    }

    ros::Rate rate(3000);

    while (ros::ok()) {
        if (target_pose_updated) {
            target_pose_updated = false;
        
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints.push_back(current_target_pose);

            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.01;
            double fraction = arm_move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            if (fraction > 0.6) {
                arm_move_group.execute(trajectory);
            }
        }

        if (gripper_gap_updated) {
            gripper_gap_updated = false;

            franka_gripper::MoveGoal goal;
            goal.width = current_gripper_gap;
            goal.speed = 0.2;

            gripper_client.sendGoal(goal);
            ROS_INFO("Gripper action goal sent.");
        }

        geometry_msgs::PoseStamped current_pose_stamped = arm_move_group.getCurrentPose();
        end_effector_pose_publisher.publish(current_pose_stamped.pose);

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}