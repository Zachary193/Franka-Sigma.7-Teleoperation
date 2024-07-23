#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_datatypes.h"
#include <dhdc.h>
#include <drdc.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

template<typename T>
constexpr const T& clamp(const T& a_val, const T& a_minVal, const T& a_maxVal) {
    return (a_val < a_minVal) ? a_minVal : (a_maxVal < a_val) ? a_maxVal : a_val;
}

// Bezier curve interpolation
geometry_msgs::Pose bezierInterpolation(const geometry_msgs::Pose& start, const geometry_msgs::Pose& control1, const geometry_msgs::Pose& control2, const geometry_msgs::Pose& end, double t) {
    geometry_msgs::Pose result;

    result.position.x = pow(1-t, 3) * start.position.x + 3 * pow(1-t, 2) * t * control1.position.x + 3 * (1-t) * pow(t, 2) * control2.position.x + pow(t, 3) * end.position.x;
    result.position.y = pow(1-t, 3) * start.position.y + 3 * pow(1-t, 2) * t * control1.position.y + 3 * (1-t) * pow(t, 2) * control2.position.y + pow(t, 3) * end.position.y;
    result.position.z = pow(1-t, 3) * start.position.z + 3 * pow(1-t, 2) * t * control1.position.z + 3 * (1-t) * pow(t, 2) * control2.position.z + pow(t, 3) * end.position.z;

    // Spherical linear interpolation)
    tf2::Quaternion q_start(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
    tf2::Quaternion q_control1(control1.orientation.x, control1.orientation.y, control1.orientation.z, control1.orientation.w);
    tf2::Quaternion q_control2(control2.orientation.x, control2.orientation.y, control2.orientation.z, control2.orientation.w);
    tf2::Quaternion q_end(end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w);

    tf2::Quaternion q_interpolated_1 = q_start.slerp(q_control1, t);
    tf2::Quaternion q_interpolated_2 = q_control1.slerp(q_control2, t);
    tf2::Quaternion q_interpolated_3 = q_control2.slerp(q_end, t);
    tf2::Quaternion q_combined_1 = q_interpolated_1.slerp(q_interpolated_2, t);
    tf2::Quaternion q_combined = q_combined_1.slerp(q_interpolated_3, t);

    result.orientation.x = q_combined.getX();
    result.orientation.y = q_combined.getY();
    result.orientation.z = q_combined.getZ();
    result.orientation.w = q_combined.getW();

    return result;
}

int moveToCenter(double a_moveBase, double a_moveWrist, double a_moveGripper) {
    if (drdRegulatePos(a_moveBase) < 0) {
        std::cout << "error: failed to set base regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    if (dhdHasActiveWrist() && drdRegulateRot(a_moveWrist) < 0) {
        std::cout << "error: failed to set wrist regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    if (dhdHasActiveGripper() && drdRegulateGrip(a_moveGripper) < 0) {
        std::cout << "error: failed to set gripper regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    if (drdStart() < 0) {
        std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    double positionCenter[DHD_MAX_DOF] = {};
    if (drdMoveTo(positionCenter, true) < 0) {
        std::cout << "error: failed to move the device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    if (drdStop(true) < 0) {
        std::cout << "error: failed to stop robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    return 0;
}

void CheckAvailableDevices(int &devs) {
    ros::Rate r(0.5);
    while (ros::ok() && devs == 0) {
        for (int i = 0; i < 2; i++) {
            if (drdOpenID((char) i) > -1)
                devs = i + 1;
        }
        r.sleep();
        ROS_INFO("Looking for connected devices...");
    }

    ROS_INFO("Found %d Device(s)", devs);
}

class SigmaDevice {
public:
    SigmaDevice(ros::NodeHandle& nh) : nh_(nh), k_0(0.2), alpha(0.05), current_gripper_gap(0.0) {
        id++;
        
        pub_target_pose = nh_.advertise<geometry_msgs::Pose>("target_pose", 10);
        pub_target_gripper_gap = nh_.advertise<std_msgs::Float32>("target_gripper_gap", 10);
        sub_current_end_effector_pose = nh_.subscribe("current_end_effector_pose", 10, &SigmaDevice::EndEffectorPoseCallback, this);
        sub_gripper_joint_states = nh_.subscribe("/franka_gripper/joint_states", 10, &SigmaDevice::GripperJointStatesCallback, this);
        sub_franka_feedback = nh_.subscribe("/franka_state_controller/F_ext", 10, &SigmaDevice::WrenchCallback, this);

        if (CalibrateDevice() == -1)
            ros::shutdown();
        moveToCenter(true, true, true);

        // Initialize Current Pose
        double p[7];
        dhdGetPositionAndOrientationRad(&p[0], &p[1], &p[2], &p[3], &p[4], &p[5]);
        current_pose.position.x = p[0];
        current_pose.position.y = p[1];
        current_pose.position.z = p[2];
        tf::Quaternion q;
        q.setRPY(M_PI + p[3], p[4], -M_PI / 4 + p[5]);
        current_pose.orientation.w = q.getW();
        current_pose.orientation.x = q.getX();
        current_pose.orientation.y = q.getY();
        current_pose.orientation.z = q.getZ();
        dhdGetGripperAngleDeg(&p[6]);
        gripper_angle.data = p[6];

        previous_target_pose.position.x = 0.0;
        previous_target_pose.position.y = 0.0;
        previous_target_pose.position.z = 0.0;
        previous_target_pose.orientation.w = 1.0;
        previous_target_pose.orientation.x = 0.0;
        previous_target_pose.orientation.y = 0.0;
        previous_target_pose.orientation.z = 0.0;
    }

    // Haptic data acquisition
    int ReadMeasurementsFromDevice() {
        double px = 0, py = 0, pz = 0, ra = 0, rb = 0, rg = 0;
        dhdGetPositionAndOrientationRad(&px, &py, &pz, &ra, &rb, &rg);
        current_pose.position.x = px;
        current_pose.position.y = py;
        current_pose.position.z = pz;

        // Rotate 180 degrees along the x-axis
        tf2::Quaternion x_rotation;
        x_rotation.setRPY(M_PI + ra, rb, rg);

        // Rotate 45 degrees along the z-axis
        tf2::Quaternion z_rotation;
        z_rotation.setRPY(0, 0, M_PI/4);

        tf2::Quaternion combined_orientation = x_rotation * z_rotation;
        combined_orientation.normalize();

        current_pose.orientation.w = combined_orientation.getW();
        current_pose.orientation.x = combined_orientation.getX();
        current_pose.orientation.y = combined_orientation.getY();
        current_pose.orientation.z = combined_orientation.getZ();

        // Gripper
        double temp;
        dhdGetGripperAngleRad(&temp);
        gripper_angle.data = (float)temp;

        ROS_INFO("Sigma7:");
        ROS_INFO("Current Position: (x: %.2f, y: %.2f, z: %.2f)", px, py, pz);
        ROS_INFO("Current Orientation: (roll: %.2f, pitch: %.2f, yaw: %.2f)", ra, rb, rg);
        ROS_INFO("Current Gripper Angle: %.2f", gripper_angle.data);

        return 0;
    }

    void WrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
        latest_wrench = *msg;
        new_wrench_msg = true;
    }

    void EndEffectorPoseCallback(const geometry_msgs::Pose::ConstPtr &msg) {
        current_end_effector_pose = *msg;
    }

    void GripperJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
        if (msg->name.size() > 0 && msg->name[0] == "panda_finger_joint1") {
            current_gripper_gap = msg->position[0];
        }
    }

    void PublishTargetPose()
    {
        geometry_msgs::Pose target_pose_msg;

        // Workspace transform
        target_pose_msg.position.x = 0.4 + 3 * current_pose.position.x;
        target_pose_msg.position.y = 0 + 3 * current_pose.position.y;
        target_pose_msg.position.z = 0.4 + 3 * current_pose.position.z;

        target_pose_msg.orientation = current_pose.orientation;

        // Updated target pose
        target_pose_msg.position.x = -k_0 * (current_end_effector_pose.position.x - target_pose_msg.position.x) + target_pose_msg.position.x;
        target_pose_msg.position.y = -k_0 * (current_end_effector_pose.position.y - target_pose_msg.position.y) + target_pose_msg.position.y;
        target_pose_msg.position.z = -k_0 * (current_end_effector_pose.position.z - target_pose_msg.position.z) + target_pose_msg.position.z;

        // Control points
        geometry_msgs::Pose control_pose1;
        control_pose1.position.x = previous_target_pose.position.x + 0.5 * (target_pose_msg.position.x - previous_target_pose.position.x);
        control_pose1.position.y = previous_target_pose.position.y + 0.5 * (target_pose_msg.position.y - previous_target_pose.position.y);
        control_pose1.position.z = previous_target_pose.position.z + 0.5 * (target_pose_msg.position.z - previous_target_pose.position.z);
        control_pose1.orientation = current_pose.orientation;

        geometry_msgs::Pose control_pose2;
        control_pose2.position.x = target_pose_msg.position.x - 0.5 * (target_pose_msg.position.x - previous_target_pose.position.x);
        control_pose2.position.y = target_pose_msg.position.y - 0.5 * (target_pose_msg.position.y - previous_target_pose.position.y);
        control_pose2.position.z = target_pose_msg.position.z - 0.5 * (target_pose_msg.position.z - previous_target_pose.position.z);
        control_pose2.orientation = current_pose.orientation;

        target_pose_msg = bezierInterpolation(previous_target_pose, control_pose1, control_pose2, target_pose_msg, alpha);
        previous_target_pose = target_pose_msg;

        ROS_INFO("Franka:");
        ROS_INFO("Current Position: (x: %.2f, y: %.2f, z: %.2f)", current_end_effector_pose.position.x, current_end_effector_pose.position.y, current_end_effector_pose.position.z);
        ROS_INFO("Current Orientation: (w: %.2f, x: %.2f, y: %.2f, z: %.2f)", current_end_effector_pose.orientation.w, current_end_effector_pose.orientation.x, current_end_effector_pose.orientation.y, current_end_effector_pose.orientation.z);
        ROS_INFO("Target Position: (x: %.2f, y: %.2f, z: %.2f)", target_pose_msg.position.x, target_pose_msg.position.y, target_pose_msg.position.z);
        ROS_INFO("Target Orientation: (w: %.2f, x: %.2f, y: %.2f, z: %.2f)", target_pose_msg.orientation.w, target_pose_msg.orientation.x, target_pose_msg.orientation.y, target_pose_msg.orientation.z);

        pub_target_pose.publish(target_pose_msg);
    }

    void PublishTargetGripperGap() {
        std_msgs::Float32 target_gripper_msg;
        target_gripper_msg.data = gripper_angle.data * 0.080 / 0.51;

        if (target_gripper_msg.data < current_gripper_gap) {
            target_gripper_msg.data = current_gripper_gap;
        }

        ROS_INFO("Target Gripper Gap: %.2f", target_gripper_msg.data);

        pub_target_gripper_gap.publish(target_gripper_msg);
    }

public:
    geometry_msgs::WrenchStamped latest_wrench;
    bool new_wrench_msg = false;

private:
    ros::NodeHandle nh_;
    int id = -1;

    geometry_msgs::Pose current_pose;
    std_msgs::Float32 gripper_angle;
    double k_0;
    double alpha;

    ros::Subscriber sub_franka_feedback;
    ros::Subscriber sub_current_end_effector_pose;
    ros::Subscriber sub_gripper_joint_states;

    ros::Publisher pub_target_pose;
    ros::Publisher pub_target_gripper_gap;

    geometry_msgs::Pose current_end_effector_pose;
    geometry_msgs::Pose previous_target_pose;

    double current_gripper_gap;

    // CalibrateDevice
    int CalibrateDevice() {
        ROS_INFO("Calibrating device %i ...", id);
        if (drdOpenID((char)id) < 0) {
            ROS_ERROR("No device %i found. dhd says: %s", id, dhdErrorGetLastStr());
            dhdSleep(2.0);
            drdClose((char)id);
            return -1;
        }

        if (drdIsInitialized((char)id)) {
            ROS_INFO("Device %i is already calibrated.", id);
        } else if (drdAutoInit((char)id) < 0) {
            ROS_ERROR("Initialization of device %i failed. dhd says: (%s)", id, dhdErrorGetLastStr());
            dhdSleep(2.0);
        }

        double nullPose[DHD_MAX_DOF] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        drdMoveTo(nullPose);
        drdStop(true, (char)id);
        dhdEnableForce(DHD_ON, (char)id);
        dhdSleep(1);

        ROS_INFO("Device %i ready.", id);
        return 0;
    }
};

// Center position maintenance and haptic feedback
int holdOnCenter(SigmaDevice* sigma_device) {
    dhdEnableExpertMode();
    double positionCenter[DHD_MAX_DOF] = {};
    drdMoveTo(positionCenter);

    std::cout << "press 'q' to quit" << std::endl << std::endl;

    double lastDisplayUpdateTime = dhdGetTime();

    double px = 0;
    double py = 0;
    double pz = 0;
    double ra = 0;
    double rb = 0;
    double rg = 1.60;
    double pg = 0.51;

    double jointAnglesVelocity[DHD_MAX_DOF] = {};
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double wa = 0.0;
    double wb = 0.0;
    double wg = 0.0;
    double vg = 0.0;

    double pxHold = 0;
    double pyHold = 0;
    double pzHold = 0;
    double raHold = 0;
    double rbHold = 0;
    double rgHold = 1.60;
    double pgHold = 0.51;

    double fx = 0.0;
    double fy = 0.0;
    double fz = 0.0;
    double qa = 0.0;
    double qb = 0.0;
    double qg = 0.0;
    double fg = 0.0;

    constexpr double filter_constant = 0.03;

    while (ros::ok()) {
        ros::spinOnce();

        double p[7];
        dhdGetPositionAndOrientationRad(&p[0], &p[1], &p[2], &p[3], &p[4], &p[5]);
        sigma_device->ReadMeasurementsFromDevice();
        sigma_device->PublishTargetPose();
        sigma_device->PublishTargetGripperGap();

        if (dhdGetPositionAndOrientationDeg(&px, &py, &pz, &ra, &rb, &rg) < 0) {
            std::cout << "error: failed to retrieve device position (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        if (dhdGetGripperGap(&pg) < 0) {
            std::cout << "error: failed to retrieve device gripper opening (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        if (dhdGetLinearVelocity(&vx, &vy, &vz) < 0) {
            std::cout << "error: failed to retrieve linear velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        if (dhdGetGripperLinearVelocity(&vg) < 0) {
            std::cout << "error: failed to retrieve gripper velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        if (dhdGetJointVelocities(jointAnglesVelocity) < 0) {
            std::cout << "error: failed to retrieve angular velocity (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }
        wa = jointAnglesVelocity[3];
        wb = jointAnglesVelocity[4];
        wg = jointAnglesVelocity[5];

        constexpr double Kp = 100.0;
        constexpr double Kv = 20.0;
        constexpr double MaxForce = 2.0;

        constexpr double Kr = 0.01;
        constexpr double Kw = 0.04;
        constexpr double MaxTorque = 0.02;

        constexpr double Kgp = 100.0;
        constexpr double Kgv = 5.0;
        constexpr double MaxGripperForce = 1.0;

        double external_fx = sigma_device->latest_wrench.wrench.force.x;
        double external_fy = sigma_device->latest_wrench.wrench.force.y;
        double external_fz = sigma_device->latest_wrench.wrench.force.z;
        double external_torque_x = sigma_device->latest_wrench.wrench.torque.x;
        double external_torque_y = sigma_device->latest_wrench.wrench.torque.y;
        double external_torque_z = sigma_device->latest_wrench.wrench.torque.z;

        if (std::abs(external_fx) < 10 && std::abs(external_fy) < 10 && std::abs(external_fz) < 10 &&
            std::abs(external_torque_x) < 100 && std::abs(external_torque_y) < 100 && std::abs(external_torque_z) < 100) {
            fx = Kp * (pxHold - px) - Kv * vx;
            fy = Kp * (pyHold - py) - Kv * vy;
            fz = Kp * (pzHold - pz) - Kv * vz;

            double forceMagnitude = std::sqrt(fx * fx + fy * fy + fz * fz);
            if (forceMagnitude > MaxForce) {
                double forceRatio = MaxForce / forceMagnitude;
                fx *= forceRatio;
                fy *= forceRatio;
                fz *= forceRatio;
            }

            qa = clamp(Kr * (raHold - ra) - Kw * wa, -MaxTorque, MaxTorque);
            qb = clamp(Kr * (rbHold - rb) - Kw * wb, -MaxTorque, MaxTorque);
            qg = clamp(Kr * (rgHold - rg) - Kw * wg, -MaxTorque, MaxTorque);

            fg = clamp(Kgp * (pgHold - pg) - Kgv * vg, -MaxGripperForce, MaxGripperForce);
        } else {
            fx += filter_constant * (-external_fx - fx);
            fy += filter_constant * (external_fy - fy);
            fz += filter_constant * (external_fz - fz);

            qa = clamp(Kr * (raHold - ra) - Kw * wa, -MaxTorque, MaxTorque);
            qb = clamp(Kr * (rbHold - rb) - Kw * wb, -MaxTorque, MaxTorque);
            qg = clamp(Kr * (rgHold - rg) - Kw * wg, -MaxTorque, MaxTorque);

            fg = 0.0;
        }

        if (dhdSetForceAndWristJointTorquesAndGripperForce(fx, fy, fz, qa, qb, qg, fg) < 0) {
            std::cout << "error: failed to apply forces (" << dhdErrorGetLastStr() << ")" << std::endl;
            dhdSleep(2.0);
            break;
        }

        double time = dhdGetTime();
        if (time - lastDisplayUpdateTime > 0.1) {
            lastDisplayUpdateTime = time;
            std::cout.flush();
        }

        if (dhdKbHit() && dhdKbGet() == 'q') {
            std::cout << std::endl << std::endl << "exiting at user's request" << std::endl;
            break;
        }
    }

    if (drdClose() < 0) {
        std::cout << "error: failed to close the connection (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }
    return 0;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "sigma7");
    ros::NodeHandle nh;

    if (drdOpen() < 0) {
        std::cout << "error: failed to open device (" << dhdErrorGetLastStr() << ")" << std::endl;
        dhdSleep(2.0);
        return -1;
    }

    int devs = 0;
    CheckAvailableDevices(devs);

    SigmaDevice sigma(nh);

    double rate;
    nh.param("frequency", rate, 3000.0);
    ROS_INFO("Set frequency: %f", rate);
    ros::Rate loop_rate(rate);

    holdOnCenter(&sigma);
    ROS_INFO("Initialization done.");

    while (ros::ok()) {
        sigma.ReadMeasurementsFromDevice();
        sigma.PublishTargetPose();
        sigma.PublishTargetGripperGap();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}
