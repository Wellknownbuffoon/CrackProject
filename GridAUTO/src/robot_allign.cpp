#include "robot_allign.h"
#include <cmath>

ROBOTALLIGN::ROBOTALLIGN(ros::NodeHandle& nh)
: yaw_pid(0.5,0.0,0.0)
{
  cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  pose_sub = nh.subscribe("/amcl_pose", 10, &ROBOTALLIGN::poseCallback, this);
  nav_cmd_sub = nh.subscribe("/nav_cmd_vel", 10, &ROBOTALLIGN::navCmdCallback, this);
  imu_sub     = nh.subscribe("/imu", 10, &ROBOTALLIGN::imuCallback, this);
  ROS_INFO("Robot alignment node started");
}
double ROBOTALLIGN::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void ROBOTALLIGN::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_yaw = yaw;  // update from IMU
}
void ROBOTALLIGN::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // Extract yaw from quaternion
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    static ros::Time last_update = ros::Time(0);
    if ((ros::Time::now() - last_update).toSec() > 2.0) {
        current_yaw = 0.9 * current_yaw + 0.1 * yaw;
        last_update = ros::Time::now();
    }
}
void ROBOTALLIGN::navCmdCallback(const geometry_msgs::Twist::ConstPtr& msg){
    geometry_msgs::Twist cmd = *msg;

    //orientations (0, 90, 180, 270 deg)
    double target_angles[4] = {0, M_PI/2, M_PI, -M_PI/2};
    double nearest = target_angles[0];
    double min_diff = fabs(normalizeAngle(nearest - current_yaw));
    
    for (int i = 1; i < 4; i++) {
        double diff = fabs(normalizeAngle(current_yaw - target_angles[i]));
        if (diff < min_diff) {
            min_diff = diff;
            nearest = target_angles[i];
        }
    }
    //compute error for PIDcontroller
    double error = normalizeAngle(nearest - current_yaw);
    double control = yaw_pid.compute(error, ros::Time::now());
    //limit angular velocity
    if(control > MAX_ANGLE_VEL) control = MAX_ANGLE_VEL;
    if (control < -MAX_ANGLE_VEL) control = -MAX_ANGLE_VEL;
    if (fabs(control) < 0.05 && fabs(error) > 0.05) {
        control = (control > 0 ? 0.05 : -0.05);
    }

    // Rotate if not aligned
    if (fabs(error) > 0.05) { // tolerance ~3 deg
        cmd.angular.z += control;
    } else {
        cmd.angular.z = 0.0; // stop rotation
        yaw_pid.reset();
    }

    cmd_pub.publish(cmd);
}

