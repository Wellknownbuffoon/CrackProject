#include "robot_allign.h"
#include <cmath>
#include <algorithm>



ROBOTALLIGN::ROBOTALLIGN(ros::NodeHandle& nh)
: yaw_pid(1,0.0,0.0), state_(RobotState::IDLE), path_index_(0)
{
  cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  pose_sub = nh.subscribe("/amcl_pose", 10, &ROBOTALLIGN::poseCallback, this);
  
  imu_sub     = nh.subscribe("/imu", 10, &ROBOTALLIGN::imuCallback, this);
  
  control_loop_timer_ = nh.createTimer(ros::Duration(0.05), &ROBOTALLIGN::controlLoop, this);
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
    robot_pose_ = msg->pose.pose;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    static ros::Time last_update = ros::Time(0);
    if ((ros::Time::now() - last_update).toSec() > 0.1) { // smoother update
        current_yaw = 0.9 * current_yaw + 0.1 * yaw;
        last_update = ros::Time::now();
    }
}

void ROBOTALLIGN::controlLoop(const ros::TimerEvent& event){
    geometry_msgs::Twist cmd;
    if (path_.empty() || path_index_ >= path_.size()) {
        state_ = RobotState::IDLE;
    }

    if (state_ == RobotState::IDLE) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    } else {
        geometry_msgs::Point target = path_[path_index_];
        double dx = target.x - robot_pose_.position.x;
        double dy = target.y - robot_pose_.position.y;
        double dist = sqrt(dx*dx + dy*dy);
        double target_yaw = atan2(dy, dx);
        double yaw_error = normalizeAngle(target_yaw - current_yaw);

        switch(state_) {
            case RobotState::TURNING:
                cmd.linear.x = 0.0;
                cmd.angular.z = std::clamp(yaw_error * 1.0, -MAX_ANGLE_VEL, MAX_ANGLE_VEL);
                if (fabs(yaw_error) < 0.05) state_ = RobotState::MOVING; // done turning
                break;

            case RobotState::MOVING:
                cmd.angular.z = std::clamp(yaw_error*1.0, -MAX_ANGLE_VEL, MAX_ANGLE_VEL);
                cmd.linear.x  = std::clamp(dist*0.5, 0.0, MAX_LINEAR_VEL);

                if (dist < 0.02) { // reached waypoint
                    path_index_++;
                    if (path_index_ < path_.size()) state_ = RobotState::TURNING;
                    else state_ = RobotState::IDLE;
                }
                break;

            default:
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                break;
        }
    }

    cmd_pub.publish(cmd);
}

// Optional: function to set a path from outside
void ROBOTALLIGN::setPath(const std::vector<geometry_msgs::Point>& path) {
    path_ = path;
    path_index_ = 0;
    if (!path_.empty()) state_ = RobotState::TURNING;
}

