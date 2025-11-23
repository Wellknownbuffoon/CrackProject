#ifndef ROBOT_ALLIGN_H
#define ROBOT_ALLIGN_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include "PIDcontroller.h"
enum class RobotState { IDLE, TURNING, MOVING };

class ROBOTALLIGN{
    public:
      ROBOTALLIGN(ros::NodeHandle& nh);
      void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
      void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
      double normalizeAngle(double angle);
      void setPath(const std::vector<geometry_msgs::Point>& path);
      
      void startNavigation() { 
        if (!path_.empty()) state_ = RobotState::TURNING; 
      }

    private:
      void controlLoop(const ros::TimerEvent& event);
      ros::Publisher cmd_pub;
      ros::Timer control_loop_timer_;
      ros::Subscriber pose_sub;
      ros::Subscriber imu_sub; 
      PIDController yaw_pid;
      RobotState state_;                       // missing
      int path_index_;                          // missing
      std::vector<geometry_msgs::Point> path_; // missing
      geometry_msgs::Pose robot_pose_; 
      double current_yaw = 0.0;

      const double MAX_LINEAR_VEL = 0.2;
      const double MAX_ANGLE_VEL  = 1.0;

      const double LIN_VEL_STEP_SIZE = 0.01;
      const double ANG_VEL_STEP_SIZE = 0.1;
};

#endif
