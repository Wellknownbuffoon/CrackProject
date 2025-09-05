#ifndef ROBOT_ALLIGN_H
#define ROBOT_ALLIGN_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include "PIDcontroller.h"

class ROBOTALLIGN{
    public:
      ROBOTALLIGN(ros::NodeHandle& nh);
      void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
      void navCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
      void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
      double normalizeAngle(double angle);
    private:
      ros::Publisher cmd_pub;
      ros::Subscriber pose_sub;
      ros::Subscriber nav_cmd_sub;
      ros::Subscriber imu_sub; 
      PIDController yaw_pid;
      double current_yaw = 0.0;

      const double MAX_LINEAR_VEL = 0.2;
      const double MAX_ANGLE_VEL  = 1.5;

      const double LIN_VEL_STEP_SIZE = 0.01;
      const double ANG_VEL_STEP_SIZE = 0.1;
};

#endif