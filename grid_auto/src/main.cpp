#include <ros/ros.h>
#include "robot_allign.h"
#include "grid_overlay.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_system");
    ros::NodeHandle nh;

    ROBOTALLIGN robot_align(nh);      // alignment module
    GridOverlay grid(nh,&robot_align);      // grid visualization module

    ROS_INFO("Grid system started: alignment + overlay running");
    ros::spin();
    return 0;
}
