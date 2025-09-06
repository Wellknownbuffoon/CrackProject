#include "grid_overlay.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_overlay_node");
    ros::NodeHandle nh;
    GridOverlay grid(nh);
    ros::spin();
    return 0;
}
