#ifndef GRID_OVERLAY_H
#define GRID_OVERLAY_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

class GridOverlay {
public:
    GridOverlay(ros::NodeHandle& nh);

private:
    ros::Subscriber map_sub_;
    ros::Subscriber amcl_sub_;
    ros::Subscriber click_sub_;
    ros::Publisher markers_pub_;
    ros::Publisher nav_cmd_pub_;   // <-- NEW

    nav_msgs::OccupancyGrid::ConstPtr map_;
    geometry_msgs::Pose robot_pose_;

    double grid_step_;
    double origin_x_, origin_y_;
    bool got_pose_, got_map_, grid_ready_;

    geometry_msgs::Point goal_point_;
    bool has_goal_;
    std::vector<geometry_msgs::Point> path_;
    size_t path_index_;            // <-- NEW

    void amclCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void clickCb(const geometry_msgs::PointStamped::ConstPtr& msg);

    void computeGridOrigin();
    void publishMarkers();
    void snapToGrid(double wx, double wy, double& sx, double& sy);
    bool computePath(double sx, double sy, double gx, double gy, std::vector<geometry_msgs::Point>& path);
};

#endif
