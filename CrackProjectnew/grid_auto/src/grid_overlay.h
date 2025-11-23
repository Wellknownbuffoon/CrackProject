#ifndef GRID_OVERLAY_H
#define GRID_OVERLAY_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <fstream>

// Forward declaration to avoid circular dependency
class ROBOTALLIGN; 

class GridOverlay {
public:
    GridOverlay(ros::NodeHandle& nh, ROBOTALLIGN* align);

    // Logging functions
    void initLog(const std::string &base_filename);
    void logPathData(double path_length, int num_turns);
    void logNavigationData(const std::string& event, double distance_to_goal = 0.0);
    void closeLog();
    double computePathLength(const std::vector<geometry_msgs::Point>& path);
    int computeNumTurns(const std::vector<geometry_msgs::Point>& path);

private:
    ros::Subscriber map_sub_;
    ros::Subscriber amcl_sub_;
    ros::Subscriber click_sub_;
    ros::Publisher markers_pub_;
    
    // Logs
    std::ofstream path_logfile_;
    std::ofstream navigation_logfile_;
    ros::Time start_time_;

    // Map & Robot State
    nav_msgs::OccupancyGrid::ConstPtr map_;
    geometry_msgs::Pose robot_pose_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

    // --- NEW FUNCTIONS ADDED HERE ---
    void createInteractiveMarker(double x, double y, int id);
    void processGoal(double gx_raw, double gy_raw); 
    // --------------------------------

    // Callbacks
    void amclCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void clickCb(const geometry_msgs::PointStamped::ConstPtr& msg);

    // Algorithms
    void computeGridOrigin();
    void snapToGrid(double wx, double wy, double& sx, double& sy);
    bool computePath(double sx, double sy, double gx, double gy, std::vector<geometry_msgs::Point>& path);
    void publishMarkers();

    // Variables
    double grid_step_;
    double origin_x_, origin_y_;
    bool got_pose_, got_map_, grid_ready_;

    geometry_msgs::Point goal_point_;
    bool has_goal_;
    std::vector<geometry_msgs::Point> path_;
    size_t path_index_;

    int num_turns_;
    double path_length_;
    int path_counter_;
    
    ROBOTALLIGN* robot_align_;
};

#endif
