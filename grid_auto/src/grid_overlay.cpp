#include "grid_overlay.h"
#include "robot_allign.h"  
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <interactive_markers/interactive_marker_server.h>


GridOverlay::GridOverlay(ros::NodeHandle& nh, ROBOTALLIGN* align) : robot_align_(align){
    map_sub_   = nh.subscribe("/map", 1, &GridOverlay::mapCb, this);
    local_map_sub_ = nh.subscribe("/move_base/local_costmap/costmap", 1, &GridOverlay::localMapCb, this);
    amcl_sub_  = nh.subscribe("/amcl_pose", 1, &GridOverlay::amclCb, this);
    click_sub_ = nh.subscribe("/clicked_point", 1, &GridOverlay::clickCb, this);
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("grid_points");


    markers_pub_  = nh.advertise<visualization_msgs::MarkerArray>("grid_overlay", 1);
    nav_cmd_pub_  = nh.advertise<geometry_msgs::Twist>("/nav_cmd_vel", 10);

    grid_step_ = 0.5;
    got_pose_ = got_map_ = grid_ready_ = false;
    has_goal_ = false;
    got_local_map_ = false;
    path_index_ = 0;

    ROS_INFO("GridOverlay initialized (click-to-goal enabled)");
}
void GridOverlay::localMapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    local_map_ = msg;
    got_local_map_ = true;
}


void GridOverlay::amclCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    robot_pose_ = msg->pose.pose;
    got_pose_ = true;

    if (got_map_ && !grid_ready_) computeGridOrigin();

    // navigation logic
    if (has_goal_ && !path_.empty()) {
        geometry_msgs::Point target = path_[path_index_];

        double dx = target.x - robot_pose_.position.x;
        double dy = target.y - robot_pose_.position.y;
        double dist = sqrt(dx*dx + dy*dy);
        double dist_to_final_goal = sqrt(pow(goal_point_.x - robot_pose_.position.x, 2) + 
                                       pow(goal_point_.y - robot_pose_.position.y, 2));
        logNavigationData("navigating", dist_to_final_goal);

        geometry_msgs::Twist nav_cmd;

        if (dist < 0.02) { // reached waypoint
            if (path_index_ + 1 < path_.size()) {
                path_index_++;
                ROS_INFO("Reached waypoint %zu/%zu", path_index_, path_.size());
            } else {
                ROS_INFO("Goal reached!");
                has_goal_ = false;
            }
        } else {
            nav_cmd.linear.x = 0.05; // move forward slowly
        }

        nav_cmd_pub_.publish(nav_cmd);
    }

    publishMarkers();
}

void GridOverlay::mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_ = msg;
    got_map_ = true;
    if (got_pose_ && !grid_ready_) computeGridOrigin();
    publishMarkers();
}

void GridOverlay::clickCb(const geometry_msgs::PointStamped::ConstPtr& msg) {
    if (!grid_ready_ || !map_) return;

    double gx, gy;
    snapToGrid(msg->point.x, msg->point.y, gx, gy);
    goal_point_.x = gx; goal_point_.y = gy; goal_point_.z = 0.05;

    double sx, sy;
    snapToGrid(robot_pose_.position.x, robot_pose_.position.y, sx, sy);

    if (computePath(sx, sy, gx, gy, path_)) {
        path_index_ = 0;
        has_goal_ = true;
        double path_length_ = computePathLength(path_);
        int num_turns_ = computeNumTurns(path_);
        logPathData(path_length_, num_turns_);
        double dist_to_goal = sqrt(pow(gx - robot_pose_.position.x, 2) + 
                              pow(gy - robot_pose_.position.y, 2));
        logNavigationData("path_planned", dist_to_goal);

        // Send path to ROBOTALLIGN
        if (robot_align_) {
            robot_align_->setPath(path_);
            ROS_INFO("Path sent to ROBOTALLIGN with %zu waypoints", path_.size());
        }
    } else {
        ROS_WARN("No path found");
        has_goal_ = false;
    }
    


    publishMarkers();
}

void GridOverlay::computeGridOrigin() {
    double rx = robot_pose_.position.x;
    double ry = robot_pose_.position.y;
    double i = std::round(rx / grid_step_);
    double j = std::round(ry / grid_step_);
    origin_x_ = rx - (i + 0.5) * grid_step_;
    origin_y_ = ry - (j + 0.5) * grid_step_;
    grid_ready_ = true;
}

void GridOverlay::snapToGrid(double wx, double wy, double& sx, double& sy) {
    double i = std::round((wx - origin_x_) / grid_step_ - 0.5);
    double j = std::round((wy - origin_y_) / grid_step_ - 0.5);
    sx = origin_x_ + (i + 0.5) * grid_step_;
    sy = origin_y_ + (j + 0.5) * grid_step_;
}

bool GridOverlay::computePath(double sx, double sy, double gx, double gy,
                              std::vector<geometry_msgs::Point>& path) {
    if (!got_local_map_) return false;
    auto costmap = local_map_;
    int map_w = costmap->info.width;
    int map_h = costmap->info.height;
    double map_x0 = costmap->info.origin.position.x;
    double map_y0 = costmap->info.origin.position.y;
    double map_res = costmap->info.resolution;

    auto worldToGrid = [&](double wx, double wy, int& i, int& j) {
        i = (int)std::round((wx - origin_x_) / grid_step_ - 0.5);
        j = (int)std::round((wy - origin_y_) / grid_step_ - 0.5);
    };

    auto gridToWorld = [&](int i, int j, double& wx, double& wy) {
        wx = origin_x_ + (i + 0.5) * grid_step_;
        wy = origin_y_ + (j + 0.5) * grid_step_;
    };

    auto isFree = [&](int i, int j) {
        double wx, wy; gridToWorld(i, j, wx, wy);
        int mx = (int)std::floor((wx - map_x0) / map_res);
        int my = (int)std::floor((wy - map_y0) / map_res);
        if (mx < 0 || mx >= map_w || my < 0 || my >= map_h) return false;
        int idx = my * map_w + mx;
        int cost = costmap->data[idx];
    if (cost < 0) return false;  // unknown is not free
    if (cost >= 50) return false; // occupied

    // Inflate: check neighbors around (mx,my)
    int inflation_radius = 1; // cells (tune depending on robot radius)
    for (int dx=-inflation_radius; dx<=inflation_radius; dx++) {
        for (int dy=-inflation_radius; dy<=inflation_radius; dy++) {
            int nx = mx + dx, ny = my + dy;
            if (nx < 0 || nx >= map_w || ny < 0 || ny >= map_h) continue;
            int nidx = ny * map_w + nx;
            if (costmap->data[nidx] >= 50) return false; // too close to obstacle
        }
    }

    return true;
    };

    int si, sj, gi, gj;
    worldToGrid(sx, sy, si, sj);
    worldToGrid(gx, gy, gi, gj);
    if (!isFree(si, sj) || !isFree(gi, gj)) return false;

    using KeyT = uint64_t;
    auto key = [](int i, int j)->KeyT {
        return ((KeyT)( (uint32_t)i ) << 32) | (uint32_t)j;
    };
    auto decode = [](KeyT k)->std::pair<int,int> {
        int i = (int)( (int32_t)(k >> 32) );
        int j = (int)( (int32_t)(k & 0xffffffff) );
        return {i,j};
    };

    std::priority_queue<std::tuple<double,int,int,double>,
                        std::vector<std::tuple<double,int,int,double>>,
                        std::greater<>> open;
    std::unordered_map<KeyT,double> gscore;
    std::unordered_map<KeyT,KeyT> came_from;

    auto heuristic = [&](int i, int j){ return fabs(i-gi)+fabs(j-gj); }; 

    gscore[key(si,sj)] = 0;
    open.emplace(heuristic(si,sj), si, sj, 0);

    const int di[4] = {1,-1,0,0};
    const int dj[4] = {0,0,1,-1};

    bool found = false;
    long long goalKey = 0;

    while (!open.empty()) {
        auto [f,i,j,g] = open.top(); open.pop();
        if (i==gi && j==gj) {found=true; goalKey=key(i,j); break;}
        for (int k=0;k<4;k++) {
            int ni=i+di[k], nj=j+dj[k];
            if (!isFree(ni,nj)) continue;
            long long nk=key(ni,nj);
            double ng=g+1;
            if (!gscore.count(nk)||ng<gscore[nk]) {
                gscore[nk]=ng;
                came_from[nk]=key(i,j);
                open.emplace(ng+heuristic(ni,nj),ni,nj,ng);
            }
        }
    }

    if (!found) return false;

    // reconstruct path
    path.clear();
    long long cur=goalKey;
    while (cur!=key(si,sj)) {
        int i=(int)(cur>>32), j=(int)(cur&0xffffffff);
        double wx,wy; gridToWorld(i,j,wx,wy);
        geometry_msgs::Point p; p.x=wx; p.y=wy; p.z=0;
        path.push_back(p);
        cur=came_from[cur];
    }
    std::reverse(path.begin(), path.end());

// Compress collinear waypoints (keep only corners + start/goal)
std::vector<geometry_msgs::Point> compressed;
if (!path.empty()) {
    compressed.push_back(path.front());
    int dx_prev = 0, dy_prev = 0;

    for (size_t i = 1; i < path.size(); i++) {
        int dx = (int)std::round(path[i].x - path[i-1].x);
        int dy = (int)std::round(path[i].y - path[i-1].y);

        if (dx != dx_prev || dy != dy_prev) {
            // direction changed → keep last point
            compressed.push_back(path[i-1]);
        }
        dx_prev = dx;
        dy_prev = dy;
    }

    compressed.push_back(path.back());
    path = compressed;
}

    return true;
}

void GridOverlay::publishMarkers() {
    visualization_msgs::MarkerArray ma;
    if (!map_ || !grid_ready_) return;



    // Green grid points
    int id=0;
    double map_x0=map_->info.origin.position.x;
    double map_y0=map_->info.origin.position.y;
    int map_w=map_->info.width;
    int map_h=map_->info.height;
    double map_res=map_->info.resolution;

    int min_i=(int)std::floor((map_x0-origin_x_)/grid_step_)-1;
    int max_i=(int)std::ceil((map_x0+map_w*map_res-origin_x_)/grid_step_)+1;
    int min_j=(int)std::floor((map_y0-origin_y_)/grid_step_)-1;
    int max_j=(int)std::ceil((map_y0+map_h*map_res-origin_y_)/grid_step_)+1;

    for (int j=min_j;j<=max_j;j++) {
        for (int i=min_i;i<=max_i;i++) {
            double wx=origin_x_+(i+0.5)*grid_step_;
            double wy=origin_y_+(j+0.5)*grid_step_;

            int mx=(int)std::floor((wx-map_x0)/map_res);
            int my=(int)std::floor((wy-map_y0)/map_res);
            if (mx<0||mx>=map_w||my<0||my>=map_h) continue;
            int idx=my*map_w+mx;
            if (map_->data[idx]!=0) continue;

            createInteractiveMarker(wx, wy, id++);

        }
    }
    server_->applyChanges();

    // Robot marker (red)
    visualization_msgs::Marker robot;
    robot.header.frame_id="map";
    robot.header.stamp=ros::Time::now();
    robot.ns="robot";
    robot.id=id++;
    robot.type=visualization_msgs::Marker::SPHERE;
    robot.pose.position=robot_pose_.position;
    robot.pose.position.z=0.05;
    robot.scale.x=0.08; robot.scale.y=0.08; robot.scale.z=0.08;
    robot.color.r=1.0; robot.color.a=1.0;
    ma.markers.push_back(robot);

    // Goal marker (blue) + path
    if (has_goal_) {
        visualization_msgs::Marker goal;
        goal.header.frame_id="map";
        goal.header.stamp=ros::Time::now();
        goal.ns="goal";
        goal.id=id++;
        goal.type=visualization_msgs::Marker::SPHERE;
        goal.pose.position=goal_point_;
        goal.scale.x=0.1; goal.scale.y=0.1; goal.scale.z=0.05;
        goal.color.b=1.0; goal.color.a=1.0;
        ma.markers.push_back(goal);

        visualization_msgs::Marker line;
        line.header.frame_id="map";
        line.header.stamp=ros::Time::now();
        line.ns="path";
        line.id=id++;
        line.type=visualization_msgs::Marker::LINE_STRIP;
        line.scale.x=0.02;
        line.color.b=1.0; line.color.a=1.0;
        for (auto& p: path_) line.points.push_back(p);
        ma.markers.push_back(line);
    }

    markers_pub_.publish(ma);
}
void GridOverlay::createInteractiveMarker(double x, double y, int id) {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.name = "marker_" + std::to_string(id);
    int_marker.scale = 0.2;
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = 0.05;

    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.05;
    marker.color.g = 1.0;
    marker.color.a = 1.0;
    control.markers.push_back(marker);

    int_marker.controls.push_back(control);

    server_->insert(int_marker, [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("Clicked marker: " << feedback->marker_name);

        double gx = feedback->pose.position.x;
        double gy = feedback->pose.position.y;

        double sx, sy;
        snapToGrid(robot_pose_.position.x, robot_pose_.position.y, sx, sy);

        if (computePath(sx, sy, gx, gy, path_)) {
            path_index_ = 0;
            has_goal_ = true;
            double path_length_ = computePathLength(path_);
            int num_turns_ = computeNumTurns(path_);
            logPathData(path_length_, num_turns_);
            double dist_to_goal = sqrt(pow(gx - robot_pose_.position.x, 2) + 
                              pow(gy - robot_pose_.position.y, 2));
            logNavigationData("path_planned", dist_to_goal);
            if (robot_align_){
               robot_align_->setPath(path_);
               robot_align_->startNavigation();
            }
            ROS_INFO("Path sent to ROBOTALLIGN with %zu waypoints", path_.size());
        } else {
            ROS_WARN("No path found");
            has_goal_ = false;
        }
    }); // properly close lambda
}
void GridOverlay::initLog(const std::string& base_filename) {
    // Create timestamp for unique filenames
    ros::Time now = ros::Time::now();
    std::string timestamp = std::to_string(now.sec);
    
    // Open path log file
    std::string path_filename = base_filename + "_path_" + timestamp + ".txt";
    path_logfile_.open(path_filename, std::ios::out);
    path_logfile_ << "timestamp,path_length,num_turns,waypoints,goal_x,goal_y\n";
    
    // Open navigation log file  
    std::string nav_filename = base_filename + "_navigation_" + timestamp + ".txt";
    navigation_logfile_.open(nav_filename, std::ios::out);
    navigation_logfile_ << "timestamp,event,distance_to_goal,robot_x,robot_y\n";
    
    start_time_ = now;
    path_counter_ = 0;
    num_turns_ = 0;
    path_length_ = 0.0;
    
    ROS_INFO("Log files initialized: %s, %s", path_filename.c_str(), nav_filename.c_str());
}

void GridOverlay::logPathData(double path_length, int num_turns) {
    double elapsed = (ros::Time::now() - start_time_).toSec();
    path_logfile_ << elapsed << "," 
                 << path_length << ","
                 << num_turns << ","
                 << path_.size() << ","
                 << goal_point_.x << ","
                 << goal_point_.y << "\n";
    path_logfile_.flush();
    
    ROS_INFO("Path logged: length=%.2fm, turns=%d, waypoints=%zu", 
             path_length, num_turns, path_.size());
}

void GridOverlay::logNavigationData(const std::string& event, double distance_to_goal) {
    double elapsed = (ros::Time::now() - start_time_).toSec();
    navigation_logfile_ << elapsed << ","
                       << event << ","
                       << distance_to_goal << ","
                       << robot_pose_.position.x << ","
                       << robot_pose_.position.y << "\n";
    navigation_logfile_.flush();
    
    ROS_DEBUG("Navigation event: %s, distance=%.2fm", event.c_str(), distance_to_goal);
}

void GridOverlay::closeLog() {
    if (path_logfile_.is_open()) {
        path_logfile_ << "# End of log\n";
        path_logfile_.close();
    }
    if (navigation_logfile_.is_open()) {
        navigation_logfile_ << "# End of log\n";
        navigation_logfile_.close();
    }
    ROS_INFO("Log files closed");
}

double GridOverlay::computePathLength(const std::vector<geometry_msgs::Point>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        len += std::sqrt(dx*dx + dy*dy);
    }
    return len;
}

int GridOverlay::computeNumTurns(const std::vector<geometry_msgs::Point>& path) {
    if (path.size() < 3) return 0;
    int turns = 0;
    for (size_t i = 2; i < path.size(); i++) {
        double dx1 = path[i-1].x - path[i-2].x;
        double dy1 = path[i-1].y - path[i-2].y;
        double dx2 = path[i].x   - path[i-1].x;
        double dy2 = path[i].y   - path[i-1].y;
        if ((dx1*dy2 - dy1*dx2) != 0) { // direction changed
            turns++;
        }
    }
    return turns;
}





