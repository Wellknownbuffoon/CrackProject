#include "grid_overlay.h"
#include "robot_allign.h"  
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <interactive_markers/interactive_marker_server.h>

GridOverlay::GridOverlay(ros::NodeHandle& nh, ROBOTALLIGN* align) : robot_align_(align){
    map_sub_   = nh.subscribe("/map", 1, &GridOverlay::mapCb, this);
    amcl_sub_  = nh.subscribe("/amcl_pose", 1, &GridOverlay::amclCb, this);
    // We keep this for standard RViz clicks too
    click_sub_ = nh.subscribe("/clicked_point", 1, &GridOverlay::clickCb, this);
    
    // Initialize the Marker Server
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("grid_points");

    markers_pub_  = nh.advertise<visualization_msgs::MarkerArray>("grid_overlay", 1);

    grid_step_ = 0.2; // 20cm grid
    got_pose_ = got_map_ = grid_ready_ = false;
    has_goal_ = false;
    path_index_ = 0;

    ROS_INFO("GridOverlay initialized (Interactive Markers Enabled)");
}

void GridOverlay::amclCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    robot_pose_ = msg->pose.pose;
    got_pose_ = true;

    if (got_map_ && !grid_ready_) computeGridOrigin();

    publishMarkers();
}

void GridOverlay::mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_ = msg;
    got_map_ = true;
    if (got_pose_ && !grid_ready_) computeGridOrigin();
    publishMarkers();
}

void GridOverlay::clickCb(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // This function handles standard RViz "Publish Point" tool
    processGoal(msg->point.x, msg->point.y);
}

// Helper to process a goal (from click or interactive marker)
void GridOverlay::processGoal(double gx_raw, double gy_raw) {
    if (!grid_ready_ || !map_) return;

    double gx, gy;
    snapToGrid(gx_raw, gy_raw, gx, gy);
    goal_point_.x = gx; goal_point_.y = gy; goal_point_.z = 0.05;

    double sx, sy;
    snapToGrid(robot_pose_.position.x, robot_pose_.position.y, sx, sy);

    if (computePath(sx, sy, gx, gy, path_)) {
        path_index_ = 0;
        has_goal_ = true;
        
        // Logs
        double path_length_ = computePathLength(path_);
        int num_turns_ = computeNumTurns(path_);
        logPathData(path_length_, num_turns_);
        double dist_to_goal = sqrt(pow(gx - robot_pose_.position.x, 2) + 
                                   pow(gy - robot_pose_.position.y, 2));
        logNavigationData("path_planned", dist_to_goal);

        // Send to Robot
        if (robot_align_) {
            robot_align_->setPath(path_);
            ROS_INFO("Path sent to ROBOTALLIGN.");
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

// --- RESTORED MARKER CREATION ---
void GridOverlay::createInteractiveMarker(double x, double y, int id) {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.name = "m_" + std::to_string(id); // Unique name
    int_marker.scale = 0.15; // Slightly smaller than grid step
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = 0.01;

    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.02;
    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Green
    marker.color.a = 0.6; // Semi-transparent
    control.markers.push_back(marker);

    int_marker.controls.push_back(control);

    // Callback when marker is clicked
    server_->insert(int_marker, [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO("Interactive Marker Clicked at %.2f, %.2f", feedback->pose.position.x, feedback->pose.position.y);
        processGoal(feedback->pose.position.x, feedback->pose.position.y);
    });
}

void GridOverlay::publishMarkers() {
    visualization_msgs::MarkerArray ma;
    if (!map_ || !grid_ready_) return;

    // --- 1. CLEAR OLD MARKERS ---
    server_->clear();

    // --- 2. DRAW INTERACTIVE MARKERS (Adaptive: No Walls) ---
    int id = 0;
    double range_limit = 3.0; // 3 meters bubble

    // Map info for checking obstacles
    int map_w = map_->info.width;
    int map_h = map_->info.height;
    double map_res = map_->info.resolution;
    double map_x0 = map_->info.origin.position.x;
    double map_y0 = map_->info.origin.position.y;

    // Calculate loop bounds based on bubble
    int min_i = (int)((robot_pose_.position.x - range_limit - origin_x_) / grid_step_);
    int max_i = (int)((robot_pose_.position.x + range_limit - origin_x_) / grid_step_);
    int min_j = (int)((robot_pose_.position.y - range_limit - origin_y_) / grid_step_);
    int max_j = (int)((robot_pose_.position.y + range_limit - origin_y_) / grid_step_);

    for (int i = min_i; i <= max_i; i++) {
        for (int j = min_j; j <= max_j; j++) {
             double wx = origin_x_ + (i + 0.5) * grid_step_;
             double wy = origin_y_ + (j + 0.5) * grid_step_;

             // 1. Distance Check (Bubble)
             double dx = wx - robot_pose_.position.x;
             double dy = wy - robot_pose_.position.y;
             if (dx*dx + dy*dy > range_limit*range_limit) continue;

             // 2. OBSTACLE CHECK (The Fix!)
             // Convert World(wx,wy) to Map Index
             int mx = (int)((wx - map_x0) / map_res);
             int my = (int)((wy - map_y0) / map_res);

             // Check bounds
             if (mx >= 0 && mx < map_w && my >= 0 && my < map_h) {
                 int index = my * map_w + mx;
                 int cost = map_->data[index];

                 // ONLY draw if Free (0) or low cost. 
                 // -1 is unknown, 100 is wall.
                 if (cost != -1 && cost < 50) {
                     createInteractiveMarker(wx, wy, id++);
                 }
             }
        }
    }
    server_->applyChanges();

    // --- 3. DRAW VISUAL MARKERS (Robot, Goal, Path) ---
    int vid = 0;
    
    // Robot Marker
    visualization_msgs::Marker robot;
    robot.header.frame_id="map";
    robot.header.stamp=ros::Time::now();
    robot.ns="robot";
    robot.id=vid++;
    robot.type=visualization_msgs::Marker::SPHERE;
    robot.pose.position=robot_pose_.position;
    robot.pose.position.z=0.05;
    robot.scale.x=0.15; robot.scale.y=0.15; robot.scale.z=0.15;
    robot.color.r=1.0; robot.color.a=1.0;
    ma.markers.push_back(robot);

    // Goal & Path
    if (has_goal_) {
        visualization_msgs::Marker goal;
        goal.header.frame_id="map";
        goal.header.stamp=ros::Time::now();
        goal.ns="goal";
        goal.id=vid++;
        goal.type=visualization_msgs::Marker::SPHERE;
        goal.pose.position=goal_point_;
        goal.scale.x=0.15; goal.scale.y=0.15; goal.scale.z=0.05;
        goal.color.b=1.0; goal.color.a=1.0;
        ma.markers.push_back(goal);

        visualization_msgs::Marker line;
        line.header.frame_id="map";
        line.header.stamp=ros::Time::now();
        line.ns="path";
        line.id=vid++;
        line.type=visualization_msgs::Marker::LINE_STRIP;
        line.scale.x=0.05;
        line.color.b=1.0; line.color.a=1.0;
        for (auto& p: path_) line.points.push_back(p);
        ma.markers.push_back(line);
    }

    markers_pub_.publish(ma);
}
bool GridOverlay::computePath(double sx, double sy, double gx, double gy,
                              std::vector<geometry_msgs::Point>& path) {
    if (!got_map_) return false;
    auto costmap = map_; // Use Static Map

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
        if (cost < 0 || cost >= 50) return false; 
        
        // Check inflation (3x3)
        for (int dx=-1; dx<=1; dx++) {
            for (int dy=-1; dy<=1; dy++) {
                int nx = mx + dx, ny = my + dy;
                if (nx >= 0 && nx < map_w && ny >= 0 && ny < map_h) {
                     if (costmap->data[ny*map_w + nx] >= 50) return false;
                }
            }
        }
        return true;
    };

    int si, sj, gi, gj;
    worldToGrid(sx, sy, si, sj);
    worldToGrid(gx, gy, gi, gj);

    // Escape logic for start point
    if (!isFree(si, sj)) {
         bool found = false;
         for(int dx=-2; dx<=2; dx++){
             for(int dy=-2; dy<=2; dy++){
                 if(isFree(si+dx, sj+dy)){
                     si+=dx; sj+=dy;
                     found=true; goto start_found;
                 }
             }
         }
         start_found: if(!found) return false;
    }
    if (!isFree(gi, gj)) return false;

    // A* Search
    using KeyT = uint64_t;
    auto key = [](int i, int j)->KeyT { return ((KeyT)( (uint32_t)i ) << 32) | (uint32_t)j; };
    auto heuristic = [&](int i, int j){ return std::hypot(i-gi, j-gj); };

    std::priority_queue<std::tuple<double,int,int,double>,
                        std::vector<std::tuple<double,int,int,double>>,
                        std::greater<>> open;
    std::unordered_map<KeyT,double> gscore;
    std::unordered_map<KeyT,KeyT> came_from;

    gscore[key(si,sj)] = 0;
    open.emplace(heuristic(si,sj), si, sj, 0);

    const int di[8] = {1,-1,0,0, 1,1,-1,-1};
    const int dj[8] = {0,0,1,-1, 1,-1,1,-1};

    bool found = false;
    while (!open.empty()) {
        auto [f,i,j,g] = open.top(); open.pop();
        if (i==gi && j==gj) { found=true; break; }
        if (g > gscore[key(i,j)] && gscore.count(key(i,j))) continue;

        for (int k=0; k<8; k++) {
            int ni = i+di[k], nj = j+dj[k];
            if (!isFree(ni,nj)) continue;
            double dist = (k<4) ? 1.0 : 1.414;
            double ng = g + dist;
            KeyT nk = key(ni,nj);
            if (!gscore.count(nk) || ng < gscore[nk]) {
                gscore[nk] = ng;
                came_from[nk] = key(i,j);
                open.emplace(ng + heuristic(ni,nj), ni, nj, ng);
            }
        }
    }
    if (!found) return false;

    // Reconstruct
    path.clear();
    int cur_i = gi, cur_j = gj;
    while (cur_i != si || cur_j != sj) {
        double wx, wy; gridToWorld(cur_i, cur_j, wx, wy);
        geometry_msgs::Point p; p.x=wx; p.y=wy; p.z=0;
        path.push_back(p);
        KeyT prev = came_from[key(cur_i, cur_j)];
        cur_i = (int)(prev >> 32);
        cur_j = (int)(prev & 0xFFFFFFFF);
    }
    std::reverse(path.begin(), path.end());
    return true;
}

// --- LOGGING FUNCTIONS (Keep unchanged) ---
void GridOverlay::initLog(const std::string& base_filename) {
    // Same as your previous file
}
void GridOverlay::logPathData(double path_length, int num_turns) {
    // Same as your previous file
}
void GridOverlay::logNavigationData(const std::string& event, double distance_to_goal) {
    // Same as your previous file
}
void GridOverlay::closeLog() {
    // Same as your previous file
}
double GridOverlay::computePathLength(const std::vector<geometry_msgs::Point>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); i++) {
        len += std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
    }
    return len;
}
int GridOverlay::computeNumTurns(const std::vector<geometry_msgs::Point>& path) {
    if (path.size() < 3) return 0;
    int turns = 0;
    for (size_t i = 2; i < path.size(); i++) {
        double dx1 = path[i-1].x - path[i-2].x;
        double dy1 = path[i-1].y - path[i-2].y;
        double dx2 = path[i].x - path[i-1].x;
        double dy2 = path[i].y - path[i-1].y;
        // If cross product is significant, it's a turn
        if (std::abs(dx1*dy2 - dy1*dx2) > 1e-5) turns++;
    }
    return turns;
}
