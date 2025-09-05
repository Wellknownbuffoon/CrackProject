#include "grid_overlay.h"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <algorithm>

GridOverlay::GridOverlay(ros::NodeHandle& nh) {
    map_sub_   = nh.subscribe("map", 1, &GridOverlay::mapCb, this);
    amcl_sub_  = nh.subscribe("/amcl_pose", 1, &GridOverlay::amclCb, this);
    click_sub_ = nh.subscribe("/clicked_point", 1, &GridOverlay::clickCb, this);

    markers_pub_  = nh.advertise<visualization_msgs::MarkerArray>("grid_overlay", 1);
    nav_cmd_pub_  = nh.advertise<geometry_msgs::Twist>("/nav_cmd_vel", 10);

    grid_step_ = 0.02;
    got_pose_ = got_map_ = grid_ready_ = false;
    has_goal_ = false;
    path_index_ = 0;

    ROS_INFO("GridOverlay initialized (click-to-goal enabled)");
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
    has_goal_ = true;

    double sx, sy;
    snapToGrid(robot_pose_.position.x, robot_pose_.position.y, sx, sy);

    if (computePath(sx, sy, gx, gy, path_)) {
        path_index_ = 0;
        ROS_INFO("Path computed with %zu waypoints", path_.size());
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
    if (!map_) return false;
    int map_w = map_->info.width;
    int map_h = map_->info.height;
    double map_x0 = map_->info.origin.position.x;
    double map_y0 = map_->info.origin.position.y;
    double map_res = map_->info.resolution;

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
        return map_->data[idx] == 0;
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
    return true;
}

void GridOverlay::publishMarkers() {
    visualization_msgs::MarkerArray ma;
    if (!map_ || !grid_ready_) return;

    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    ma.markers.push_back(clear);

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

            visualization_msgs::Marker m;
            m.header.frame_id="map";
            m.header.stamp=ros::Time::now();
            m.ns="grid";
            m.id=id++;
            m.type=visualization_msgs::Marker::SPHERE;
            m.pose.position.x=wx;
            m.pose.position.y=wy;
            m.pose.position.z=0.01;
            m.scale.x=grid_step_*0.6;
            m.scale.y=grid_step_*0.6;
            m.scale.z=0.01;
            m.color.g=1.0; m.color.a=1.0;

            m.lifetime = ros::Duration(0.6);
            
            ma.markers.push_back(m);
        }
    }

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
