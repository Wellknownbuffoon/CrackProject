#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

namespace GPU {

struct Surfel {
    float x,y,z;
    float nx,ny,nz;
    uint8_t r,g,b;
    float radius;
    float confidence;
    float curvature;
    float crack_width;
};

class SurfelMap {
public:
    void fuseSurfels(const std::vector<Surfel>& new_surfels);
    void toROSMsg(sensor_msgs::PointCloud2& msg, const std::string& frame_id);
};

std::vector<Surfel> generateSurfels(const cv::Mat& rgb, const cv::Mat& depth, const std::string& camera_name);
void transformSurfelsToMap(std::vector<Surfel>& surfels, tf::TransformListener& listener, const std::string& camera_name);

}
