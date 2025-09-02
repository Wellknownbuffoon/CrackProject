#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "gpu_surfel_fusion.h"

class DualCameraDenseFusion {
public:
    DualCameraDenseFusion(ros::NodeHandle& nh)
    {
        pub_surfel_map_ = nh.advertise<sensor_msgs::PointCloud2>("/dense_surfel_map", 1);

        // Subscribe Front camera
        front_rgb_sub_.subscribe(nh, "/d435_front/color/image_raw", 1);
        front_depth_sub_.subscribe(nh, "/d435_front/aligned_depth_to_color/image_raw", 1);
        front_sync_.reset(new FrontSync(FrontSyncPolicy(10), front_rgb_sub_, front_depth_sub_));
        front_sync_->registerCallback(boost::bind(&DualCameraDenseFusion::rgbDepthCallbackFront, this, _1, _2));

        // Subscribe Left camera
        left_rgb_sub_.subscribe(nh, "/d435_left/color/image_raw", 1);
        left_depth_sub_.subscribe(nh, "/d435_left/aligned_depth_to_color/image_raw", 1);
        left_sync_.reset(new LeftSync(LeftSyncPolicy(10), left_rgb_sub_, left_depth_sub_));
        left_sync_->registerCallback(boost::bind(&DualCameraDenseFusion::rgbDepthCallbackLeft, this, _1, _2));

        ROS_INFO("Dual Camera Dense Fusion Node Initialized.");
    }

private:
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber;

    ImageSubscriber front_rgb_sub_, front_depth_sub_;
    ImageSubscriber left_rgb_sub_, left_depth_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> FrontSyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<FrontSyncPolicy>> front_sync_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> LeftSyncPolicy;
    boost::shared_ptr<message_filters::Synchronizer<LeftSyncPolicy>> left_sync_;

    ros::Publisher pub_surfel_map_;
    tf::TransformListener tf_listener_;

    GPU::SurfelMap global_map_;

    void rgbDepthCallbackFront(const sensor_msgs::ImageConstPtr& rgb_msg,
                               const sensor_msgs::ImageConstPtr& depth_msg)
    {
        processFrame(rgb_msg, depth_msg, "d435_front");
    }

    void rgbDepthCallbackLeft(const sensor_msgs::ImageConstPtr& rgb_msg,
                              const sensor_msgs::ImageConstPtr& depth_msg)
    {
        processFrame(rgb_msg, depth_msg, "d435_left");
    }

    void processFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg,
                      const std::string& camera_name)
    {
        // Convert ROS images to OpenCV
        cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
        cv::Mat depth = cv_bridge::toCvShare(depth_msg, "16UC1")->image;

        // Generate surfels (GPU)
        std::vector<GPU::Surfel> surfels = GPU::generateSurfels(rgb, depth, camera_name);

        // Transform to map frame
        GPU::transformSurfelsToMap(surfels, tf_listener_, camera_name);

        // Fuse into global map
        global_map_.fuseSurfels(surfels);

        // Publish
        sensor_msgs::PointCloud2 output_msg;
        global_map_.toROSMsg(output_msg, "map");
        pub_surfel_map_.publish(output_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_camera_dense_fusion");
    ros::NodeHandle nh;

    DualCameraDenseFusion fusion_node(nh);
    ros::spin();
    return 0;
}
