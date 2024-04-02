// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

//c++
#include <vector>
#include <iostream>
#include <mutex>
#include <deque>
#include <condition_variable>

#include "parameters.h"

// std::mutex m_m_buf;
std::mutex m_buf;
// std::condition_variable con_m_buf;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<sensor_msgs::Image::ConstPtr> depth_buf;
std::queue<sensor_msgs::Image::ConstPtr> image_buf;

ReadParameters params;

// FeatureTracker point_tracker;
// LineTracker line_tracker;
// PlaneTracker plane_tracker;

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
    m_buf.lock();
    image_buf.emplace(image_msg);//push
    m_buf.unlock();
}

void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{
    m_buf.lock();
    depth_buf.emplace(depth_msg);
    m_buf.unlock();
}

// void feature_tracker()
// {
//     while(true)
//     {
//         cv::Mat rgb_image, depth_image;
//         double time = 0;
//         m_buf.lock();
//         if (!image_buf.empty() && !depth_buf.empty())
//         {
//             double time_image = image_buf.front()->header.stamp.toSec();
//             double time_depth = depth_buf.front()->header.stamp.toSec();

//             // 0.003s sync tolerance
//             if(time_depth < time_image - 0.003)
//             {
//                 depth_buf.pop();
//                 ROS_WARN("throw depth image");
//             }
//             else if(time_depth > time_image + 0.003)
//             {
//                 image_buf.pop();   
//                 ROS_WARN("throw rgb image");
//             }
//             else
//             {
//                 time = time_image;
//                 rgb_image = cv_bridge::toCvCopy(image_buf.front(), sensor_msgs::image_encodings::MONO8)->image;
//                 depth_image = cv_bridge::toCvCopy(depth_buf.front(), sensor_msgs::image_encodings::MONO16)->image;
//                 header = image_buf.front()->header;
//                 image_buf.pop();
//                 depth_buf.pop();
//             }
//         }
//         m_buf.unlock();
//         //input image to estimator
//         if (!depth_image.empty() && !rgb_image.empty())
//         {
//             std::thread thread_points{trackPoints, rgb_image, depth_image, time};
//         }
//     }
// }

// void trackPoints(const cv::Mat& rgb_image, const cv::Mat& depth_image, double cur_time)
// {
//     // feature_tracker.trackImage(rgb_image, cur_time);
//     // feature_tracker.associateDepthGMM(depth_image);
// }

// void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
// {
//     if (imu_msg->header.stamp.toSec() <= last_imu_t)
//     {
//         ROS_WARN("imu message in disorder!");
//         return;
//     }
//     last_imu_t = imu_msg->header.stamp.toSec();

//     m_m_buf.lock();
//     imu_buf.push(imu_msg);
//     m_m_buf.unlock();
//     // con_m_buf.notify_one();
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);//debug

    params.init(n);
    // params.paramList();

    ROS_WARN("waiting for image and imu...");

    // publishers.registerPub(n);

    // ros::Subscriber sub_imu = n.subscribe(params.IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber sub_image = n.subscribe(params.IMAGE_TOPIC, 100, image_callback);
    // ros::Subscriber sub_depth = n.subscribe(params.DEPTH_TOPIC, 100, depth_callback); 

    // std::thread frontend_process{feature_tracker};
    // std::thread frontend_process{vins_estimator};
    ros::spin();
    return 0;
}