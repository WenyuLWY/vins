// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/opencv.hpp>

//c++
#include <mutex>
#include <deque>

#include "parameters.h"
#include "point_tracker.h"
#include "visualization.h"
// #include "tic_toc.h"

std::mutex m_buf;
std::queue<sensor_msgs::Image::ConstPtr> depth_buf;
std::queue<sensor_msgs::Image::ConstPtr> image_buf;
std::queue<sensor_msgs::Image::ConstPtr> mask_buf;

ReadParameters params;
Visualization publishers;
PointTracker point_tracker;

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

void mask_callback(const sensor_msgs::ImageConstPtr &mask_msg)
{
    m_buf.lock();
    mask_buf.emplace(mask_msg);
    m_buf.unlock();
}

bool image_sync()
{
    bool sync_flag = false;

    if (!image_buf.empty() &&  !depth_buf.empty() && !mask_buf.empty() )
    {
        double time_image = image_buf.front()->header.stamp.toSec();
        double time_depth = depth_buf.front()->header.stamp.toSec();
        double time_mask = mask_buf.front()->header.stamp.toSec();
        // ROS_WARN("time_image: %f, time_depth: %f, time_mask: %f", time_image, time_depth, time_mask);

        // 0.003s sync tolerance, rgbd sync
        if(time_depth < time_image - 0.003)
        {
            depth_buf.pop();
            // ROS_WARN("throw depth image, rgbd unsynced"); //ROS_ERROR
            sync_flag = false;
        }
        else if(time_image < time_depth - 0.003)
        {
            image_buf.pop();   
            // ROS_WARN("throw rgb image, rgbd unsynced"); //ROS_ERROR
            sync_flag = false;
        }

        if (time_mask < time_depth - 0.003 || time_mask < time_image - 0.003)
        {
            mask_buf.pop();
            // ROS_WARN("throw mask image, object detection slow");//error
            sync_flag = false;
        }
        else if (time_image < time_mask - 0.003 || time_depth < time_mask - 0.003) // 0.003s sync tolerance, object detection sync
        {
            if( time_image < time_mask - 0.003)
            {
                image_buf.pop();   
                // ROS_WARN("throw rgb image, object detection init");
            }

            if (time_depth < time_mask - 0.003)
            {
                depth_buf.pop();
                // ROS_WARN("throw depth image, object detection inits");
            }

            sync_flag = false;

        }
        else
        {
            sync_flag = true;
        }        
    }
    else
    {
        sync_flag = false;
    }
    

    return sync_flag;
}

void feature_tracker()
{
    while(true)
    {   
        cv::Mat rgb_image, depth_image, mask_image;
        double time = 0;

        m_buf.lock();
        if (image_sync())
        {
            time = image_buf.front()->header.stamp.toSec();
            rgb_image = cv_bridge::toCvCopy(image_buf.front(), sensor_msgs::image_encodings::MONO8)->image;
            depth_image = cv_bridge::toCvCopy(depth_buf.front(), sensor_msgs::image_encodings::TYPE_16UC1)->image;
            mask_image = cv_bridge::toCvCopy(mask_buf.front(), sensor_msgs::image_encodings::MONO8)->image;
            image_buf.pop();
            depth_buf.pop();
            mask_buf.pop();
        }
        m_buf.unlock();


        //input image to estimator
        if (!mask_image.empty() && !rgb_image.empty() && !depth_image.empty())
        {
            point_tracker.trackImage(rgb_image, mask_image, time);

            cv::Mat visMaskImage = point_tracker.getMaskImage();
            publishers.pubMaskImage(visMaskImage, time);
            cv::Mat visPointImage = point_tracker.getTrackImage();
            publishers.pubPointImage(visPointImage, time);
            cv::Mat visOpticalFlowImage = point_tracker.getOpticalFlowImage();
            publishers.pubOpticalFlow(visOpticalFlowImage, time); 
        }



    }
}



// void image_callback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &mask_msg)
// {
//     TicToc t_r;
//     cv::Mat mask_image,color_image;
//     color_image = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::MONO8)->image;
//     mask_image= cv_bridge::toCvCopy(mask_msg, sensor_msgs::image_encodings::MONO8)->image;
//     point_tracker.trackImage(color_image, mask_image, color_msg->header.stamp.toSec());
//     cv::Mat visPointImage = point_tracker.getTrackImage();
//     publishers.pubPointImage(visPointImage, color_msg->header.stamp.toSec());
//     cv::Mat visOpticalFlowImage = point_tracker.getOpticalFlowImage();
//     publishers.pubOpticalFlow(visOpticalFlowImage, color_msg->header.stamp.toSec());
//     cv::Mat visMaskImage = point_tracker.getMaskImage();
//     publishers.pubMaskImage(visMaskImage, color_msg->header.stamp.toSec());
//     ROS_WARN("whole processing costs: %f", t_r.toc());
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);//Debug Info

    params.init(n);
    point_tracker.init(params);
    // params.paramList();

    ROS_WARN("waiting for image and imu...");

    publishers.registerPub(n);

    ros::Subscriber sub_image = n.subscribe(params.IMAGE_TOPIC, 100, image_callback);
    ros::Subscriber sub_depth = n.subscribe(params.DEPTH_TOPIC, 100, depth_callback); 
    ros::Subscriber sub_mask = n.subscribe("/object_detection_node/mask_image", 200, mask_callback);
    std::thread frontend_process{feature_tracker};

    // message_filters::Subscriber<sensor_msgs::Image> sub_image(n, params.IMAGE_TOPIC, 100);
    // message_filters::Subscriber<sensor_msgs::Image> sub_mask(n, "/object_detection_node/mask_image", 100);
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_image, sub_mask, 100);
    // sync.registerCallback(boost::bind(&image_callback, _1, _2));



    // std::thread frontend_process{vins_estimator};

    ros::spin();
    return 0;
}