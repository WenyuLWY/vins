// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

//c++
#include <mutex>
#include <deque>

#include "parameters.h"
#include "visualization.h"
#include "point_tracker.h"




ReadParameters params;
Visualization publishers;
PointTracker point_tracker;
// LineTracker line_tracker;
// PlaneTracker plane_tracker;


double first_image_time,last_image_time;
bool first_image_flag = true;

int FREQ;
bool PUB_THIS_FRAME;
int pub_count = 1;

bool init(const sensor_msgs::ImageConstPtr &color_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = color_msg->header.stamp.toSec();
        last_image_time = color_msg->header.stamp.toSec();
        return false;
    }
    

    // // detect unstable camera stream
    // if (color_msg->header.stamp.toSec() - last_image_time > 1.0 || color_msg->header.stamp.toSec() < last_image_time)
    // {
    //     ROS_WARN("image discontinue! reset the feature tracker!");
    //     first_image_flag = true; 
    //     last_image_time = 0;
    //     pub_count = 1;
    //     std_msgs::Bool restart_flag;
    //     restart_flag.data = true;
    //     pub_restart.publish(restart_flag);
    //     return false;
    // }
    // last_image_time = color_msg->header.stamp.toSec();

    // frequency control
    if (round(1.0 * pub_count / (color_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (color_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = color_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    return true;
}

void image_callback(const sensor_msgs::ImageConstPtr &color_msg)
{

    if (!init(color_msg))
    {
        return;
    }

    cv::Mat show_img,color_image,depth_image;
    color_image = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::MONO8)->image;

    depth_image=color_image;
    point_tracker.trackImage(color_image, depth_image, color_msg->header.stamp.toSec());

    cv::Mat visPointImage = point_tracker.getTrackImage();
    publishers.pubPointImage(visPointImage, color_msg->header.stamp.toSec());
    cv::Mat visOpticalFlowImage = point_tracker.getOpticalFlowImage();
    publishers.pubOpticalFlow(visOpticalFlowImage, color_msg->header.stamp.toSec());
    cv::Mat visMaskImage = point_tracker.getMaskImage();
    publishers.pubMaskImage(visMaskImage, color_msg->header.stamp.toSec());

    // if (PUB_THIS_FRAME)
    // {
    //     pub_count++;

    // }
    // else
    // {
    //     return;
    // }
        
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker_test");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);//Debug Info

    params.init(n);
    point_tracker.init(params);
    FREQ=params.FREQ;
    PUB_THIS_FRAME=params.PUB_THIS_FRAME;
    // params.paramList();

    ROS_WARN("waiting for image and imu...");

    publishers.registerPub(n);

    ros::Subscriber sub_image = n.subscribe(params.IMAGE_TOPIC, 100, image_callback);
    // ros::Subscriber sub_mask = n.subscribe("/object_detection_node/mask_image", 200, mask_callback);

    ros::spin();
    return 0;
}