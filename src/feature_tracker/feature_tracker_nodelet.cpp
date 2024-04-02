#include <ros/ros.h>
#include <nodelet/nodelet.h>  
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>


#include <boost/type_index.hpp>
#include <thread>
#include <condition_variable>
#include <omp.h>


//msgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
// #include <sensor_msgs/image_encodings.h>


// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>


#include "timer.h"
#include "param.h"
#include "feature_tracker/feature_tracker.h"



namespace feature_tracker
{
    class FeatureTrackerNodelet : public nodelet::Nodelet
    {
    public:
        FeatureTrackerNodelet(){}
        ~FeatureTrackerNodelet() {
            ROS_WARN("FeatureTrackerNodelet destructor");
        }

    private:
        ros::Publisher pub_cloud;
        ros::Subscriber sub_image, sub_depth;
        std::mutex m_buf;
        std::condition_variable cond_var;
        std::queue<sensor_msgs::Image::ConstPtr> depth_buf;
        std::queue<sensor_msgs::Image::ConstPtr> image_buf;
        std::thread depth_detection_thread;
        vins::parameters param;
        virtual void onInit()
        {
            ros::NodeHandle& private_nh = getPrivateNodeHandle();
            ros::NodeHandle& global_nh = getNodeHandle();
            ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

            // vins::parameters param(n);
            
            param.init(private_nh);
            // param.paramList();

            sub_image = private_nh.subscribe(param.IMAGE_TOPIC, 100, &FeatureTrackerNodelet::image_callback, this);
            sub_depth = private_nh.subscribe(param.DEPTH_TOPIC, 100, &FeatureTrackerNodelet::depth_callback, this);

            pub_cloud = private_nh.advertise<sensor_msgs::PointCloud2>("pcd_cloud", 10);

            depth_detection_thread   = std::thread(&FeatureTrackerNodelet::depth_detection, this);
        }

        void depth_detection()
        {
            std::unique_lock<std::mutex> lock(m_buf, std::defer_lock);
            while(1)
            {
                lock.lock();
                cond_var.wait(lock, [this]{ return !image_buf.empty() && !depth_buf.empty(); });

                cv::Mat mono_image,rgb_image, depth_image;
                // pcl_depth_image;
                rgb_image = cv_bridge::toCvCopy(image_buf.front(), sensor_msgs::image_encodings::BGR8)->image;
                
                depth_image = cv_bridge::toCvCopy(depth_buf.front(), sensor_msgs::image_encodings::TYPE_16UC1)->image;
                // pcl_conversions::toPCL(*(depth_buf.front()), pcl_depth_image);


                // cv::Mat pcd_xyzrgb;
                // det::depthToxyzrgb(depth_image, rgb_image, param.K, pcd_xyzrgb);

                // sensor_msgs::PointCloud2 cloud_msg;
                // cloud_msg.header.frame_id = "base_link";
                // det::Mat2ROSMsg(pcd_xyzrgb, cloud_msg);
                // pub_cloud.publish(cloud_msg); 

                double time_image = image_buf.front()->header.stamp.toSec();
                double time_depth = depth_buf.front()->header.stamp.toSec();
                // ROS_WARN("time_image: %f, time_depth: %f", time_image, time_depth);
                image_buf.pop();
                depth_buf.pop();
                
                lock.unlock();
            }

        }

        void image_callback(const sensor_msgs::ImageConstPtr& image_msg) 
        {
            std::lock_guard<std::mutex> guard(m_buf);
            image_buf.emplace(image_msg);
            cond_var.notify_one(); // or notify_all
        }

        void depth_callback(const sensor_msgs::ImageConstPtr& depth_msg) 
        {
            std::lock_guard<std::mutex> guard(m_buf);
            depth_buf.emplace(depth_msg);
            cond_var.notify_one(); // or notify_all
        }



    };

PLUGINLIB_EXPORT_CLASS(feature_tracker::FeatureTrackerNodelet, nodelet::Nodelet)
}

