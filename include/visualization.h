#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Dense>

class Visualization {
    public:
    
        ros::Publisher pub_point_img;
        ros::Publisher pub_plane_img;
        ros::Publisher pub_line_img;
        ros::Publisher pub_optical_flow;
        ros::Publisher pub_mask_image;
        ros::Publisher pub_latest_odometry;
        // ros::Publisher pub_img,pub_match,pub_restart;

        Visualization() {}

        void registerPub(ros::NodeHandle &n)
        {
            pub_point_img = n.advertise<sensor_msgs::Image>("point_img", 1000);
            pub_plane_img = n.advertise<sensor_msgs::Image>("plane_img", 1000);
            pub_line_img = n.advertise<sensor_msgs::Image>("line_img", 1000);
            pub_optical_flow = n.advertise<sensor_msgs::Image>("optical_flow", 1000);
            pub_mask_image = n.advertise<sensor_msgs::Image>("mask_image", 1000);
            pub_latest_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);

            // pub_img = n.advertise<sensor_msgs::PointCloud2>("feature", 1000);
            // pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
            // pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
            
        }

        void pubPointImage(const cv::Mat &img, const double t)
        {
            std_msgs::Header header;
            header.frame_id = "camera_color_optical_frame";
            header.stamp = ros::Time(t);

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
            pub_point_img.publish(img_msg);

        }

        void pubOpticalFlow(const cv::Mat &img, const double t)
        {
            std_msgs::Header header;
            header.frame_id = "camera_color_optical_frame";
            header.stamp = ros::Time(t);

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
            pub_optical_flow.publish(img_msg);
        }

        void pubMaskImage(const cv::Mat &img, const double t)
        {
            std_msgs::Header header;
            header.frame_id = "camera_color_optical_frame";
            header.stamp = ros::Time(t);

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
            pub_mask_image.publish(img_msg);
        }

        void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const std_msgs::Header &header)
        {
            Eigen::Quaterniond quadrotor_Q = Q ;

            nav_msgs::Odometry odometry;
            odometry.header = header;
            odometry.header.frame_id = "map";
            odometry.pose.pose.position.x = P.x();
            odometry.pose.pose.position.y = P.y();
            odometry.pose.pose.position.z = P.z();
            // std::cout<<"P: "<<P.x()<<" "<<P.y()<<" "<<P.z()<<std::endl;
            ROS_WARN("P: %f %f %f",P.x(),P.y(),P.z());
            odometry.pose.pose.orientation.x = quadrotor_Q.x();
            odometry.pose.pose.orientation.y = quadrotor_Q.y();
            odometry.pose.pose.orientation.z = quadrotor_Q.z();
            odometry.pose.pose.orientation.w = quadrotor_Q.w();
            odometry.twist.twist.linear.x = V.x();
            odometry.twist.twist.linear.y = V.y();
            odometry.twist.twist.linear.z = V.z();
            pub_latest_odometry.publish(odometry);
        }

};
