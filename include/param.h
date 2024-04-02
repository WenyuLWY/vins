#pragma once

#include <ros/ros.h>
#include <thread>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

namespace vins
{
    class parameters 
    {
        public:

            std::string IMU_TOPIC;
            std::string IMAGE_TOPIC;
            std::string DEPTH_TOPIC;

            float fx,fy,cx,cy,k1,k2,p1,p2,k3;
            cv::Mat K,D;
            // int ROW, COL;
            int image_width, image_height;



            parameters() 
            {

            }

            void init(ros::NodeHandle& n)
            {
                n.getParam("image_topic", IMAGE_TOPIC);
                n.getParam("depth_topic", DEPTH_TOPIC);
                n.getParam("imu_topic", IMU_TOPIC);


                n.getParam("fx", fx);
                n.getParam("fy", fy);
                n.getParam("cx", cx);
                n.getParam("cy", cy);
                n.getParam("k1", k1);
                n.getParam("k2", k2);
                n.getParam("p1", p1);
                n.getParam("p2", p2);
                n.getParam("k3", k3);
                K = (cv::Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
                D = (cv::Mat_<float>(1,5) << k1, k2, p1, p2, k3);

                n.getParam("image_width", image_width);
                n.getParam("image_height", image_height);
            }

            void paramList()
            {
                std::cout << "IMAGE_TOPIC: " << IMAGE_TOPIC << std::endl;
                std::cout << "DEPTH_TOPIC: " << DEPTH_TOPIC << std::endl;
            }


        private:


    }; // class parameters
} // namespace det
