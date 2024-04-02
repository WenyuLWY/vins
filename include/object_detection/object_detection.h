#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace det
{
    std::string type2str(int type) {
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) {
            case CV_8U:  r = "8U"; break;
            case CV_8S:  r = "8S"; break;
            case CV_16U: r = "16U"; break;
            case CV_16S: r = "16S"; break;
            case CV_32S: r = "32S"; break;
            case CV_32F: r = "32F"; break;
            case CV_64F: r = "64F"; break;
            default:     r = "User"; break;
        }

        r += "C";
        r += (chans+'0');

        return r;
    }

    void printtype(const cv::Mat &mat)
    {
        std::string matType = type2str(mat.type());
        std::cout << "Matrix: " << matType << std::endl;
    }

    void Mat2ROSMsg (const cv::Mat& cvPointCloud, sensor_msgs::PointCloud2& cloudMsg) 
    {

        // ROS_ASSERT(cvPointCloud.type() == CV_32FC3);
        if(cvPointCloud.type() == CV_32FC3)
        {
            cloudMsg.height = cvPointCloud.rows; // 无组织点云的高度为1
            cloudMsg.width = cvPointCloud.cols; // 点云中点的总数
            cloudMsg.is_dense = false; // 点云中可能包含NaN或Inf

            sensor_msgs::PointCloud2Modifier modifier(cloudMsg);
            modifier.setPointCloud2FieldsByString(1, "xyz");

            cloudMsg.data.resize(cvPointCloud.rows * cvPointCloud.cols * sizeof(float) * 3); // 3个浮点数（XYZ）每个点
            cloudMsg.point_step = sizeof(float) * 3; // 每个点的大小
            cloudMsg.row_step = cloudMsg.point_step * cvPointCloud.cols; // 每一行的大小

            const float* mat_data = (const float*)cvPointCloud.data;
            std::memcpy(cloudMsg.data.data(), mat_data, cloudMsg.data.size());
        }
        else if (cvPointCloud.type() == CV_MAKETYPE(CV_32F, 6))
        {
            // ROS_WARN("CV_32FC6");
        }


    }

    cv::Mat getcolorLUT(int numColors)
    {
    
        cv::Mat hsvLUT(256, 1, CV_8UC3,cv::Scalar(0, 0, 0));
        // std::cout<<numColors << std::endl;
        // ROS_WARN("numColors: %d", numColors);

            for (int i = 0; i < numColors; i++) {
            //默认为黑色
            hsvLUT.at<cv::Vec3b>(i) = cv::Vec3b(i * 180 / numColors, 255, 255);
        }
        hsvLUT.at<cv::Vec3b>(255) = cv::Vec3b(0, 0, 0);


        cv::Mat bgrLUT;

        cv::cvtColor(hsvLUT, bgrLUT, cv::COLOR_HSV2BGR);
        return bgrLUT;

    }

    void depthToxyzrgb(const cv::Mat& in_depth, const cv::Mat& rgb, const cv::Mat_<float>& K, cv::Mat& points3d)
    {
        ROS_ASSERT(K.cols == 3 && K.rows == 3 &&  K.depth()==CV_32F);
        ROS_ASSERT(in_depth.type() == CV_16UC1);
        ROS_ASSERT(rgb.type() == CV_8UC3);

        const float inv_fx = 1.0f / K(0, 0);
        const float inv_fy = 1.0f / K(1, 1);
        const float ox = K(0, 2);
        const float oy = K(1, 2);
        cv::Mat_<float> z_mat;
        in_depth.convertTo(z_mat, CV_32F, 0.001); 

        points3d.create(in_depth.size(), CV_MAKETYPE(CV_32F, 6));



        cv::Mat_<float> x_cache(1, in_depth.cols), y_cache(in_depth.rows, 1);
        float* x_cache_ptr = x_cache[0], *y_cache_ptr = y_cache[0];
        for (int x = 0; x < in_depth.cols; ++x, ++x_cache_ptr)
            *x_cache_ptr = (x - ox) * inv_fx;
        for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
            *y_cache_ptr = (y - oy) * inv_fy;
            y_cache_ptr = y_cache[0];
    

        for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
        {
            cv::Vec<float, 3>* point = points3d.ptr<cv::Vec<float, 3> >(y);
            const cv::Vec3b* rgb_ptr = rgb.ptr<cv::Vec3b>(y);

            const float* x_cache_ptr_end = x_cache[0] + in_depth.cols;
            const float* depth = z_mat[y];
             
            for (x_cache_ptr = x_cache[0]; x_cache_ptr != x_cache_ptr_end; ++x_cache_ptr, ++point, ++depth)
            {
                float z = *depth;
                (*point)[0] = (*x_cache_ptr) * z;
                (*point)[1] = (*y_cache_ptr) * z;
                (*point)[2] = z;   
                (*point)[3] = (*rgb_ptr)[0]; 
                (*point)[4] = (*rgb_ptr)[1]; 
                (*point)[5] = (*rgb_ptr)[2]; 
            }
        }
    }


    void depthToxyz(const cv::Mat& in_depth, const cv::Mat& rgb, const cv::Mat_<float>& K, cv::Mat& points3d)
    {
        ROS_ASSERT(K.cols == 3 && K.rows == 3 &&  K.depth()==CV_32F);
        ROS_ASSERT(in_depth.type() == CV_16UC1);
        
        const float inv_fx = 1.0f / K(0, 0);
        const float inv_fy = 1.0f / K(1, 1);
        const float ox = K(0, 2);
        const float oy = K(1, 2);
        cv::Mat_<float> z_mat;
        in_depth.convertTo(z_mat, CV_32F, 0.001); 

        points3d.create(in_depth.size(), CV_MAKETYPE(CV_32F, 3));



        cv::Mat_<float> x_cache(1, in_depth.cols), y_cache(in_depth.rows, 1);
        float* x_cache_ptr = x_cache[0], *y_cache_ptr = y_cache[0];
        for (int x = 0; x < in_depth.cols; ++x, ++x_cache_ptr)
            *x_cache_ptr = (x - ox) * inv_fx;
        for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
            *y_cache_ptr = (y - oy) * inv_fy;
            y_cache_ptr = y_cache[0];
    

        for (int y = 0; y < in_depth.rows; ++y, ++y_cache_ptr)
        {
            cv::Vec<float, 3>* point = points3d.ptr<cv::Vec<float, 3> >(y);
            
            const float* x_cache_ptr_end = x_cache[0] + in_depth.cols;
            const float* depth = z_mat[y];
             
            for (x_cache_ptr = x_cache[0]; x_cache_ptr != x_cache_ptr_end; ++x_cache_ptr, ++point, ++depth)
            {
                float z = *depth;
                (*point)[0] = (*x_cache_ptr) * z;
                (*point)[1] = (*y_cache_ptr) * z;
                (*point)[2] = z;   
            }
        }
    }



}

