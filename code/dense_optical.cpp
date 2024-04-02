#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>


#include <Eigen/Dense>
#include "timer.h"
#include <omp.h>

double fx,fy,cx,cy;
double depth_factor;

ros::Publisher pcl_dense_optical;


cv::Mat prev_img;


void drawOpticalFlow(const cv::Mat &flow, cv::Mat &flowImage, int step = 16, float scale = 10.0) {
    cv::Mat hsv(flow.size(), CV_8UC3, cv::Scalar::all(255));
    for (int y = 0; y < flow.rows; y += step) {
        for (int x = 0; x < flow.cols; x += step) {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            float angle = std::atan2(fxy.y, fxy.x) * 180 / CV_PI;
            float magnitude = sqrt(fxy.x * fxy.x + fxy.y * fxy.y);

            cv::Vec3b& hsvPixel = hsv.at<cv::Vec3b>(y, x);
            hsvPixel[0] = static_cast<uchar>((angle + 360) / 2); // 色调
            hsvPixel[1] = 255; // 饱和度
            hsvPixel[2] = cv::saturate_cast<uchar>(magnitude * scale); // 亮度
        }
    }
    cv::cvtColor(hsv, flowImage, cv::COLOR_HSV2BGR);
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg)
{

    cv::Mat cur_img;
    cv::Mat flow,flowImage;
    cv::Mat vis;

    cur_img = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8)->image;
    
    if (prev_img.empty()) 
    {
        prev_img = cur_img;
        return;
    }

    cv::calcOpticalFlowFarneback(prev_img, cur_img, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
    
   
    prev_img = cur_img.clone();

    // drawOpticalFlow(flow, flowImage);

    cv::cvtColor(prev_img, flowImage, cv::COLOR_GRAY2BGR);

    // flowImage = cv::Mat::ones(cur_img.rows, cur_img.cols, CV_8UC3) * 255;

    double sumX = 0, sumY = 0;
    int count = 0;

    for (int row = 0; row < flow.rows; row++) {
        for (int col = 0; col < flow.cols; col++) {
            const cv::Point2f fxy = flow.at<cv::Point2f>(row, col);
            sumX += std::abs(fxy.x);
            sumY += std::abs(fxy.y);
            count++;
        }
    }

    double avgX = sumX / count;
    double avgY = sumY / count;
    // std::cout << "avgX: " << avgX << std::endl;
    // std::cout << "avgY: " << avgY << std::endl;

    for (int row = 0; row < flow.rows; row++) {
        for (int col = 0; col < flow.cols; col++) {
            const cv::Point2f fxy = flow.at<cv::Point2f>(row, col);
            if (std::abs(fxy.x) > 10*avgX || std::abs(fxy.y >10*avgY)) {				
                cv::circle(flowImage, cv::Point(col, row), 2, cv::Scalar(0, 255, 0), -1);
            }
        }
    }

    std_msgs::Header header;
    header.frame_id = "camera_color_optical_frame";
    header.stamp = image_msg->header.stamp;

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", flowImage).toImageMsg();
    pcl_dense_optical.publish(img_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dense_optical_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    pcl_dense_optical = n.advertise<sensor_msgs::Image>("dense_optical_flow", 10);

    ros::Subscriber sub_image = n.subscribe("/camera/color/image_raw", 100, image_callback); 
    int num_procs = omp_get_num_procs();


    // std::string depth_path;
    // n.getParam("depth_path", depth_path);
    n.getParam("fx", fx);
    n.getParam("fy", fy);
    n.getParam("cx", cx);
    n.getParam("cy", cy);
    n.getParam("depth_factor", depth_factor);
    std::cout << "fx: " << fx << std::endl;

    ros::spin();


    return 0;
}