#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>

// #include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include "timer.h"
#include <omp.h>

double fx,fy,cx,cy;
double depth_factor;

ros::Publisher pcl_pub;


void depth2pcd_old(const cv::Mat& depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float precision=2) {
    
    double precision_control = std::pow(10.0, precision); 

    cloud->clear();

    // cloud->points.resize(non_zero_count);
    // cloud->width = non_zero_count;
    // cloud->height = 1;
    // cloud->is_dense = false;
    // int point_idx = 0;

    // #pragma omp parallel for
    for (auto row = 0; row < depth_image.rows; row++) 
    {
        for (auto col = 0; col < depth_image.cols; col++) 
        {
            auto depth_value = depth_image.at<uint16_t>(row, col);
            if (depth_value != 0) 
            {
                float z = std::round(depth_value / depth_factor * precision_control) / precision_control;
                float x = std::round((col - cx) * z / fx * precision_control) / precision_control;
                float y = std::round((row - cy) * z / fy * precision_control) / precision_control;

                cloud->points.push_back(pcl::PointXYZ(z, -x, -y));   //ros
                
                
                // #pragma omp critical
                // std::cout<<"x: "<<x<<" y: "<<y<<" z: "<<z<<std::endl;
                
                // cloud->points.push_back(pcl::PointXYZ(-x, -y, z));      //pcl
                

                // pcl::PointXYZ& point = cloud->points[point_idx++];
                // point.x = -x;
                // point.y = -y;
                // point.z = z;
            }
        }
    }
}



// void depth2pcd(const cv::Mat& depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int num_procs,float precision=2) {
    

//     double precision_control = std::pow(10.0, precision); 


//     cloud->clear();

//     cloud->width = depth_image.cols;  // 图像宽度
//     cloud->height = depth_image.rows;  // 图像高度
//     cloud->is_dense = false; // 点云中可能包含无效点
//     cloud->points.resize(cloud->width * cloud->height);

//     // cloud->points.resize(non_zero_count);
//     // cloud->width = non_zero_count;
//     // cloud->height = 1;
//     // cloud->is_dense = false;
//     // int point_idx = 0;
//     omp_set_num_threads(num_procs);
//     #pragma omp parallel for
//     for (auto row = 0; row < depth_image.rows; row++) 
//     {
//         for (auto col = 0; col < depth_image.cols; col++) 
//         {
//             auto depth_value = depth_image.at<uint16_t>(row, col);
//             if (depth_value != 0) 
//             {
//                 float z = std::round(depth_value / depth_factor * precision_control) / precision_control;
//                 float x = std::round((col - cx) * z / fx * precision_control) / precision_control;
//                 float y = std::round((row - cy) * z / fy * precision_control) / precision_control;

//                 cloud->points[row * depth_image.cols + col] = pcl::PointXYZ(z, -x, -y);

                
//                     // cloud->points.push_back(pcl::PointXYZ(z, -x, -y));   //ros
                
                
//                 // #pragma omp critical
//                 // std::cout<<"x: "<<x<<" y: "<<y<<" z: "<<z<<std::endl;
                
//                 // cloud->points.push_back(pcl::PointXYZ(-x, -y, z));      //pcl
                

//                 // pcl::PointXYZ& point = cloud->points[point_idx++];
//                 // point.x = -x;
//                 // point.y = -y;
//                 // point.z = z;
//             }
//             else
//             {
//                 cloud->points[row * depth_image.cols + col] = pcl::PointXYZ(std::numeric_limits<float>::quiet_NaN(),
//                                                                    std::numeric_limits<float>::quiet_NaN(),
//                                                                    std::numeric_limits<float>::quiet_NaN());
//             }
//         }
//     }


// }

// void depth2pcd(const cv::Mat& depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int num_procs)
// {
//     cloud->clear();
//     double precision_control = 100.0;

//     std::vector<cv::Point> locations;
//     cv::findNonZero(depth_image, locations);
//     cloud->points.resize(locations.size());

//     omp_set_num_threads(num_procs);
//     #pragma omp parallel for
//         for (int i = 0; i < locations.size(); i++) 
//         {
//             auto depth_value = depth_image.at<uint16_t>(locations[i].y, locations[i].x);
//             // float z = std::round(depth_value / depth_factor * precision_control) / precision_control;
//             // float x = std::round((locations[i].x - cx) * z / fx * precision_control) / precision_control;
//             // float y = std::round((locations[i].y - cy) * z / fy * precision_control) / precision_control;
//             float z = depth_value / depth_factor;
//             float x = (locations[i].x - cx) * z / fx;
//             float y = (locations[i].y - cy) * z / fy;
//             cloud->points[i] = pcl::PointXYZ(z, -x, -y);
//         }
// }

void r2dio(const cv::Mat& depth_image, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cv::Mat_<cv::Vec3f> cloud_peac(depth_image.rows, depth_image.cols);
    for (int r = 0; r < depth_image.rows; r++) 
    {
        const float *depth_ptr = depth_image.ptr<float>(r);
        cv::Vec3f *pt_ptr = cloud_peac.ptr<cv::Vec3f>(r);
        for (int c = 0; c < depth_image.cols; c++) 
        {
            float z = (float) depth_ptr[c] / depth_factor;

            pcl::PointXYZ &p = cloud->points[r * depth_image.cols + c];
            p.z = z;
            p.x = (c - cx) * p.z / fx;
            p.y = (r - cy) * p.z / fy;

            // p.b = color_im.ptr<uchar>(r)[c * 3];
            // p.g = color_im.ptr<uchar>(r)[c * 3 + 1];
            // p.r = color_im.ptr<uchar>(r)[c * 3 + 2];

            // pt_ptr[c][0] = p.x * depth_factor;//m->mm
            // pt_ptr[c][1] = p.y * depth_factor;//m->mm
            // pt_ptr[c][2] = z * depth_factor;//m->mm
        }
    }
}

void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{

    
    auto timer = std::make_unique<Timer>("depth2pcd");
            cv::Mat depth_image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    // depth2pcd(depth_image, cloud, 20);
    // depth2pcd_old(depth_image, cloud);
    
    {
         auto timer = std::make_unique<Timer>("depth2pcd");
        r2dio(depth_image, cloud);
    }
   
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "base_link"; // 设置坐标系
    output.header.stamp = depth_msg->header.stamp;
    pcl_pub.publish(output);
 

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_cluster_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    pcl_pub = n.advertise<sensor_msgs::PointCloud2>("pcl_output", 10);

    ros::Subscriber sub_depth = n.subscribe("/camera/aligned_depth_to_color/image_raw", 100, depth_callback); 
    int num_procs = omp_get_num_procs();


    std::string depth_path;
    n.getParam("depth_path", depth_path);
    n.getParam("fx", fx);
    n.getParam("fy", fy);
    n.getParam("cx", cx);
    n.getParam("cy", cy);
    n.getParam("depth_factor", depth_factor);
    std::cout << "fx: " << fx << std::endl;

    // cv::Mat depth_image = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_old(new pcl::PointCloud<pcl::PointXYZ>);

    ros::spin();

    // std::cout << "Number of points in cloud: " << cloud->size() << std::endl;

    // std::cout<<"hole processing costs:" << time.toc() << std::endl;
    // std::cout << "Number of points in cloud: " << cloud->size() << std::endl;




    // // Assuming cloud is a pcl::PointCloud<pcl::PointXYZ>::Ptr
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "sample cloud");
    // viewer->addCoordinateSystem(1.0);

    // viewer->setCameraPosition(-0.481567, 1.09875, -1.92997,  // Position
    //                       0.0115483, 0.35773, 1.07959,  // Viewpoint
    //                       0.018848, 0.971539, 0.236127);  // Up vector

    // // Main visualization loop
    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce(100);
    // }





    // 归一化深度图

    // cv::Mat depth_normalized;
    // double min;
    // double max;
    // cv::minMaxIdx(depth_image, &min, &max);
    // depth_image.convertTo(depth_normalized, CV_8UC1, 255 / (max-min), -min);

    // cv::imshow("depth_image", depth_normalized);
    // cv::waitKey(0);
    return 0;
}