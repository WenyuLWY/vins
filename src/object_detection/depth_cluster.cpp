#include <ros/ros.h>
#include <boost/type_index.hpp>

// roslib
#include <cv_bridge/cv_bridge.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp> 

//msgs
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
// #include <sensor_msgs/image_encodings.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

//REMOVE NAN
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h> //vox
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
//plane
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

// normal
#include <pcl/features/integral_image_normal.h>

// GROUND SEG
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>
#include "timer.h"
#include "object_detection/object_detection.h"
#include <omp.h>


#define HASH_P 116101
#define MAX_N 10000000000

float fx,fy,cx,cy;
float depth_factor;
int image_width, image_height;
int decimation = 1;

ros::Publisher pcl_pub;
ros::Publisher image_plane_pub;

Timer* timer = nullptr; 

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

std::mutex mtx; 

float precision_control;









// void mask2pcd(const cv::Mat& mask,const cv::Mat& depth_image)
// {
//     for (auto row = 0; row < mask.rows; row++) 
//     {
//         for (auto col = 0; col < mask.cols; col++) 
//         {
//             auto depth_value = depth_image.at<uint16_t>(row, col);
//             if (depth_value != 0) 
//             {
//                 float z = std::round(depth_value / depth_factor * precision_control) / precision_control;
//                 float x = std::round((col - cx) * z / fx * precision_control) / precision_control;
//                 float y = std::round((row - cy) * z / fy * precision_control) / precision_control;

//                 cloud->points.push_back(pcl::PointXYZ(z, -x, -y));   //ros
//             }
//         }
//     }

// }


void depth_callback(const sensor_msgs::ImageConstPtr &depth_msg)
{

    cv::Mat depth_image;
    depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    cv::Mat filteredImage;


    // timer = new Timer("medianBlur");
    cv::medianBlur(depth_image, filteredImage, 5);
    // delete timer;




    //output cloud.size     std::cout << "cloud->size: " << cloud->size() << std::endl;


    cv::Mat CAMERA_MATRIX = (cv::Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    cv::Mat opencv_pcd;
    det::depthTo3d(filteredImage, CAMERA_MATRIX, opencv_pcd);
        
    // cv::rgbd::depthTo3d(filteredImage, CAMERA_MATRIX, opencv_pcd);
    


    //    cv::Ptr<cv::rgbd::RgbdPlane> rgbdPlane = cv::rgbd::RgbdPlane::create(
    //     cv::rgbd::RgbdPlane::RGBD_PLANE_METHOD_DEFAULT, // 方法
    //     250, // 块大小
    //     500, // 最小聚类大小
    //     0.01, // 阈值（米）
    //     0, // Kinect 传感器误差系数
    //     0,
    //     0
    // );
    // cv::Mat planeMask,planeCoefficients;
    // timer = new Timer("rgbdPlane");
    //     rgbdPlane->operator()(opencv_pcd, planeMask, planeCoefficients);
    // delete timer;

    // cv::Mat colorMask;
    // cv::Mat colorLUT = det::getcolorLUT(planeCoefficients.rows);
    // cv::Mat mask3c;
    // cv::cvtColor(planeMask, mask3c, cv::COLOR_GRAY2BGR);
    // cv::LUT(mask3c, colorLUT, colorMask);

    //planeCoefficients.rows+1类，从0开始，255为非地面


    // for (int value : uniqueValues) {
    //     std::cout << value << std::endl;
        
    // }

    // std::cout<<planeCoefficients << std::endl;

    //publish planeMask
    // sensor_msgs::Image plane_image;
    // cv_bridge::CvImage cv_image;
    // cv_image.image = colorMask;
    // cv_image.encoding = "bgr8";
    // cv_image.toImageMsg(plane_image);
    // image_plane_pub.publish(plane_image);




    // sensor_msgs::PointCloud2 cloud_msg;
    // cloud_msg.header.frame_id = "base_link";
    // det::Mat2ROSMsg(opencv_pcd, cloud_msg);


 

    // std::string matType = type2str(opencv_pcd.type());
    // std::cout << "Matrix: " << matType << std::endl;
    // std::cout << "colorLUT: " << type2str(colorLUT.type()) << std::endl;
    // std::cout << "planeMask: " << type2str(planeMask.type()) << std::endl;

    // step1 depth filter
    // cv::Mat depthMapFloat;
    // depth_image.convertTo(depthMapFloat, CV_32F);
    




    // step2 CreateAngleImage

    // depth2pcd(depth_image, cloud, 20);
    // depth2pcd_old(depth_image, cloud);
    
    
     
        // r2dio(depth_image, cloud);
        // depth2pcd(depth_image, cloud, 2);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    // timer = new Timer("depth2pcd");
    //     depth2pcd(depth_image);
    // delete timer;
    
    // timer = new Timer("depth2pcd");

    // pcl::IndicesPtr indices(new std::vector<int>);
        // cloudFromDepthRGB(filteredImage, indices.get());
        // cloud->clear();
        // cloud->height = image_height/decimation;
        // cloud->width  = image_width/decimation;
        // cloud->is_dense = false;
        // cloud->resize(image_width * image_height);
        // depth2pcd(filteredImage);
    // delete timer;

   
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud(cloud); // 设置输入点云
        // sor.setMeanK(50); // 设置在进行统计时考虑的临近点个数
        // sor.setStddevMulThresh(1.0); // 设置判断离群点的阈值（以标准差的倍数表示）
        // sor.filter(*cloud_filtered);



    // pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // ne.setInputCloud(cloud);
    // ne.compute(*cloud_normals);

    // pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
    // mps.setMinInliers(100); // 设置最小分割大小
    // mps.setInputNormals(cloud_normals);
    // mps.setInputCloud(cloud);

        // pcl::PassThrough<pcl::PointXYZ> pass;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
        // pass.setInputCloud (cloud_out);			
        // pass.setFilterFieldName ("z");		
        // pass.setFilterLimits (-0.4, FLT_MAX);		
        // //pass.setFilterLimitsNegative (true);//是否反向过滤，默认为false
        // pass.filter (*cloud_passthrough);
    
    //cloud.size 
    

    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud, output);
    // output.header.frame_id = "base_link"; // 设置坐标系
    // output.header.stamp = depth_msg->header.stamp;




    // pcl_pub.publish(output);
    // pcl_pub.publish(cloud_msg); 

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_cluster_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    pcl_pub = n.advertise<sensor_msgs::PointCloud2>("pcl_output", 10);
    image_plane_pub = n.advertise<sensor_msgs::Image>("plane_image", 10);


    ros::Subscriber sub_depth = n.subscribe("/camera/aligned_depth_to_color/image_raw", 100, depth_callback); 
    
    int num_procs = omp_get_num_procs();


    std::string depth_path;
    n.getParam("depth_path", depth_path);
    n.getParam("fx", fx);
    n.getParam("fy", fy);
    n.getParam("cx", cx);
    n.getParam("cy", cy);
    n.getParam("depth_factor", depth_factor);
    n.getParam("image_width", image_width);
    n.getParam("image_height", image_height);
    std::cout<<"image_width: " << image_width << std::endl;
    std::cout<<"image_height: " << image_height << std::endl;

    precision_control = std::pow(10.0, 2); 
    int pcd_num= image_width * image_height;

    cloud->height = image_height/decimation;
    cloud->width  = image_width/decimation;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    //输出cloud->height和cloud->width的类型

    std::cout << "cloud->height: " << boost::typeindex::type_id_with_cvr<decltype(cloud->height)>().pretty_name() << std::endl;
    std::cout << "cloud->width: " << boost::typeindex::type_id_with_cvr<decltype(cloud->width)>().pretty_name() << std::endl;

    // cloud->points.resize(pcd_num);
    // cloud->width = pcd_num;
    // cloud->height = 1;
    // cloud->is_dense = false; 

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