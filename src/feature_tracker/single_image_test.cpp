#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
// #include "tic_toc.h"



std::vector<cv::Point2f>    n_pts;
std::vector<int>            ids; 
std::vector<int>            track_cnt; 
std::vector<cv::Point2f>    prev_pts, cur_pts;
std::vector<cv::Point2f>    prev_un_pts, cur_un_pts;
std::map<int, cv::Point2f>  prev_un_pts_map, cur_un_pts_map;

int                         n_id;


cv::Mat CAMERA_MATRIX    =   (cv::Mat_<double>(3,3) << 675.585510, 0, 492.865662, 0, 676.195007, 269.670898, 0, 0, 1);
cv::Mat DIST_COEFFS      =   (cv::Mat_<double>(1,4) << 0.166439, -0.502598, 0.000923, 0.001173, 0.461488);

int maxCorners  =   200;
double minDistance  =   15;
cv::Mat mask = cv::Mat(540, 960, CV_8UC1, cv::Scalar(255));
cv::Mat cur_img;
cv::Mat prev_img;

void printvector(std::vector<cv::Point2f> &vec, std::string name) {
    std::cout << name << ":";
    for (auto val : vec) {
        std::cout << val << ",";
    }
    std::cout << std::endl;
}

void printvector(std::vector<int> &vec, std::string name) {
    std::cout << name << ":";
    for (auto val : vec) {
        std::cout << val << ",";
    }
    std::cout << std::endl;
}

void printvector(std::vector<uchar> &vec, std::string name) {
    std::cout << name << ":";
    for (auto val : vec) {
        std::cout << (int)val << ",";
    }
    std::cout << std::endl;
}

void printvector(std::map<int, cv::Point2f> &vec, std::string name) {
    std::cout << name << ":";
    for (auto val : vec) {
        std::cout <<  val.first << " " << val.second.x << " " << val.second.y << "     ";
    }
    std::cout << std::endl;
}


bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < 960 - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < 540 - BORDER_SIZE;
}

void findmaxinvector (std::vector<cv::Point2f> &vec) {
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();
    for(const auto& point : vec) {
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
    }       
    std::cout << "max_x: " << max_x << " max_y: " << max_y << std::endl;
}

void rejectWithF()
{
    std::vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
    std::vector<cv::Point2f> un_cur_pts_norm(cur_pts.size()), un_prev_pts_norm(prev_pts.size());

    cv::undistortPoints(cur_pts, un_cur_pts_norm, CAMERA_MATRIX, DIST_COEFFS);
    cv::undistortPoints(prev_pts, un_prev_pts_norm, CAMERA_MATRIX, DIST_COEFFS);
    cv::perspectiveTransform(un_cur_pts_norm, un_cur_pts, CAMERA_MATRIX); 
    cv::perspectiveTransform(un_prev_pts_norm, un_prev_pts, CAMERA_MATRIX); 
    // printvector(un_cur_pts, "un_cur_pts");
    // printvector(un_prev_pts, "un_prev_pts");
    std::vector<uchar> status;
    cv::findFundamentalMat(un_cur_pts,un_prev_pts, cv::FM_RANSAC, 1.0, 0.99,status);
    cv::findFundamentalMat(un_cur_pts,un_prev_pts, cv::FM_RANSAC, 1.0, 0.99,mask);

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "single_test");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    auto clahe = cv::createCLAHE(3.0, cv::Size(8, 8));



    //first image
    cv::Mat image1 = cv::imread("/root/downloads/bdr_image/day2/bdr_compare_longloop_static/1.png", cv::IMREAD_GRAYSCALE);
    clahe->apply(image1, image1);
    
    for (auto &n : track_cnt)
        n++;

    cv::goodFeaturesToTrack(image1, n_pts, maxCorners, 0.01, minDistance, mask);
    cv::Mat show_img1;
    cv::cvtColor(image1, show_img1, cv::COLOR_GRAY2RGB);//cv::COLOR_GRAY2RGB
    for (auto i = 0; i < n_pts.size(); i++) {
        cv::circle(show_img1, n_pts[i], 5, cv::Scalar(255, 0, 0), -1);
    }

    for (auto &p : n_pts)
    {
        cur_pts.push_back(p);
        ids.push_back(n_id++);// or ids.push_back(-1)
        track_cnt.push_back(1);
    }

    cur_un_pts.clear();
    cur_un_pts_map.clear();

    std::vector<cv::Point2f> undistortedNormalizedPoints;
    cv::undistortPoints(cur_pts, undistortedNormalizedPoints, CAMERA_MATRIX, DIST_COEFFS);
    cv::perspectiveTransform(undistortedNormalizedPoints, cur_un_pts, CAMERA_MATRIX); 
    for (auto i = 0; i < cur_pts.size(); i++)
    {
        cur_un_pts_map.insert(std::make_pair(ids[i],cur_un_pts[i]));
    }
    
    prev_img = image1.clone();
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;

    //*********************************************************second image
    cv::Mat image2 = cv::imread("/root/downloads/bdr_image/day2/bdr_compare_longloop_static/2.png", cv::IMREAD_GRAYSCALE);
    cur_img=image2.clone();
    clahe->apply(cur_img, cur_img);
    cur_pts.clear();
    cur_un_pts.clear();
    cur_un_pts_map.clear();

    std::vector<uchar> status;
    std::vector<float> err;

    if (prev_pts.size() > 0)
    {
      
        cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        
    }
    
    for (int i = 0; i < int(cur_pts.size()); i++)
    {
        if (status[i] && !inBorder(cur_pts[i]))
        {
            status[i] = 0;
            ROS_WARN("Point %d is out of border", i);
        }
    }

    // findmaxinvector(cur_pts);

    for (auto &n : track_cnt)
        n++;

    // printvector(track_cnt, "track_cnt");
    //  printvector(prev_pts, "prev_pts");
    
    rejectWithF();






    cv::Mat show_img2;
    cv::cvtColor(cur_img, show_img2, cv::COLOR_GRAY2RGB);//cv::COLOR_GRAY2RGB
    
    for(size_t i = 0; i < prev_pts.size(); i++) {
        if(status[i]) { // 如果该点成功跟踪
            // 绘制从初始点到跟踪点的线段
            // ROS_WARN("prev_pts[i]: %f %f", prev_pts[i].x, prev_pts[i].y);
            // ROS_WARN("cur_pts[i]: %f %f", cur_pts[i].x, cur_pts[i].y);
            cv::line(show_img2, prev_pts[i], cur_pts[i], cv::Scalar(0, 255, 0),10);
            // 在跟踪点位置绘制圆圈
            // cv::circle(show_img2, cur_pts[i], 3, cv::Scalar(0, 0, 255), -1);
        }
    }

    cv::imshow("feature", show_img1);
    cv::imshow("feature2", show_img2);
    cv::waitKey(0);
    ros::spin();
    return 0;
}