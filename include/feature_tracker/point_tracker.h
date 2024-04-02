#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "parameters.h"

class PointTracker
{

public:
    PointTracker(){};
    ~PointTracker(){};


    // current,forward光流的前后帧,当前帧为foward. previous是上一帧 
    cv::Mat                     mask;
    double                      cur_time, prev_time;
    cv::Mat                     prev_img, cur_img;
    cv::Mat                     prev_depth, cur_depth;
    std::vector<cv::Point2f>    prev_pts, cur_pts;              //光流跟踪到的角点,像素坐标  
    std::vector<cv::Point2f>    prev_un_pts, cur_un_pts;        //去畸变后的归一化像素坐标
    std::map<int, cv::Point2f>  prev_un_pts_map, cur_un_pts_map;
    std::vector<cv::Point2f>    pts_velocity;
    std::vector<cv::Point2f>    n_pts;                          // 检测到的角点,与cur_pts合并
    /*
        ids和track_cnt都只是记录当前帧成功跟踪到的特征点
        如果没有被当前帧成功看到，那么这个点就会从ids和track_cnt中被剔除
    */
    std::vector<int>            ids;                            // ids：当有新的特征点进入时，特征点ID号+1，并push_back进入ids里面
    std::vector<int>            track_cnt;                      // track_cnt：记录特征点成功跟踪的次数.
    int                         n_id;                           // FEATURE_ID

    cv::Mat                     visOpticalFlowImage;            // 光流图像
    cv::Mat                     visPointImage;                  // 特征点图像
    cv::Mat                     visMask;


    void init(const ReadParameters& param)
    {
        EQUALIZE         =   param.EQUALIZE;
        PREDICTION      =   false;
        FLOW_BACK       =   false;
        MIN_DIST        =   param.MIN_DIST;
        MAX_CNT         =   param.MAX_CNT;
        F_THRESHOLD     =   param.F_THRESHOLD;
        COL             =   param.COL;
        ROW             =   param.ROW;
        CAMERA_MATRIX    =   param.CAMERA_MATRIX;
        DIST_COEFFS      =   param.DIST_COEFFS;
        n_id            =   0;
        WINDOW_SIZE     =   param.WINDOW_SIZE;
    }

    void trackImage(const cv::Mat &_img, const cv::Mat &_seg_image, double _cur_time)
    {

        cv::Mat img;
        cv::Mat seg_img = _seg_image;
        if (EQUALIZE)
        {
            auto start = std::chrono::high_resolution_clock::now();

            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe->apply(_img, img);

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration<double, std::milli>(end - start).count(); 
            // ROS_DEBUG("CLAHE costs: %fms", duration);
        }
        else
        {
            img = _img;
        }

        cur_time = _cur_time;
        cur_img = img;
        cur_pts.clear();
        cur_un_pts.clear();
        cur_un_pts_map.clear();

        if (prev_pts.size() > 0) //第二帧及之后的额外处理
        {

            std::vector<uchar> status;
            std::vector<float> err;


            if (PREDICTION)
            {
                // TO DO
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
            }
            else
            {
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
            }

            if (FLOW_BACK)
            {
                // TO DO
            }

            for (int i = 0; i < int(cur_pts.size()); i++)
                if (status[i] && !inBorder(cur_pts[i]))
                    status[i] = 0;

            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(cur_un_pts, status); 
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            visOpticalFlowImage=drawOpticalFlowImage(prev_img, cur_img, prev_pts, cur_pts, status);
        }

        //已经将跟踪失败的特征点进行剔除，剩下的特征点说明又被成功跟踪，那么跟踪次数依次+1.  track_cnt 每个元素+1
        for (auto &n : track_cnt)
            n++;

        if(1)//pub this frame / freq control
        {
            rejectWithF();
            setMask(seg_img);
            visMask = mask.clone();
            //以上是对被跟踪特征点的处理，下面是对新特征点的处理

            int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
            if (n_max_cnt > 0)
            {
                if(mask.empty())
                    std::cout << "mask is empty " << std::endl;
                if (mask.type() != CV_8UC1)
                    std::cout << "mask type wrong " << std::endl;
                cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
            }
            else
            {
                n_pts.clear();
            }
            // ROS_DEBUG("detect feature costs");
            addPoints();

        }
        undistortPoints();
        visPointImage=drawPointFeature(cur_img, cur_pts, track_cnt);

        prev_img = cur_img;
        prev_pts = cur_pts;
        prev_un_pts = cur_un_pts;
        prev_un_pts_map = cur_un_pts_map;
        prev_time = cur_time;
    }

    /*  getTrackImage
    在imgTrack图像上，以cur_pts[i]为中心画圆。圆的半径设置为2像素，圆的颜色根据len的值动态变化，
    其中红色和蓝色的强度由len的值决定，使得跟踪时间较长的点趋向于紫色，短的则趋向于红色。圆的线条粗细被设置为2像素。
    */

    cv::Mat getTrackImage()
    {
        return visPointImage;
    }
    cv::Mat getOpticalFlowImage()
    {
        return visOpticalFlowImage;
    }
    cv::Mat getMaskImage()
    {
        return visMask;
    }
    
    


private:

    int WINDOW_SIZE;
    int EQUALIZE;
    bool PREDICTION,FLOW_BACK;
    double F_THRESHOLD;
    int MIN_DIST;
    int MAX_CNT;
    int ROW, COL;
    cv::Mat CAMERA_MATRIX;
    cv::Mat DIST_COEFFS;


    bool inBorder(const cv::Point2f &pt)
    {
        const int BORDER_SIZE = 1;
        int img_x = cvRound(pt.x);
        int img_y = cvRound(pt.y);
        return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
    }

    void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

    void reduceVector(std::vector<int> &v, std::vector<uchar> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);            
    }

    void rejectWithF()
    {
        if (cur_pts.size() >= 8)
        {
            // ROS_DEBUG("FM ransac begins");
            
            std::vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
            std::vector<cv::Point2f> un_cur_pts_norm(cur_pts.size()), un_prev_pts_norm(prev_pts.size());
            

            cv::undistortPoints(cur_pts, un_cur_pts_norm, CAMERA_MATRIX, DIST_COEFFS);
            cv::undistortPoints(prev_pts, un_prev_pts_norm, CAMERA_MATRIX, DIST_COEFFS);
            cv::perspectiveTransform(un_cur_pts_norm, un_cur_pts, CAMERA_MATRIX); 
            cv::perspectiveTransform(un_prev_pts_norm, un_prev_pts, CAMERA_MATRIX); 

            //调用cv::findFundamentalMat对un_cur_pts和un_forw_pts计算F矩阵
            std::vector<uchar> status;
  
            cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);

            int size_a = cur_pts.size();
            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(cur_un_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);

            // ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);


            // ROS_DEBUG("FM ransac costs: %fms", duration);
        }
    }


    /*
    在给定的特征点集合中，根据追踪次数和特征点的空间分布（通过遮罩管理），筛选出一组优先级高且分布合理的特征点。
    */
    void setMask( const cv::Mat &_seg_image)
    {
        // ROS_DEBUG("set mask");
        /*
        这行代码创建了一个大小为row x col，类型为CV_8UC1（8位单通道，即灰度图像）的新图像（cv::Mat对象），
        并将所有像素值初始化为255（白色）。这个遮罩用于标记哪些区域的特征点可以被选取。
        */
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
        

        std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;


        /*
        这段代码将当前追踪的特征点（cur_pts），它们的追踪次数（track_cnt），和对应的ID（ids）组合成一个列表（cnt_pts_id）。
        这个列表中的每个元素都包含一个追踪次数、一个特征点坐标和一个ID。
        */
        for (unsigned int i = 0; i < cur_pts.size(); i++)
            cnt_pts_id.push_back(std::make_pair(track_cnt[i], std::make_pair(cur_pts[i], ids[i])));
        

        /*
        使用自定义比较函数，这段代码将cnt_pts_id中的元素按照追踪次数（track_cnt）降序排序。这意味着追踪次数最多的特征点会被放置在列表的前面。
        */    
        std::sort(
            cnt_pts_id.begin(), 
            cnt_pts_id.end(),
            [](const std::pair<int, std::pair<cv::Point2f, int>> &a, 
                const std::pair<int, std::pair<cv::Point2f, int>> &b) -> bool
            {
                return a.first > b.first;
            }
        );


        /*
        清空当前追踪的特征点列表、ID列表和追踪次数列表，为更新这些列表做准备。
        */
        cur_pts.clear();
        ids.clear();
        track_cnt.clear();


        /*
        遍历排序后的特征点列表。对于每个特征点，如果它位于遮罩的白色区域（即允许选取的区域），
        则将该特征点、其ID和追踪次数添加到相应的列表中，并在遮罩上以该特征点为中心画一个黑色的圆，
        圆的直径由MIN_DIST定义。这个过程旨在更新mask，防止选取过于接近的特征点，保证特征点之间有足够的空间分布。
        */
        for (auto &it : cnt_pts_id)
        {
            if (mask.at<uchar>(it.second.first) == 255)
            {
                cur_pts.push_back(it.second.first);
                ids.push_back(it.second.second);
                track_cnt.push_back(it.first);
                cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
            }
        }

        cv::bitwise_and(mask, _seg_image, mask); 
    }



    /*
    vins fusion中写在了trackImage函数中.通过ids.push_back(n_id++)更新id
    vins-mono写成单独的addPoints函数. 添加特征点时首先ids.push_back(-1),再使用通过updateID函数
    */
    void addPoints()
    {
        for (auto &p : n_pts)
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);
            track_cnt.push_back(1);
        }

    }
    
    void undistortPoints()
    {

        std::vector<cv::Point2f> cur_pts_norm;
        cv::undistortPoints(cur_pts, cur_pts_norm, CAMERA_MATRIX, DIST_COEFFS);
        cv::perspectiveTransform(cur_pts_norm, cur_un_pts, CAMERA_MATRIX); 
        for (auto i = 0; i < cur_pts.size(); i++)
        {
            cur_un_pts_map.insert(std::make_pair(ids[i],cur_un_pts[i]));
        }

        // caculate points velocity
        if (!prev_un_pts_map.empty())
        {
            double dt = cur_time - prev_time;
            pts_velocity.clear();
            for (auto i = 0; i < cur_un_pts.size(); i++)
            {
                if (ids[i] != -1)
                {
                    std::map<int, cv::Point2f>::iterator it;
                    it = prev_un_pts_map.find(ids[i]);
                    if (it != prev_un_pts_map.end())
                    {
                        double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                        double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                        pts_velocity.push_back(cv::Point2f(v_x, v_y));
                    }
                    else
                        pts_velocity.push_back(cv::Point2f(0, 0));
                }
                else
                {
                    pts_velocity.push_back(cv::Point2f(0, 0));
                }
            }
        }
        else
        {
            for (auto i = 0; i < cur_pts.size(); i++)
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }

    }

    cv::Mat drawOpticalFlowImage(const cv::Mat& prev_img, 
                                    const cv::Mat& cur_img, 
                                    const std::vector<cv::Point2f>& prev_pts, 
                                    const std::vector<cv::Point2f>& cur_pts, 
                                    const std::vector<uchar>& status)
    {
        cv::Mat vis_raw(cv::max(prev_img.rows, cur_img.rows), prev_img.cols + cur_img.cols, prev_img.type());
        prev_img.copyTo(vis_raw(cv::Rect(0, 0, prev_img.cols, prev_img.rows)));
        cur_img.copyTo(vis_raw(cv::Rect(prev_img.cols, 0, cur_img.cols, cur_img.rows)));
        cv::Mat vis;
        cv::cvtColor(vis_raw, vis, cv::COLOR_GRAY2RGB);
        for (size_t i = 0; i < prev_pts.size(); i++) 
        {
            if (status[i]) {
                cv::Point2f pt_prev = prev_pts[i];
                cv::Point2f pt_cur = cur_pts[i] + cv::Point2f(static_cast<float>(prev_img.cols), 0.0f);

                cv::line(vis, pt_prev, pt_cur, cv::Scalar(0, 255, 0));
                cv::circle(vis, pt_prev, 5, cv::Scalar(0, 0, 255), -1);
                cv::circle(vis, pt_cur, 5, cv::Scalar(255, 0, 0), -1);
            }
        }
        return vis;
    }

    cv::Mat drawPointFeature(const cv::Mat& cur_img, 
                                const std::vector<cv::Point2f>& cur_pts, 
                                const std::vector<int>& track_cnt)
    {
        cv::Mat vis;
        cv::cvtColor(cur_img, vis, cv::COLOR_GRAY2RGB);

        for (int i = 0; i < cur_pts.size(); i++)
        {
            double len = std::min(1.0, 1.0 * track_cnt[i] / WINDOW_SIZE);
            cv::circle(vis, cur_pts[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        }
        return vis;
    }

};