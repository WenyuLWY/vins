#pragma once

#include <ros/ros.h>
#include <thread>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

class parametersaafs {
    public:
        // gloabl
        bool PUB_THIS_FRAME;
        int WINDOW_SIZE;

        // load
        int NUM_THREADS;

        std::string IMU_TOPIC;
        std::string IMAGE_TOPIC;
        std::string DEPTH_TOPIC;
        std::string OUTPUT_PATH;
        std::string EX_CALIB_RESULT_PATH;
        std::string VINS_RESULT_PATH;

        double DEPTH_MIN_DIST;
        double DEPTH_MAX_DIST;
            
        float fx,fy,cx,cy,k1,k2,p1,p2,k3;
        cv::Mat CAMERA_MATRIX;
        cv::Mat DIST_COEFFS; 
        int ROW, COL;

        int    ESTIMATE_EXTRINSIC;
        std::string EXTRINSICTRANSLATION;
        std::string EXTRINSICROTATION;
        std::vector<Eigen::Matrix3d> RIC;
        std::vector<Eigen::Vector3d> TIC;

        int MAX_CNT;
        int MIN_DIST;
        int FREQ;
        double F_THRESHOLD;
        int SHOW_TRACK;
        int EQUALIZE;
        

        int ESTIMATE_TD;
        double  TD;
        int ROLLING_SHUTTER;
        double  TR;
        
        double SOLVER_TIME;
        int NUM_ITERATIONS;
        double MIN_PARALLAX;

        double ACC_N, ACC_W, GYR_N, GYR_W;
        Eigen::Vector3d  G{0.0, 0.0, 9.8};

        ReadParameters() {
        
        }

        void init(ros::NodeHandle& n)
        {

            PUB_THIS_FRAME = false;
            WINDOW_SIZE  = 10;
            
            n.getParam("num_threads", NUM_THREADS);
            if (NUM_THREADS <= 1)
            {
                NUM_THREADS = std::thread::hardware_concurrency();
            }

            n.getParam("imu_topic", IMU_TOPIC);
            n.getParam("image_topic", IMAGE_TOPIC);
            
            n.getParam("output_path", OUTPUT_PATH);
            VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.txt";
            std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
            fout.close();

            n.getParam("depth_topic", DEPTH_TOPIC);
            n.getParam("output_path", OUTPUT_PATH);

            n.getParam("depth_min_dist", DEPTH_MIN_DIST);
            n.getParam("depth_max_dist", DEPTH_MAX_DIST);

            n.getParam("fx", fx);
            n.getParam("fy", fy);
            n.getParam("cx", cx);
            n.getParam("cy", cy);
            n.getParam("k1", k1);
            n.getParam("k2", k2);
            n.getParam("p1", p1);
            n.getParam("p2", p2);
            n.getParam("k3", k3);
            CAMERA_MATRIX = (cv::Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
            DIST_COEFFS = (cv::Mat_<float>(1,5) << k1, k2, p1, p2, k3);

            n.getParam("image_width", COL);
            n.getParam("image_height", ROW);

            n.getParam("estimate_extrinsic", ESTIMATE_EXTRINSIC); 
            switch (ESTIMATE_EXTRINSIC)
            {
                case 2:   
                    ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
                    RIC.emplace_back(Eigen::Matrix3d::Identity());
                    TIC.emplace_back(Eigen::Vector3d::Zero());
                    EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.txt";
                    break;
                case 1:
                    ROS_WARN("Optimize extrinsic param around initial guess!");
                    EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.txt";
                    break;
                case 0:
                    ROS_WARN("fix extrinsic param!");
                    break;
                default:
                    ROS_ERROR("WRONG EXTRINSIC PARAMETER");
            }

            n.getParam("extrinsicTranslation", EXTRINSICTRANSLATION);
            Eigen::Vector3d eigen_T;
            std::istringstream iss(EXTRINSICTRANSLATION);
            std::string value;
            std::vector<double> values;
            while (std::getline(iss, value, ',')) {
                values.push_back(std::stod(value));
            }
            eigen_T = Eigen::Map<Eigen::Vector3d>(values.data(),3,1);
            TIC.push_back(eigen_T);
            
            values.clear();
            iss.clear();

            n.getParam("extrinsicRotation", EXTRINSICROTATION);
            Eigen::Matrix3d eigen_R;
            iss.str(EXTRINSICROTATION);
            while (std::getline(iss, value, ',')) {
                values.push_back(std::stod(value));
            }
            eigen_R = Eigen::Map<Eigen::Matrix3d>(values.data(),3,3);
            Eigen::Quaterniond Q(eigen_R);
            eigen_R = Q.normalized();
            RIC.push_back(eigen_R);
            

            n.getParam("max_cnt", MAX_CNT);
            n.getParam("min_dist", MIN_DIST);
            n.getParam("freq", FREQ);
            if (FREQ == 0)
                FREQ = 100;
            n.getParam("F_threshold", F_THRESHOLD);
            n.getParam("show_track", SHOW_TRACK);
            n.getParam("equalize", EQUALIZE);

            n.getParam("max_solver_time", SOLVER_TIME);
            n.getParam("max_num_iterations", NUM_ITERATIONS);
            n.getParam("keyframe_parallax", MIN_PARALLAX);
            MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

            n.getParam("acc_n", ACC_N);
            n.getParam("acc_w", ACC_W);
            n.getParam("gyr_n", GYR_N);
            n.getParam("gyr_w", GYR_W);
            n.getParam("g_norm", G.z());



            n.getParam("estimate_td", ESTIMATE_TD);
            n.getParam("td", TD);
            if (ESTIMATE_TD)
            {
                ROS_WARN("Unsynchronized sensors, online estimate time offset, initial td: %f", TD);
            }
            else
            {
                ROS_WARN("Synchronized sensors, fix time offset: %f", TD);
            }
                
            n.getParam("rolling_shutter", ROLLING_SHUTTER);
            if (ROLLING_SHUTTER)
            {
                n.getParam("rolling_shutter_tr", TR);
                ROS_WARN("rolling shutter camera, read out time per line: %f", TR);
            }
            else
            {
                TR = 0;
            }
        }
        void paramList()
        {
            std::cout << "NUM_THREADS: " << NUM_THREADS << std::endl;
            std::cout << "IMU_TOPIC: " << IMU_TOPIC << std::endl;
            std::cout << "IMAGE_TOPIC: " << IMAGE_TOPIC << std::endl;
            std::cout << "OUTPUT_PATH: " << OUTPUT_PATH << std::endl;
            std::cout << "DEPTH_TOPIC: " << DEPTH_TOPIC << std::endl;
            std::cout << "VINS_RESULT_PATH: " << VINS_RESULT_PATH << std::endl;
            std::cout << "EX_CALIB_RESULT_PATH: " << EX_CALIB_RESULT_PATH << std::endl;
            std::cout << "extrinsicTranslation: " << TIC[0].transpose()  << std::endl;
            std::cout << "extrinsicRotation: " << RIC[0] << std::endl;
            std::cout << "TD: " << TD << std::endl;
            std::cout << "TR: " << TR << std::endl;
        }


private:
    double FOCAL_LENGTH = 460.0; //shan:What's this?---seems a virtual focal used in rejectWithF.
    
    // int    NUM_OF_CAM   = 1;
    // int    NUM_OF_F     = 1000;
    // double INIT_DEPTH = 5.0;
    double BIAS_ACC_THRESHOLD = 0.1;
    // double BIAS_GYR_THRESHOLD = 0.1;

};
