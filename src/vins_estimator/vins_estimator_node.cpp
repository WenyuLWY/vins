// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

//
#include <eigen3/Eigen/Dense>

//c++
#include <mutex>
#include <deque>
#include <condition_variable>

//
#include "parameters.h"
#include "visualization.h"
#include "point_tracker.h"
#include "estimator.h"
#include "utility.h"


ReadParameters params;
Visualization publishers;

double last_imu_t = 0;
std::mutex m_buf;
std::mutex m_state;

std::condition_variable con;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;

bool init_imu = true;
double latest_time=0;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;


void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = false;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};
    Eigen::Vector3d g(0,0,9.8);
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder! %f",imu_msg->header.stamp.toSec() );
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    {
        std::lock_guard<std::mutex> lg(m_state);
        //predict imu (no residual error)
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "map";
        // if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        publishers.pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);//Debug Info

    params.init(n);
    // point_tracker.init(params);

    // params.paramList();

    ROS_WARN("waiting for image and imu...");

    publishers.registerPub(n);

    // ros::Subscriber sub_image = n.subscribe(params.IMAGE_TOPIC, 100, image_callback);
    ros::Subscriber sub_imu = n.subscribe(params.IMU_TOPIC , 2000, imu_callback, ros::TransportHints().tcpNoDelay());

    ros::spin();
    return 0;
}