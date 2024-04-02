#pragma once
#include <ros/ros.h>
#include <chrono>

class Timer
{
  public:
    Timer(const std::string& name = "Timer") : name(name), start(std::chrono::high_resolution_clock::now()) {}


    ~Timer() 
    {
        end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end - start).count(); 

        // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        ROS_WARN("%s processing costs: %.3f ms", name.c_str(), duration);

    }

    // void tic()
    // {
    //     start = std::chrono::system_clock::now();
    // }

    // double toc()
    // {
    //     end = std::chrono::system_clock::now();

    //     auto duration = std::chrono::duration<double, std::milli>(end - start).count(); 

    //     stopped = true;

    //     return duration;
        
    // }

  private:
    // std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::string name;

};