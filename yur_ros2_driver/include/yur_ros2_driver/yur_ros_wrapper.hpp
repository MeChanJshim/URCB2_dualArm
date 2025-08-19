#pragma once

/* Basic headers */
#include "yur_ros2_driver/ur_driver.h"
#include "yur_ros2_driver/do_output.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <time.h>

/* ROS2 headers */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class YRos2Wrapper : public rclcpp::Node {
protected:
    /* UR driver instances & parameters */
    UrDriver robot_;
    std::condition_variable rt_msg_cond_;
    std::condition_variable msg_cond_;
    double io_flag_delay_;
    std::vector<double> joint_offsets_;

    /* ROS2 Subscribers */
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Yjointcmd_sub_;

    /* ROS2 Publishers */
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    /* Thread instances */
    std::thread* mb_publish_thread_; // thread를 바로 실행시키지 않기 위해 포인터 사용
    std::thread* rt_publish_thread_;

    /* Robot name */
    std::string robot_name_;

public:
    /* Constructor */
    YRos2Wrapper(std::string host, int reverse_port, std::string robot_name);

    /* Destructor */
    ~YRos2Wrapper();

    /* Halt function */
    void halt();

private:
    /* Thread locker */
    std::mutex RT_msg_lock; // The values are locked for reading in the class, so just use a dummy mutex
    std::mutex Mb_msg_lock; // The values are locked for reading in the class, so just use a dummy mutex

    /* Subscriber functions */
    void jointcmdCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    /* Real-time state publisher function */
    void publishRTMsg();
    void publishMbMsg();
};