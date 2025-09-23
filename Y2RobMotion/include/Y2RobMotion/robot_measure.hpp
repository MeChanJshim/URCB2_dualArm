#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <fstream>
#include <memory>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <filesystem>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <array>
#include <cctype>
#include <functional>  // for std::bind

// Constants
constexpr int NUMBER_OF_JOINTS = 6;
constexpr int ROBOT_HZ = 125;
constexpr double MEASURE_PERIOD = 1.0 / (ROBOT_HZ * 5);  // To avoid aliasing

enum class Mode { Continuous, Discrete };


class UrMeasure
{
public:
    explicit UrMeasure(Mode mode,
                       const rclcpp::Node::SharedPtr& node,
                       const std::string& robot_name, const std::string& basic_dir);
    ~UrMeasure();

private:
    // Node (injected) & settings
    rclcpp::Node::SharedPtr node_;
    std::string robot_name_;
    std::string basic_dir_;
    Mode mode_;

    // File handles
    std::unique_ptr<std::ofstream> cp_file_;
    std::unique_ptr<std::ofstream> cj_file_;
    std::unique_ptr<std::ofstream> cf_file_;
    std::unique_ptr<std::ofstream> tp_file_;
    std::unique_ptr<std::ofstream> tj_file_;
    std::unique_ptr<std::ofstream> tf_file_;

    // File Names
    std::string CP_PATH = "/currentP.txt";
    std::string CJ_PATH = "/currentJ.txt";
    std::string CF_PATH = "/currentF.txt";
    std::string TP_PATH = "/targetP.txt";
    std::string TJ_PATH = "/targetJ.txt";
    std::string TF_PATH = "/targetF.txt";

    // Latest data buffers (for discrete snapshot)
    std::array<double, NUMBER_OF_JOINTS> last_cj_{};
    std::array<double, 6> last_cp_{};
    std::array<double, 6> last_cf_{};
    std::array<double, NUMBER_OF_JOINTS> last_tj_{};
    std::array<double, 6> last_tp_{};
    std::array<double, 6> last_tf_{};
    bool have_cj_{false}, have_cp_{false}, have_cf_{false},
         have_tj_{false}, have_tp_{false}, have_tf_{false};
    std::mutex buf_mtx_;

    // ROS2 Subscribers & Timer
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_j_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_f_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_j_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_p_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_f_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Discrete mode trigger & input thread
    std::atomic<bool> snap_trigger_{false};
    std::thread input_thread_;

    // Callbacks
    void currentJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void currentFCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targetJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targetPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targetFCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void timerCallback();

    // Helpers
    bool initializeFiles();
    void startInputThreadIfDiscrete();
    void writeContinuousCJ(const std_msgs::msg::Float64MultiArray::SharedPtr& msg);
    void writeContinuousCP(const std_msgs::msg::Float64MultiArray::SharedPtr& msg);
    void writeContinuousCF(const std_msgs::msg::Float64MultiArray::SharedPtr& msg);
    void writeContinuousTJ(const std_msgs::msg::Float64MultiArray::SharedPtr& msg);
    void writeContinuousTP(const std_msgs::msg::Float64MultiArray::SharedPtr& msg);
    void writeContinuousTF(const std_msgs::msg::Float64MultiArray::SharedPtr& msg);
    void writeDiscreteSnapshot();
};
