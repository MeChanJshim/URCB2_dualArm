#pragma once
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <fstream>
#include <memory>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <filesystem>
#include <iostream>

// Constants
constexpr char NISTDUALUR_PATH[] = "/home/dual_ur/ros2_ws/src/nist_dualUR";
constexpr char CP_PATH[] = "/measured/currentP.txt";
constexpr char CJ_PATH[] = "/measured/currentJ.txt";
constexpr char TP_PATH[] = "/measured/targetP.txt";
constexpr char TJ_PATH[] = "/measured/targetJ.txt";
constexpr char ROBOT_MODEL[] = "UR10_right";
constexpr int NUMBER_OF_JOINTS = 6;
constexpr int UR10_HZ = 125;
constexpr double MEASURE_PERIOD = 1.0 / (UR10_HZ * 5);  // To avoid aliasing

class UrMeasure : public rclcpp::Node
{
public:
    explicit UrMeasure(const std::string& robot_name = ROBOT_MODEL);
    ~UrMeasure();
    
    void run();

private:
    // Robot name
    std::string robot_name_;
    
    // File handles
    std::unique_ptr<std::ofstream> cp_file_;
    std::unique_ptr<std::ofstream> cj_file_;
    std::unique_ptr<std::ofstream> tp_file_;
    std::unique_ptr<std::ofstream> tj_file_;
    
    // ROS2 Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_j_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_p_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_j_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_p_sub_;
    
    // Timer for main loop
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Callback functions
    void currentJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targetJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void targetPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    
    // Timer callback
    void timerCallback();
    
    // Helper function to initialize files
    bool initializeFiles();
};

// Constructor
UrMeasure::UrMeasure(const std::string& robot_name)
    : Node("ur_measure_node"), robot_name_(robot_name)
{
    // Initialize files
    if (!initializeFiles()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize files. Exiting.");
        rclcpp::shutdown();
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "\033[32mRecording started...\033[0m");
    
    // Setup topic names
    std::string current_j_topic = robot_name_ + "_currentJ";
    std::string current_p_topic = robot_name_ + "_currentP";
    std::string target_j_topic = robot_name_ + "_targetJ";
    std::string target_p_topic = robot_name_ + "_targetP";
    
    // Initialize subscribers
    current_j_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_j_topic, 10,
        std::bind(&UrMeasure::currentJCallback, this, std::placeholders::_1));
    
    current_p_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_p_topic, 10,
        std::bind(&UrMeasure::currentPCallback, this, std::placeholders::_1));
    
    target_j_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_j_topic, 10,
        std::bind(&UrMeasure::targetJCallback, this, std::placeholders::_1));
    
    target_p_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        target_p_topic, 10,
        std::bind(&UrMeasure::targetPCallback, this, std::placeholders::_1));
    
    // Create timer for main loop (equivalent to ros::Rate)
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(MEASURE_PERIOD),
        std::bind(&UrMeasure::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "UrMeasure node initialized for robot: %s", robot_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Measurement frequency: %.1f Hz", 1.0 / MEASURE_PERIOD);
}

// Destructor
UrMeasure::~UrMeasure()
{
    // Close files safely
    if (cp_file_ && cp_file_->is_open()) {
        cp_file_->close();
    }
    if (cj_file_ && cj_file_->is_open()) {
        cj_file_->close();
    }
    if (tp_file_ && tp_file_->is_open()) {
        tp_file_->close();
    }
    if (tj_file_ && tj_file_->is_open()) {
        tj_file_->close();
    }
    
    RCLCPP_INFO(this->get_logger(), "\033[31mProgram was terminated\033[0m");
}

// Initialize files
bool UrMeasure::initializeFiles()
{
    std::string nist_ur_path(NISTDUALUR_PATH);
    
    // Create measured directory if it doesn't exist
    std::string measured_dir = nist_ur_path + "/measured";
    if (!std::filesystem::exists(measured_dir)) {
        try {
            std::filesystem::create_directories(measured_dir);
            RCLCPP_INFO(this->get_logger(), "Created directory: %s", measured_dir.c_str());
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s", 
                        measured_dir.c_str(), e.what());
            return false;
        }
    }
    
    // Initialize file streams
    try {
        cp_file_ = std::make_unique<std::ofstream>(nist_ur_path + CP_PATH);
        cj_file_ = std::make_unique<std::ofstream>(nist_ur_path + CJ_PATH);
        tp_file_ = std::make_unique<std::ofstream>(nist_ur_path + TP_PATH);
        tj_file_ = std::make_unique<std::ofstream>(nist_ur_path + TJ_PATH);
        
        if (!cp_file_->is_open() || !cj_file_->is_open() || 
            !tp_file_->is_open() || !tj_file_->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening one or more files.");
            return false;
        }
        
        // Set precision for output files
        cp_file_->precision(3);
        cj_file_->precision(3);
        tp_file_->precision(3);
        tj_file_->precision(3);
        
        cp_file_->setf(std::ios::fixed);
        cj_file_->setf(std::ios::fixed);
        tp_file_->setf(std::ios::fixed);
        tj_file_->setf(std::ios::fixed);
        
        RCLCPP_INFO(this->get_logger(), "All measurement files opened successfully.");
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while opening files: %s", e.what());
        return false;
    }
}

// Callback functions
void UrMeasure::currentJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!cj_file_ || !cj_file_->is_open()) return;
    
    for (int i = 0; i < NUMBER_OF_JOINTS && i < static_cast<int>(msg->data.size()); i++) {
        *cj_file_ << msg->data[i] << "\t";
    }
    *cj_file_ << "\n";
    cj_file_->flush();  // Ensure data is written immediately
}

void UrMeasure::currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!cp_file_ || !cp_file_->is_open()) return;
    
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++) {
        *cp_file_ << msg->data[i] << "\t";
    }
    *cp_file_ << "\n";
    cp_file_->flush();
}

void UrMeasure::targetJCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!tj_file_ || !tj_file_->is_open()) return;
    
    for (int i = 0; i < NUMBER_OF_JOINTS && i < static_cast<int>(msg->data.size()); i++) {
        *tj_file_ << msg->data[i] << "\t";
    }
    *tj_file_ << "\n";
    tj_file_->flush();
}

void UrMeasure::targetPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!tp_file_ || !tp_file_->is_open()) return;
    
    for (int i = 0; i < 6 && i < static_cast<int>(msg->data.size()); i++) {
        *tp_file_ << msg->data[i] << "\t";
    }
    *tp_file_ << "\n";
    tp_file_->flush();
}

// Timer callback (replaces the while loop in run())
void UrMeasure::timerCallback()
{
    // This function is called at the specified frequency
    // All actual work is done in the subscription callbacks
    // This maintains the timing similar to the original ros::Rate
    
    // Optional: Add periodic status reporting
    static int counter = 0;
    counter++;
    
    // Report status every 10 seconds
    if (counter % static_cast<int>(10.0 / MEASURE_PERIOD) == 0) {
        RCLCPP_DEBUG(this->get_logger(), "Measurement running... (count: %d)", counter);
    }
}

// Run function (now much simpler due to timer-based approach)
void UrMeasure::run()
{
    RCLCPP_INFO(this->get_logger(), "Starting measurement loop...");
    
    // In ROS2, we don't need a manual while loop
    // The timer and callbacks handle everything
    rclcpp::spin(this->get_node_base_interface());
}

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto measure_node = std::make_shared<UrMeasure>(ROBOT_MODEL);
        
        // Use MultiThreadedExecutor for better performance
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(measure_node);
        
        RCLCPP_INFO(measure_node->get_logger(), "Starting UR measurement node...");
        
        // Run the node
        executor.spin();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}