#include "yur_ros2_driver/yur_ros_wrapper.hpp"

YRos2Wrapper::YRos2Wrapper(std::string host, int reverse_port, std::string robot_name) :
    Node(robot_name),
    robot_(rt_msg_cond_, msg_cond_, host, reverse_port, 0.03, 300), 
    io_flag_delay_(0.05), 
    joint_offsets_(6, 0.0),
    robot_name_(robot_name),
    mb_publish_thread_(nullptr),
    rt_publish_thread_(nullptr)
{
    /* Start robot */
    if (robot_.start()) { // Pasing the code (Upload the ur communication code to UR)


        RCLCPP_INFO(this->get_logger(), "Robot started successfully");
        
        /* Joint command subscriber from motion node */
        std::string joint_target_topic = robot_name_ + "/targetJ";

        Yjointcmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            joint_target_topic, 1, 
            std::bind(&YRos2Wrapper::jointcmdCB, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "\033[32mit can be executed by the topic: \"%s\" \033[0m", joint_target_topic.c_str());

        /* Joint state publisher */
        std::string joint_state_topic = robot_name_ + "/joint_states";
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 1);
        RCLCPP_INFO(this->get_logger(), "\033[32mReal-time joint state is published by the topic: \"%s\" \033[0m", joint_state_topic.c_str());

        /* Real-time state publisher thread generation */       
        rt_publish_thread_ = new std::thread(std::bind(&YRos2Wrapper::publishRTMsg, this));
        mb_publish_thread_ = new std::thread(std::bind(&YRos2Wrapper::publishMbMsg, this));

        RCLCPP_INFO(this->get_logger(), "\033[31mReal-time state publishing thread was generated \033[0m");
    }
}

YRos2Wrapper::~YRos2Wrapper() {
    halt();
}

void YRos2Wrapper::halt() {
    robot_.halt();
    if (rt_publish_thread_) {
        rt_publish_thread_->join();
        delete rt_publish_thread_;
        rt_publish_thread_ = nullptr;
    }
    if (mb_publish_thread_) {
        mb_publish_thread_->join();
        delete mb_publish_thread_;
        mb_publish_thread_ = nullptr;
    }
}

/* Transmit the real-time joint command to robot */
void YRos2Wrapper::jointcmdCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    robot_.servoj(msg->data);
}

/* real-time state publishing */
void YRos2Wrapper::publishRTMsg() {

    sensor_msgs::msg::JointState joint_msg;

    while (rclcpp::ok()) {

        /* Parameter declaration */  
        joint_msg.name = robot_.getJointNames();
        std::unique_lock<std::mutex> locker(RT_msg_lock);

        /* Wait until data publishing from UR */
        while (!robot_.rt_interface_->robot_state_->getDataPublished()) {
            rt_msg_cond_.wait(locker);
        }

        /* Get the joint angles */
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.position = robot_.rt_interface_->robot_state_->getQActual();

        /* Add the joint offset */
        for (unsigned int i = 0; i < joint_msg.position.size(); i++) {
            joint_msg.position[i] += joint_offsets_[i];
        }
        
        /* Get joint velocities */
        joint_msg.velocity = robot_.rt_interface_->robot_state_->getQdActual();
        joint_msg.effort = robot_.rt_interface_->robot_state_->getIActual();

        /* Publish joint states */
        joint_pub_->publish(joint_msg);

        robot_.rt_interface_->robot_state_->setDataPublished();
    }
}

void YRos2Wrapper::publishMbMsg() {
    bool warned = false;
    printf("hello Mb~~");
    while (rclcpp::ok()) {

        std::unique_lock<std::mutex> locker(Mb_msg_lock);
        while (!robot_.sec_interface_->robot_state_->getNewDataAvailable()) {
            msg_cond_.wait(locker);
        }

        if (robot_.sec_interface_->robot_state_->isEmergencyStopped()
                or robot_.sec_interface_->robot_state_->isProtectiveStopped()) {
            if (robot_.sec_interface_->robot_state_->isEmergencyStopped()
                    and !warned) {
                print_error("Emergency stop pressed!");
            } else if (robot_.sec_interface_->robot_state_->isProtectiveStopped()
                    and !warned) {
                print_error("Robot is protective stopped!");
            }

            warned = true;
        } else
            warned = false;

        robot_.sec_interface_->robot_state_->finishedReading();
    }
}