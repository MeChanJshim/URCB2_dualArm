#pragma once
#include "Y2RobMotion/robot_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <iostream>
#include <thread>

#define RARM_name "UR10_right"
#define LARM_name "UR10_left"

#define RARM_freq 125
#define LARM_freq 125

class dualArm_cmd : public rclcpp::Node
{
public:
    dualArm_cmd(const std::string& node_name, const std::string& RArm_name, const std::string& LArm_name,
    int RArm_period, int LArm_period, PathGenParam& RArm_pgParam, PathGenParam& LArm_pgParam);
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mimic_master_pub_;
    std_msgs::msg::String mimic_master_msg_;
    
    void pose_syn(std::string& master_name_, const double move_vel);

    bool is_robInit(){return (rightArm_cmd->robot_init && leftArm_cmd->robot_init);}
    YMatrix RArm_PTP_command_input(){return rightArm_cmd->PTP_command_input();}
    YMatrix LArm_PTP_command_input(){return leftArm_cmd->PTP_command_input();}

    void RArm_PTP_command_gen(const YMatrix& loaded_motion){rightArm_cmd->PTP_command_gen(loaded_motion);}
    void LArm_PTP_command_gen(const YMatrix& loaded_motion){leftArm_cmd->PTP_command_gen(loaded_motion);}


private:
    /* Generate hip instances*/
    std::unique_ptr<robot_command> rightArm_cmd;
    std::unique_ptr<robot_command> leftArm_cmd;
};

dualArm_cmd::dualArm_cmd(const std::string& node_name, const std::string& RArm_name, const std::string& LArm_name,
int RArm_period, int LArm_period, PathGenParam& RArm_pgParam, PathGenParam& LArm_pgParam)
: Node(node_name)
{
    /* Initialize hip instance of arm_cmd */
    rightArm_cmd = std::make_unique<robot_command>(this,RArm_name,RArm_period,RArm_pgParam);
    leftArm_cmd = std::make_unique<robot_command>(this,LArm_name,LArm_period,LArm_pgParam);

    mimic_master_pub_ = this->create_publisher<std_msgs::msg::String>("Mimic_Master", 10);
    
    RCLCPP_INFO(this->get_logger(), "DualUR command node initialized");
}

void dualArm_cmd::pose_syn(std::string& master_name_, const double move_vel)
{
    std::vector<double> master_pose;
    if ((master_name_ == rightArm_cmd->robot_name_) || (master_name_ == leftArm_cmd->robot_name_))
    {
        master_pose = (master_name_ == rightArm_cmd->robot_name_) ? rightArm_cmd->current_position : leftArm_cmd->current_position;
        YMatrix Loaded_motion = {{master_pose[0], master_pose[1], master_pose[2], master_pose[3], master_pose[4], master_pose[5]}};
        
        /* If the master is right arm */
        if (master_name_ == rightArm_cmd->robot_name_)
        {
            printf("\033[31m%s is transferred to %s \033[0m\n", leftArm_cmd->robot_name_.c_str(), rightArm_cmd->robot_name_.c_str());
            leftArm_cmd->pg_param.ptp_target_velocity = move_vel;
            // leftArm_cmd.PTP_command_gen(Loaded_motion);
            LArm_PTP_command_gen(Loaded_motion);
            printf("\033[30m%s was synchronized with %s \033[0m\n", leftArm_cmd->robot_name_.c_str(), rightArm_cmd->robot_name_.c_str());
        }
        /* If the master is left arm */
        else
        {
            printf("\033[31m%s is transferred to %s \033[0m\n", rightArm_cmd->robot_name_.c_str(), leftArm_cmd->robot_name_.c_str());
            rightArm_cmd->pg_param.ptp_target_velocity = move_vel;
            // rightArm_cmd.PTP_command_gen(Loaded_motion);
            RArm_PTP_command_gen(Loaded_motion);
            printf("\033[31m%s was synchronized with %s \033[0m\n", rightArm_cmd->robot_name_.c_str(), leftArm_cmd->robot_name_.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Wrong master name: %s", master_name_.c_str());
    }
}

int main(int argc, char** argv)
{
    PathGenParam rArm_pg_param, lArm_pg_param;
    /* Right Arm path generation parameter */
    rArm_pg_param.initialTransferSpeed = 5.0;      // mm/s
    rArm_pg_param.angularVelocityLimit = 5.0;      // degrees/s
    rArm_pg_param.accelerationTime = 1.0;          // seconds
    rArm_pg_param.startingTime = 2.0;              // seconds
    rArm_pg_param.lastRestingTime = 2.0;           // seconds
    rArm_pg_param.ptp_target_velocity = 0.5;       // degrees/s
    rArm_pg_param.loadFileType = "cmd_6D";    // cmd_6D, cmd_9D, cmd_continue6D, cmd_continue9D

    /* Left Arm path generation parameter */
    lArm_pg_param.initialTransferSpeed = 5.0;      // mm/s
    lArm_pg_param.angularVelocityLimit = 5.0;      // degrees/s
    lArm_pg_param.accelerationTime = 1.0;          // seconds
    lArm_pg_param.startingTime = 2.0;              // seconds
    lArm_pg_param.lastRestingTime = 2.0;           // seconds
    lArm_pg_param.ptp_target_velocity = 0.5;       // degrees/s
    lArm_pg_param.loadFileType = "cmd_6D";    // cmd_6D, cmd_9D, cmd_continue6D, cmd_continue9D

    rclcpp::init(argc, argv);
    
    try {
        /* Generate Node */
        auto dual_arm_cmd = std::make_shared<dualArm_cmd>("dualArm_cmd", RARM_name, LARM_name,
        1.0/static_cast<double>(RARM_freq), 1.0/static_cast<double>(LARM_freq), rArm_pg_param, lArm_pg_param);
        
        // Create executor for spinning
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(dual_arm_cmd);
        
        // Start executor in a separate thread
        std::thread executor_thread([&executor]() {
            executor.spin();
        });
        
        YMatrix Loaded_motion(1, 6);
        
        while (rclcpp::ok()) {
            if (dual_arm_cmd -> is_robInit())
            {
                printf("\033[33mEnter command mode (Single: 0, Multi: 1, 9999:Quit): \033[0m");
                std::string command_mode;
                std::cin >> command_mode;
                
                if (command_mode == "9999") {
                    dual_arm_cmd->mimic_master_msg_.data = "None";
                    dual_arm_cmd->mimic_master_pub_->publish(dual_arm_cmd->mimic_master_msg_);
                    // rclcpp::spin_some(dual_arm_cmd);
                    break;
                }
                else if (command_mode == "0") {
                    dual_arm_cmd->mimic_master_msg_.data = "None";
                    dual_arm_cmd->mimic_master_pub_->publish(dual_arm_cmd->mimic_master_msg_);
                    // rclcpp::spin_some(dual_arm_cmd);

                    printf("\033[33mSelect the robot (%s: 0, %s: 1, 9999:Quit): \033[0m", RARM_name, LARM_name);
                    std::string selected_robot;
                    std::cin >> selected_robot;
                    
                    if (selected_robot == "9999") { 
                        break; 
                    }
                    /* Right Arm Selected */
                    else if (selected_robot == "0") {
                        printf("\033[31m%s is selected \033[0m\n", RARM_name);
                        Loaded_motion = dual_arm_cmd->RArm_PTP_command_input();
                        // dual_arm_cmd->rightArm_cmd.PTP_command_gen(Loaded_motion);
                        dual_arm_cmd->RArm_PTP_command_gen(Loaded_motion);
                    }
                    /* Left Arm Selected */
                    else if (selected_robot == "1") {
                        printf("\033[31m%s is selected \033[0m\n", LARM_name);
                        Loaded_motion = dual_arm_cmd->LArm_PTP_command_input();
                        // dual_arm_cmd->leftArm_cmd.PTP_command_gen(Loaded_motion);
                        dual_arm_cmd->LArm_PTP_command_gen(Loaded_motion);
                    }
                    else {
                        printf("\033[31mWrong Arm Was Selected \033[0m\n");
                    }
                }
                else if (command_mode == "1")
                {
                    dual_arm_cmd->mimic_master_msg_.data = "None";
                    dual_arm_cmd->mimic_master_pub_->publish(dual_arm_cmd->mimic_master_msg_);
                    // rclcpp::spin_some(dual_arm_cmd);

                    /* select mode */
                    printf("\033[33mSelect the master robot (%s: 0, %s: 1, 9999:Quit): \033[0m", RARM_name, LARM_name);
                    std::string master_robot;
                    std::cin >> master_robot;
                    
                    /* select mode (sychro -> mimic-guide)*/
                    if (master_robot == "0")
                    {
                        printf("\033[31m%s is selected as the master \033[0m\n", RARM_name);
                        std::string selected_master(RARM_name);
                        dual_arm_cmd->pose_syn(selected_master, 10.0);
                    }
                    else if (master_robot == "1")
                    {
                        printf("\033[31m%s is selected as the master \033[0m\n", LARM_name);
                        std::string selected_master(LARM_name);  // Fixed: was RARM_name
                        dual_arm_cmd->pose_syn(selected_master, 10.0);
                    }
                    else
                    {
                        printf("\033[31mWrong Arm Was Selected \033[0m\n");
                        break;
                    }

                    printf("\033[33mStart the mimic control? (Yes: 1, No: 0): \033[0m");
                    std::string mimic_start;
                    std::cin >> mimic_start;
                    
                    if (mimic_start == "1") {
                        std::string selected_master((master_robot == "0") ? RARM_name : LARM_name);
                        dual_arm_cmd->mimic_master_msg_.data = selected_master;
                        dual_arm_cmd->mimic_master_pub_->publish(dual_arm_cmd->mimic_master_msg_);
                        rclcpp::spin_some(dual_arm_cmd);
                    }
                    else { 
                        break; 
                    }
                }
                else {
                    printf("\033[31mWrong Mode Was Selected \033[0m\n");
                }
            }
            else
            {
                std::cout << "Waiting for current position..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }

        // Clean shutdown
        executor.cancel();
        executor_thread.join();
        
    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}