#pragma once
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include "Y2Matrix/YMatrix.hpp"
#include "Y2Trajectory/MotionBlender6D.hpp"
#include "Y2RobMotion/robot_command.hpp"

// #define M_PI 3.141592
// #define DegreeToRadian(degree) ((degree) * M_PI / 180.0)
// #define RadianToDegree(radian) ((radian) * 180.0 / M_PI)

// Constants
constexpr char ROBOT_MODEL[] = "UR10_right";
constexpr int UR10_HZ = 125;
constexpr double CONTROL_PERIOD = 1.0 / UR10_HZ;

// Directories
constexpr char NISTDUALUR_PATH[] = "/home/dual_ur/ros2_ws/src/nist_dualUR";
constexpr char LOAD_FILE[] = "txtcmd/cmd_6D.txt";

class singleArm_cmd: public rclcpp::Node
{
    public:
        singleArm_cmd(const std::string& node_name, const std::string& robot_name_, double control_per, PathGenParam& pg_param)
        :Node(node_name)
        {
            rb_cmd = std::make_unique<robot_command>(this,robot_name_,control_per,pg_param);
        }

        bool is_roboInit(){return rb_cmd->robot_init;}

        YMatrix PTP_command_input(){return rb_cmd->PTP_command_input();}

        void sendCommand(const std::string& command_mode, const YMatrix& loaded_motion)
        {return rb_cmd->sendCommand(command_mode, loaded_motion);}

    private:
        std::unique_ptr<robot_command> rb_cmd;
};

int main(int argc, char** argv)
{
    /* Path parameter set */
    PathGenParam pg_param;
    pg_param.initialTransferSpeed = 5.0;      // mm/s
    pg_param.angularVelocityLimit = 5.0;      // degrees/s
    pg_param.accelerationTime = 1.0;          // seconds
    pg_param.startingTime = 2.0;              // seconds
    pg_param.lastRestingTime = 2.0;           // seconds
    pg_param.ptp_target_velocity = 0.5;       // degrees/s
    pg_param.loadFileType = "cmd_6D";    // cmd_6D, cmd_9D, cmd_continue6D, cmd_continue9D
    
    /* Node init */
    rclcpp::init(argc, argv);

    /* Node hip instance generation */
    std::string Node_name = "singleArm_cmd";
    auto cmd_node = std::make_shared<singleArm_cmd>(Node_name,ROBOT_MODEL,CONTROL_PERIOD,pg_param);

    // Create executor for spinning
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(cmd_node);
    
    // Start executor in a separate thread
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    
    YMatrix loaded_motion(1, 6);

    // Main command loop
    while (rclcpp::ok()) {
        if (cmd_node->is_roboInit()) {
            std::cout << "Enter command mode (Idling, PTP, TxtLoad, Guiding, 9999:Quit): ";
            std::string command_mode;
            std::cin >> command_mode;
            
            if (command_mode == "9999") { 
                break; 
            }
            
            if (command_mode == "PTP") {
                loaded_motion = cmd_node->PTP_command_input();
            }
            else if (command_mode == "TxtLoad") {
                std::string nist_dual_ur_path(NISTDUALUR_PATH);
                std::string load_file_path(LOAD_FILE);
                
                try {
                    auto loaded_data = YMatrix::loadFromFile(nist_dual_ur_path + "/" + load_file_path);
                    loaded_motion.resize(loaded_data.rows(), loaded_data.cols());
                    loaded_motion = loaded_data;
                    
                    if (loaded_motion.rows() < 2) {
                        RCLCPP_ERROR(cmd_node->get_logger(), "Error: Not enough positions to blend motion.");
                        continue;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(cmd_node->get_logger(), "Failed to load file: %s", e.what());
                    continue;
                }
            }

            cmd_node->sendCommand(command_mode, loaded_motion);
        }
        else {
            std::cout << "Waiting for current position..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    // Clean shutdown
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    
    return 0;

}