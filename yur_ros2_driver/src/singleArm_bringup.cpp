#include "yur_ros2_driver/yur_ros_wrapper.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string ur_ip = "192.168.1.120";
    std::string robot_name = "UR10_right";
    int reverse_port = 50002; // 50001

    auto node = std::make_shared<YRos2Wrapper>(ur_ip, reverse_port, robot_name);

    // ROS2에서는 MultiThreadedExecutor 사용
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    executor.spin();

    rclcpp::shutdown();

    return 0;
}