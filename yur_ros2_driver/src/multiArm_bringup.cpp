#include "yur_ros2_driver/yur_ros_wrapper.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::string RightArm_ip = "192.168.1.120";
    std::string RightArm_name = "UR10_right";
    std::string LeftArm_ip = "192.168.1.121";
    std::string LeftArm_name = "UR10_left";
    int RightArm_reverse_port = 50001;
    int LeftArm_reverse_port = 50002;

    // 두 개의 노드 생성
    auto RightArm = std::make_shared<YRos2Wrapper>(RightArm_ip, RightArm_reverse_port, RightArm_name);
    auto LeftArm = std::make_shared<YRos2Wrapper>(LeftArm_ip, LeftArm_reverse_port, LeftArm_name);

    // MultiThreadedExecutor로 두 노드를 동시에 실행
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(RightArm);
    executor.add_node(LeftArm);

    // 모든 노드를 병렬로 실행
    executor.spin();

    // 정리
    rclcpp::shutdown();

    return 0;
}