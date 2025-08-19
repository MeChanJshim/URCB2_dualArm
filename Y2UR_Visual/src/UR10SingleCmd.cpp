#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <csignal>
#include <cstdlib>
#include <string>

/* Header for command from Actaul */
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

/* Header for command to RVIZ */
#include "sensor_msgs/msg/joint_state.hpp"
#define M_PI 3.14159265358979323846
#define DegreeToRadian(degree) ((degree) * M_PI / 180.0)
#define RadianToDegree(radian) ((radian) * 180.0 / M_PI)

#define ROBOT_NAME "UR10_right"

void signal_handler(int signum){exit(signum);}

class UR10SingleCmd: public rclcpp::Node
{
    public:
        UR10SingleCmd(const std::string& robot_name_);
        
    private:
        std::string robot_name;
        std::vector<double> cmd_out;
        sensor_msgs::msg::JointState SenMsg_msg,ConMsg_msg;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr SenMsg_pub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ConMsg_pub;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Jangle_sub;

        rclcpp::TimerBase::SharedPtr timer_;

        void run();
        void UR10_Jangle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        
        
};
UR10SingleCmd::UR10SingleCmd(const std::string& robot_name_)
:Node("UR10SingleCmd"), cmd_out(6,0.0), robot_name(robot_name_)
{
    cmd_out = {DegreeToRadian(53.30),DegreeToRadian(-126.81),DegreeToRadian(116.34),
    DegreeToRadian(-79.43),DegreeToRadian(-93.48),DegreeToRadian(-26.17)};

    SenMsg_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",1);
    
    std::string ConMsg_TP = robot_name + "_joint_states";
    ConMsg_pub = this->create_publisher<sensor_msgs::msg::JointState>(ConMsg_TP,1);

    std::string targetJ_TP = robot_name + "_targetJ";
    Jangle_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(targetJ_TP,1,
    std::bind(&UR10SingleCmd::UR10_Jangle_callback,this,std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&UR10SingleCmd::run,this));
}

void UR10SingleCmd::UR10_Jangle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    for(int i=0;i<6;i++) {cmd_out[i] = msg->data[i];}

    /* Message Transmition */

}

void UR10SingleCmd::run()
{
    // while(rclcpp::ok())
    // {

        // 필요한 필드 설정
        SenMsg_msg.header.stamp = this->now();
        ConMsg_msg.header.stamp = this->now();

        SenMsg_msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        ConMsg_msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        SenMsg_msg.position = {cmd_out[0],cmd_out[1],cmd_out[2],cmd_out[3],cmd_out[4],cmd_out[5]};
        ConMsg_msg.position = {cmd_out[0],cmd_out[1],cmd_out[2],cmd_out[3],cmd_out[4],cmd_out[5]};

        /* Message publication */
        SenMsg_pub->publish(SenMsg_msg);
        ConMsg_pub->publish(ConMsg_msg);

        SenMsg_msg.name.clear();
        ConMsg_msg.name.clear();
        SenMsg_msg.position.clear();
        ConMsg_msg.position.clear();
    // }
}


int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<UR10SingleCmd>(ROBOT_NAME);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;   
}
