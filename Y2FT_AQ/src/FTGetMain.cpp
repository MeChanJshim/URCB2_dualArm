#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Y2FT_AQ/FT_rawGet.hpp"
#include "Y2Filters/Gen_filter.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


/* Sensor info */
#define FT1_IP "192.168.1.100"
#define FT1_PORT 8890

#define SENSOT_INIT_COUNT_NUM 50

/* Filter parameter info */
#define MOV_SIZE 10

/* Robot info */
#define ROBOT_NAME "UR10_right"
YMatrix ROT_TCP2FT = // 이거 만드는 Guidance 제시해야됨
{
    {1,  0,  0},
    {0, -1,  0},
    {0,  0, -1}
};

class FTGetMain: public rclcpp::Node
{
    public:
        FTGetMain(std::string ft_ip, int ft_port, std::string robot_name, int mov_size = 1);

        /* Main processing function */
        void transferData(YMatrix& ROT_TCP2FT, int init_count_num = 10);

    private:

        /* Parameters */
        std::vector<Y_MovFilter> movFilter; // MOV filter for each axis
        std::vector<double> current_TCP_pose; // Current TCP pose of the robot (x,y,z,wx,wy,wz)
        YMatrix ROT_Base2TCP; // Base to TCP rotation matrix

        /* FT Raw get instance*/
        FT_rawGet FT1SensorGet;

        /* Publishers */
        geometry_msgs::msg::WrenchStamped ft1_msg;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ft1_pub;

        /* Subscriber */
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr currentP_sub;
    
        /* Filtering function */
        FTData filtering(const FTData& ftdata);

        /* Callback functions */
        void currentPCB(const std_msgs::msg::Float64MultiArray::ConstPtr& msg);

};

FTGetMain::FTGetMain(std::string ft_ip, int ft_port, std::string robot_name, int mov_size) : 
Node("FTGetMain"), FT1SensorGet(ft_ip, ft_port), current_TCP_pose(6,0.0), ROT_Base2TCP(3,3)
{
    std::string ftdata_TP = robot_name + "_ftdata";
    ft1_pub = this->create_publisher<geometry_msgs::msg::WrenchStamped>(ftdata_TP, 1);

    std::string currentP_TP = robot_name + "_currentP";
    currentP_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(currentP_TP,1,
    std::bind(&FTGetMain::currentPCB, this, std::placeholders::_1));

    ft1_msg.header.frame_id = "ft_frame";

    /* MOV initialization */
    for(int i = 0; i < 6; ++i) {
        movFilter.emplace_back(mov_size);
    }

    ROT_Base2TCP = YMatrix::identity(3); // Initialize the rotation matrix to identity
}

/* Callbacks */
void FTGetMain::currentPCB(const std_msgs::msg::Float64MultiArray::ConstPtr& msg)
{
    for(size_t i = 0; i < 6 && i < msg->data.size(); ++i) {
        current_TCP_pose[i] = msg->data[i];
    }

    /* Tranfrom from spatial angle to rotation matrix */
    SpatialAngle spatial_angle(current_TCP_pose[3], current_TCP_pose[4], current_TCP_pose[5]);
    ROT_Base2TCP.insert(0,0,YMatrix::fromSpatialAngle(spatial_angle));
}

/* Filtering */
FTData FTGetMain::filtering(const FTData& ftdata)
{
    FTData filtered_data;

    /* MOV filtering */
    filtered_data.Fx = movFilter[0].MovFilter(ftdata.Fx);
    filtered_data.Fy = movFilter[1].MovFilter(ftdata.Fy);
    filtered_data.Fz = movFilter[2].MovFilter(ftdata.Fz);
    filtered_data.Mx = movFilter[3].MovFilter(ftdata.Mx);
    filtered_data.My = movFilter[4].MovFilter(ftdata.My);
    filtered_data.Mz = movFilter[5].MovFilter(ftdata.Mz);

    return filtered_data;  // Return the filtered data
}

/* Main processing */
void FTGetMain::transferData(YMatrix& ROT_TCP2FT, int init_count_num)
{
    /* Initialize the local parameters */
    FTData filtered_out;
    YMatrix SframeForce(3, 1), RframeForce(3, 1), GcompForce(3,1);
    YMatrix SframeMoment(3, 1), RframeMoment(3, 1), GcompMoment(3,1);

    while (rclcpp::ok()) {

        /* Initialize the sesor */
        FT1SensorGet.FT_init(init_count_num);

        /* Filtering the raw data */
        filtered_out = filtering(FT1SensorGet.FTGet());

        /* Real-time transformming the ftdata (sensor frame -> robot base) */
        SframeForce = {{filtered_out.Fx},{filtered_out.Fy},{filtered_out.Fz}};
        SframeMoment = {{filtered_out.Mx},{filtered_out.My},{filtered_out.Mz}};

        RframeForce = ROT_Base2TCP*ROT_TCP2FT*SframeForce;
        RframeMoment = ROT_Base2TCP*ROT_TCP2FT*SframeMoment;

        /* Gravity compensation - 채워야함 */

        GcompForce = RframeForce;
        GcompMoment = RframeMoment;

        /* Transfering the data using message */
        ft1_msg.header.stamp = this->now();
        ft1_msg.wrench.force.x = GcompForce[0][0];
        ft1_msg.wrench.force.y = GcompForce[1][0];
        ft1_msg.wrench.force.z = GcompForce[2][0];
        ft1_msg.wrench.torque.x = GcompMoment[0][0];
        ft1_msg.wrench.torque.y = GcompMoment[1][0];
        ft1_msg.wrench.torque.z = GcompMoment[2][0];

        ft1_pub->publish(ft1_msg);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    /* FTGetMain instance generation */
    auto node = std::make_shared<FTGetMain>(FT1_IP, FT1_PORT,ROBOT_NAME,MOV_SIZE);

    /* Transfering the message */
    node->transferData(ROT_TCP2FT,SENSOT_INIT_COUNT_NUM);

    rclcpp::spin(node);

}