#include "Y2RobMotion/ur10_motion.hpp"

/**** ur_motion (ur_motion) ****/
#define ROBOT_MODEL "UR10_right"

#define UR10_Hz 125
#define NUMBER_OF_JOINTS 6

/* EE to TCP HTM setting */
const YMatrix EE2TCP = {
    {-1.0,  0.0,  0.0,  0.0},
    { 0.0,  1.0,  0.0,  0.0},
    { 0.0,  0.0, -1.0,  0.0},
    { 0.0,  0.0,  0.0,  1.0}
};

class singleArm_motion: public rclcpp::Node
{
    public:
        singleArm_motion(const std::string& Node_name, const std::string& RB_name, 
            double Control_period, int numOfJoint, const YMatrix& HTMEE2TCP);

        bool jointsReceived() const {
            return UR10Motion->jointsReceived();
        }
    
        void start(bool start_flag = true) {
            UR10Motion->start(start_flag);
        }

    private:
    
        std::unique_ptr<ur10_motion> UR10Motion;
        
};

singleArm_motion::singleArm_motion(const std::string& Node_name, const std::string& RB_name, 
            double Control_period, int numOfJoint, const YMatrix& HTMEE2TCP)
: Node(Node_name)
{
    UR10Motion = std::make_unique<ur10_motion>(this,RB_name,Control_period,numOfJoint,HTMEE2TCP);
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    /* ROS node generation as hip instance */
    const std::string node_name = "urSingleArm";
    auto node = std::make_shared<singleArm_motion>(
        node_name, ROBOT_MODEL, 1/static_cast<double>(UR10_Hz), NUMBER_OF_JOINTS, EE2TCP);

    RCLCPP_INFO(node->get_logger(), "Waiting for joint states...");
    
    /* Node generation */
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    /* Bringup execution check */
    while (rclcpp::ok() && !node->jointsReceived()) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(node->get_logger(), "Joint states received!");
    
    /* Inqury of starting */
    bool start_boolean = false;
    std::cout << "\033[33m [" << ROBOT_MODEL 
              << "] start? (1: start, 0: quit) \033[0m";
    std::cin >> start_boolean;
    
    if (!start_boolean) {
        rclcpp::shutdown();
        return 0;
    }
    
    /* Node execution */
    node->start(true);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}