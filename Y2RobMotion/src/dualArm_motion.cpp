#include "Y2RobMotion/ur10_motion.hpp"


/* Color macro */
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[37m"
#define WHITE "\033[38m"
#define RESET "\033[0m"


/**** dual_motion (dual_motion) ****/
#define ROBOT_MODEL_LEFT "UR10_left"
#define ROBOT_MODEL_RIGHT "UR10_right"

// Left arm parameters
#define UR10_Hz_LEFT 125
#define NUMBER_OF_JOINTS_LEFT 6

// Right arm parameters  
#define UR10_Hz_RIGHT 125
#define NUMBER_OF_JOINTS_RIGHT 6

/* EE to TCP HTM setting for Left arm */
const YMatrix EE2TCP_LEFT = {
    {-1.0, 0.0, 0.0, 0.0},
    { 0.0, 1.0, 0.0, 0.0},
    { 0.0, 0.0, -1.0, 0.0},
    { 0.0, 0.0, 0.0, 1.0}
};

/* EE to TCP HTM setting for Right arm */
const YMatrix EE2TCP_RIGHT = {
    {-1.0, 0.0, 0.0, 0.0},
    { 0.0, 1.0, 0.0, 0.0},
    { 0.0, 0.0, -1.0, 0.0},
    { 0.0, 0.0, 0.0, 1.0}
};

class dualArm_motion: public rclcpp::Node
{
public:
    dualArm_motion(const std::string& Node_name, 
                   const std::string& RB_name_left, const std::string& RB_name_right,
                   double Control_period_left, double Control_period_right,
                   int numOfJoint_left, int numOfJoint_right,
                   const YMatrix& HTMEE2TCP_left, const YMatrix& HTMEE2TCP_right);
    
    // Check if both arms received joint states
    bool jointsReceived() const {
        return leftUR10Motion->jointsReceived() && rightUR10Motion->jointsReceived();
    }
    
    // Check individual arm joint reception
    bool leftJointsReceived() const {
        return leftUR10Motion->jointsReceived();
    }
    
    bool rightJointsReceived() const {
        return rightUR10Motion->jointsReceived();
    }
    
    // Start both arms
    void start(bool start_flag = true) {
        leftUR10Motion->start(start_flag);
        rightUR10Motion->start(start_flag);
    }
    
    // Start individual arms
    void startLeft(bool start_flag = true) {
        leftUR10Motion->start(start_flag);
    }
    
    void startRight(bool start_flag = true) {
        rightUR10Motion->start(start_flag);
    }

private:
    std::unique_ptr<ur10_motion> leftUR10Motion;
    std::unique_ptr<ur10_motion> rightUR10Motion;
    rclcpp::TimerBase::SharedPtr monitor_timer_, mimicCon_timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mimicMaster_sub;

    std::string Mimic_master;

    void dualArm_monitoring();
    void mimicControl();
    void mimicMasterCB(const std_msgs::msg::String::SharedPtr msg);

};

dualArm_motion::dualArm_motion(const std::string& Node_name, 
                               const std::string& RB_name_left, const std::string& RB_name_right,
                               double Control_period_left, double Control_period_right,
                               int numOfJoint_left, int numOfJoint_right,
                               const YMatrix& HTMEE2TCP_left, const YMatrix& HTMEE2TCP_right)
: Node(Node_name),Mimic_master("None")
{
    // Initialize left arm
    leftUR10Motion = std::make_unique<ur10_motion>(this, RB_name_left, Control_period_left, 
                                                   numOfJoint_left, HTMEE2TCP_left);
    
    // Initialize right arm
    rightUR10Motion = std::make_unique<ur10_motion>(this, RB_name_right, Control_period_right, 
                                                    numOfJoint_right, HTMEE2TCP_right);
    
    RCLCPP_INFO(this->get_logger(), "Dual arm motion node initialized for: %s and %s", 
                RB_name_left.c_str(), RB_name_right.c_str());

    monitor_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&dualArm_motion::dualArm_monitoring,this));

    mimicCon_timer_= this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(Control_period_right*1000)),
        std::bind(&dualArm_motion::mimicControl,this));

    mimicMaster_sub = this->create_subscription<std_msgs::msg::String>("Mimic_Master", 10, 
        std::bind(&dualArm_motion::mimicMasterCB, this, std::placeholders::_1));
}

void dualArm_motion::mimicMasterCB(const std_msgs::msg::String::SharedPtr msg){
    if((msg->data == rightUR10Motion->robot_name)||(msg->data == leftUR10Motion->robot_name))
    {
        Mimic_master = msg->data;

        rightUR10Motion->control_mode = (Mimic_master == rightUR10Motion->robot_name)? "Guiding" : "Position";
        leftUR10Motion->control_mode = (Mimic_master == leftUR10Motion->robot_name)? "Guiding" : "Position";
    }
    else{ 
        Mimic_master = "None";
        rightUR10Motion->control_mode = "Idling";
        leftUR10Motion->control_mode = "Idling";
    }
}

void dualArm_motion::mimicControl()
{
    /* If master robot is right arm */
    if(Mimic_master == rightUR10Motion->robot_name){
        leftUR10Motion->target_pose = rightUR10Motion->target_pose;
    }
    /* If master robot is left arm */
    else if(Mimic_master == leftUR10Motion->robot_name){
        rightUR10Motion->target_pose = leftUR10Motion->target_pose;
    }
}

void dualArm_motion::dualArm_monitoring()
    {
        printf(RED "[%s-%s] mode: %s, [%s-%s] mode: %s" RESET "\n"
            ,rightUR10Motion->robot_name.c_str(),(Mimic_master==rightUR10Motion->robot_name)?"Master":" ",rightUR10Motion->control_mode.c_str()
            ,leftUR10Motion->robot_name.c_str(),(Mimic_master==leftUR10Motion->robot_name)?"Master":" ",leftUR10Motion->control_mode.c_str());
        printf("(Ctl_modes: Idling, Position, Guiding, Force)\n");
        printf("------------------------------------------------------------------------------\n");

        /* R - Current Pose */
        printf(GREEN "<R-Current> x:%.1f, y:%.1f, z%.1f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        rightUR10Motion->current_pose[0],rightUR10Motion->current_pose[1],rightUR10Motion->current_pose[2],
        rightUR10Motion->current_pose[3],rightUR10Motion->current_pose[4],rightUR10Motion->current_pose[5]);

        /* R - Target Pose */
        printf(BLUE "<R-Target>  x:%.1f, y:%.1f, z%.1f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        rightUR10Motion->target_pose[0],rightUR10Motion->target_pose[1],rightUR10Motion->target_pose[2],
        rightUR10Motion->target_pose[3],rightUR10Motion->target_pose[4],rightUR10Motion->target_pose[5]);
        printf("------------------------------------------------------------------------------\n");

        /* L - Current Pose */
        printf(GREEN "<L-Current> x:%.1f, y:%.1f, z%.1f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        leftUR10Motion->current_pose[0],leftUR10Motion->current_pose[1],leftUR10Motion->current_pose[2],
        leftUR10Motion->current_pose[3],leftUR10Motion->current_pose[4],leftUR10Motion->current_pose[5]);

        /* L - Target Pose */
        printf(BLUE "<L-Target>  x:%.1f, y:%.1f, z%.1f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        leftUR10Motion->target_pose[0],leftUR10Motion->target_pose[1],leftUR10Motion->target_pose[2],
        leftUR10Motion->target_pose[3],leftUR10Motion->target_pose[4],leftUR10Motion->target_pose[5]);
        printf("------------------------------------------------------------------------------\n");

        /* R - Target angles */
        printf(BLUE "<R-Target>  j1:%.3f, j2:%.3f, j3:%.3f, j4:%.3f, j5:%.3f, j6:%.3f" RESET "\n",
        rightUR10Motion->target_angles[0],rightUR10Motion->target_angles[1],rightUR10Motion->target_angles[2],
        rightUR10Motion->target_angles[3],rightUR10Motion->target_angles[4],rightUR10Motion->target_angles[5]);

        /* L - Target angles */
        printf(BLUE "<L-Target>  j1:%.3f, j2:%.3f, j3:%.3f, j4:%.3f, j5:%.3f, j6:%.3f" RESET "\n",
        leftUR10Motion->target_angles[0],leftUR10Motion->target_angles[1],leftUR10Motion->target_angles[2],
        leftUR10Motion->target_angles[3],leftUR10Motion->target_angles[4],leftUR10Motion->target_angles[5]);
        printf("------------------------------------------------------------------------------\n");

        /* Measured force/torque */
        printf(GREEN "<R-FT> Fx:%+.3f, Fy:%+.3f, Fz%+.3f, Mx:%+.3f, My:%+.3f, Mz:%+.3f" RESET "\n",
        rightUR10Motion->ft1data[0],rightUR10Motion->ft1data[1],rightUR10Motion->ft1data[2],
        rightUR10Motion->ft1data[3],rightUR10Motion->ft1data[4],rightUR10Motion->ft1data[5]);

        printf(GREEN "<L-FT> Fx:%+.3f, Fy:%+.3f, Fz%+.3f, Mx:%+.3f, My:%+.3f, Mz:%+.3f" RESET "\n",
        leftUR10Motion->ft1data[0],leftUR10Motion->ft1data[1],leftUR10Motion->ft1data[2],
        leftUR10Motion->ft1data[3],leftUR10Motion->ft1data[4],leftUR10Motion->ft1data[5]);
        printf("------------------------------------------------------------------------------\n");

        /* Admittance control Mass */
        printf(BLUE "<AC-Mass> x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        rightUR10Motion->AControl[0].adm_MDK_monitor(0),rightUR10Motion->AControl[1].adm_MDK_monitor(0),rightUR10Motion->AControl[2].adm_MDK_monitor(0),
        rightUR10Motion->AControl[3].adm_MDK_monitor(0),rightUR10Motion->AControl[4].adm_MDK_monitor(0),rightUR10Motion->AControl[5].adm_MDK_monitor(0));

        printf(MAGENTA "<AC-Damper> x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        rightUR10Motion->AControl[0].adm_MDK_monitor(1),rightUR10Motion->AControl[1].adm_MDK_monitor(1),rightUR10Motion->AControl[2].adm_MDK_monitor(1),
        rightUR10Motion->AControl[3].adm_MDK_monitor(1),rightUR10Motion->AControl[4].adm_MDK_monitor(1),rightUR10Motion->AControl[5].adm_MDK_monitor(1));

        printf(GREEN "<AC-Spring> x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
        rightUR10Motion->AControl[0].adm_MDK_monitor(2),rightUR10Motion->AControl[1].adm_MDK_monitor(2),rightUR10Motion->AControl[2].adm_MDK_monitor(2),
        rightUR10Motion->AControl[3].adm_MDK_monitor(2),rightUR10Motion->AControl[4].adm_MDK_monitor(2),rightUR10Motion->AControl[5].adm_MDK_monitor(2));

        
        printf("==============================================================================\n");

    }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    /* ROS node generation as dual arm instance */
    const std::string node_name = "urDualArm";
    auto node = std::make_shared<dualArm_motion>(
        node_name, 
        ROBOT_MODEL_LEFT, ROBOT_MODEL_RIGHT, 
        1/static_cast<double>(UR10_Hz_LEFT), 1/static_cast<double>(UR10_Hz_RIGHT),
        NUMBER_OF_JOINTS_LEFT, NUMBER_OF_JOINTS_RIGHT,
        EE2TCP_LEFT, EE2TCP_RIGHT);
    
    RCLCPP_INFO(node->get_logger(), "Waiting for joint states from both arms...");
    
    /* Node executor generation */
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    /* Wait for joint states from both arms */
    while (rclcpp::ok()) {
        bool left_ready = node->leftJointsReceived();
        bool right_ready = node->rightJointsReceived();
        
        if (left_ready && right_ready) {
            RCLCPP_INFO(node->get_logger(), "Joint states received from both arms!");
            break;
        }
        else if (!left_ready && !right_ready) {
            RCLCPP_INFO(node->get_logger(), "Waiting for joint states from both arms...");
        }
        else if (!left_ready) {
            RCLCPP_INFO(node->get_logger(), "Waiting for joint states from LEFT arm...");
        }
        else if (!right_ready) {
            RCLCPP_INFO(node->get_logger(), "Waiting for joint states from RIGHT arm...");
        }
        
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    /* Inquiry of starting */
    int start_option = 0;
    std::cout << "\033[33m [DUAL ARM] start option:\033[0m" << std::endl;
    std::cout << "1: Start both arms" << std::endl;
    std::cout << "2: Start left arm only" << std::endl;
    std::cout << "3: Start right arm only" << std::endl;
    std::cout << "0: Quit" << std::endl;
    std::cout << "\033[33mEnter option: \033[0m";
    std::cin >> start_option;
    
    switch(start_option) {
        case 1:
            RCLCPP_INFO(node->get_logger(), "Starting both arms...");
            node->start(false);
            break;
        case 2:
            RCLCPP_INFO(node->get_logger(), "Starting left arm only...");
            node->startLeft(true);
            break;
        case 3:
            RCLCPP_INFO(node->get_logger(), "Starting right arm only...");
            node->startRight(true);
            break;
        case 0:
        default:
            RCLCPP_INFO(node->get_logger(), "Exiting...");
            rclcpp::shutdown();
            return 0;
    }
    
    /* Node execution */
    executor.spin();
    rclcpp::shutdown();
    return 0;
}