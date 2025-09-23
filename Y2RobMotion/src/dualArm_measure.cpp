#include "Y2RobMotion/robot_measure.hpp"

/* Contant - setup variables */
constexpr char MeasureFilePath[] = "/home/jay/ur_dualArm/src/Y2RobMotion/measured/";
constexpr char ROBOT_RIGHT_MODEL[] = "UR10_right";
constexpr char ROBOT_LEFT_MODEL[] = "UR10_left";


// ---- Main
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create a node and inject it
    auto node = std::make_shared<rclcpp::Node>("dualArm_measure_node");

    // Startup mode selection (stdin)
    std::cout << "Select mode: (c) continuous, (d) discrete > ";
    std::string mode_input;
    std::getline(std::cin, mode_input);
    Mode mode = Mode::Continuous;
    if (!mode_input.empty() && (std::tolower(mode_input[0]) == 'd')) {
        mode = Mode::Discrete;
    }

    try {
        // Compose UrMeasure with the node (no inheritance)
        auto measure_right = std::make_shared<UrMeasure>(mode, node, ROBOT_RIGHT_MODEL, MeasureFilePath);
        auto measure_left = std::make_shared<UrMeasure>(mode, node, ROBOT_LEFT_MODEL, MeasureFilePath);

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);  // only the node is added to the executor

        if (mode == Mode::Discrete) {
            std::cout << "[Discrete] Press ENTER to record a snapshot.\n"
                         "[Discrete] Type -1 then ENTER to exit.\n";
        } else {
            std::cout << "[Continuous] Recording continuously.\n";
        }

        executor.spin();

    } catch (const std::exception& e) {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}