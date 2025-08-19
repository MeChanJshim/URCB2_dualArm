#include "Y2RobMotion/ur10_motion.hpp"

void ur10_motion::control_force()
{
    control_mode = "Force";
    #if (SIMULATION == false)
    // sendServoJ(target_angles);
    #endif
    pre_control_mode = control_mode; // Store previous control mode for comparison
}