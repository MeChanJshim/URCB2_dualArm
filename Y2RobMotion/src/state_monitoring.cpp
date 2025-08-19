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


/* State Monitoring */
void ur10_motion::state_monitoring()
{
    
    printf(RED "[%s] - Control mode: %s" RESET "\n",robot_name.c_str(),control_mode.c_str());
    printf("(Ctl_modes: Idling, Position, Guiding, Force)\n");
    printf("------------------------------------------------------------------------------\n");

    /* Current angles */
    printf(GREEN "<Current> j1:%.3f, j2:%.3f, j3:%.3f, j4:%.3f, j5:%.3f, j6:%.3f" RESET "\n",
    current_angles[0],current_angles[1],current_angles[2],current_angles[3],current_angles[4],current_angles[5]);

    /* Target angles */
    printf(BLUE "<Target>  j1:%.3f, j2:%.3f, j3:%.3f, j4:%.3f, j5:%.3f, j6:%.3f" RESET "\n",
    target_angles[0],target_angles[1],target_angles[2],target_angles[3],target_angles[4],target_angles[5]);

    printf("------------------------------------------------------------------------------\n");

    /* Current angular velocity */
    printf(GREEN "<Cur Ang. Vel.> j1:%+.3f, j2:%+.3f, j3:%+.3f, j4:%+.3f, j5:%+.3f, j6:%+.3f" RESET "\n",
    current_angvel[0],current_angvel[1],current_angvel[2],current_angvel[3],current_angvel[4],current_angvel[5]);

    /* Target angular velocity */
    printf(BLUE "<Tar Ang. Vel.>  j1:%+.3f, j2:%+.3f, j3:%+.3f, j4:%+.3f, j5:%+.3f, j6:%+.3f" RESET "\n",
    target_angvel[0],target_angvel[1],target_angvel[2],target_angvel[3],target_angvel[4],target_angvel[5]);

    printf("------------------------------------------------------------------------------\n");

    /* Current Pose */
    printf(GREEN "<Current> x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
    current_pose[0],current_pose[1],current_pose[2],current_pose[3],current_pose[4],current_pose[5]);

    /* Target Pose */
    printf(BLUE "<Target>  x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
    target_pose[0],target_pose[1],target_pose[2],target_pose[3],target_pose[4],target_pose[5]);

    printf("------------------------------------------------------------------------------\n");

    /* Current cartesian velocity */
    printf(GREEN "<Cur Car. Vel.> x:%+.3f, y:%+.3f, z%+.3f, wx:%+.3f, wy:%+.3f, wz:%+.3f" RESET "\n",
    current_carvel[0],current_carvel[1],current_carvel[2],current_carvel[3],current_carvel[4],current_carvel[5]);

    /* Target cartesian velocity */
    printf(BLUE "<Tar Car. Vel.>  x:%+.3f, y:%+.3f, z%+.3f, wx:%+.3f, wy:%+.3f, wz:%+.3f" RESET "\n",
    target_carvel[0],target_carvel[1],target_carvel[2],target_carvel[3],target_carvel[4],target_carvel[5]);

    printf("------------------------------------------------------------------------------\n");
    
    /* Measured force/torque */
    printf(GREEN "<Force/Torque> Fx:%+.3f, Fy:%+.3f, Fz%+.3f, Mx:%+.3f, My:%+.3f, Mz:%+.3f" RESET "\n",
        ft1data[0],ft1data[1],ft1data[2],ft1data[3],ft1data[4],ft1data[5]);

    /* Admittance control output pose */ 
    printf(BLUE "<AC OUT>  x:%.3f, y:%.3f, z%.3f, wx:%.3f, wy:%.3f, wz:%.3f" RESET "\n",
    AC_pose[0],AC_pose[1],AC_pose[2],AC_pose[3],AC_pose[4],AC_pose[5]);


    printf("==============================================================================\n");
}
