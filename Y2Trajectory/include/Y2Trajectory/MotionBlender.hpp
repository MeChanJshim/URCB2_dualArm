#include "Y2Matrix/YMatrix.hpp"
#include "Y2Trajectory/PositionInterpolation.hpp"
#include "Y2Trajectory/QuaternionInterpolator.hpp"
#include "Y2Trajectory/AccProfiler.hpp"

#include <fstream>

class MotionBlender {
    public:
        // Constructor
        MotionBlender(const YMatrix& position_, const std::vector<double>& velocity_, 
                        double angVelLimit_,
                        double startingTime_, double lastRestingTime_, 
                        double accelerationTime_, double samplingTime_)
            : position(position_), velocity(velocity_), angVelLimit(angVelLimit_),
              startingTime(startingTime_), lastRestingTime(lastRestingTime_), 
              accelerationTime(accelerationTime_), samplingTime(samplingTime_) {}

        virtual YMatrix blendMotion() = 0;

    protected:
        YMatrix position;  // Position data
        std::vector<double> velocity;  // Velocity data
        double angVelLimit;  // Angular velocity limits
        double startingTime;  // Starting time of the motion
        double lastRestingTime;  // Time when the motion comes to rest
        double accelerationTime;  // Time taken to accelerate
        double samplingTime;  // Sampling time interval
};