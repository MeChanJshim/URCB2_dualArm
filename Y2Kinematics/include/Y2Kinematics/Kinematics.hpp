#pragma once

#include "Y2Matrix/YMatrix.hpp"
#include "Y2Kinematics/QP_solver.hpp"
#include <iostream>
#include <vector>

class Kinematics {
    public:
        Kinematics(double SamplingTime_ = 0.01, size_t numOfAxis_ = 7, const YMatrix& EE2TCP_ = YMatrix::identity(4));

        // Forward Kinematics (Based on End-Effector)
        virtual YMatrix forwardKinematics(const std::vector<double>& q) = 0;

        // Jacobian Calculation (Based on End-Effector)
        virtual YMatrix calculateJacobian(const std::vector<double>& q) = 0;

        // Enhanced QP-based IK solver(Based on End-Effector)
        std::vector<double> solve_IK(const std::vector<double>& q_current, 
                                            const YMatrix& target_HTM);

        // Utility methods
        static double roundToNthDecimal(double value, int n);

        void setControlGains(double kp_pos, double kp_rot);
        
        void setQPWeights(double omega_p_val, double alpha_val, double lambda_val);

        void setJointLimits(const std::vector<double>& q_min, 
                          const std::vector<double>& q_max,
                          const std::vector<double>& qd_min, 
                          const std::vector<double>& qd_max);

        // Print pose in a readable format
        void printPose(const YMatrix& pose, const std::string& label);

    protected:
        double dt; // Sampling time
        size_t numOfAxis;  // Number of joints
        YMatrix EE2TCP; // End-Effector to TCP transformation matrix
        int QP_precision = 4; // Precision for rounding
        double QP_tolerance = pow(10,-QP_precision); // QP solver tolerance

        // Control gains for QP-based IK
        double Kp_pos = 1.0;
        double Kp_rot = 1.0;
        
        // QP parameters
        double omega_p = 1.0;
        double alpha = 0.1;
        double lambda = 0.01;
        
        // Joint limits
        std::vector<double> q_min, q_max;
        std::vector<double> qd_min, qd_max;
    
    private:
        QPSolver qp_solver;

};
