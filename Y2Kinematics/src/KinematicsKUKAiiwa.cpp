#include "Y2Kinematics/KinematicsKUKAiiwa.hpp"


// Complete Forward Kinematics (as shown in previous artifact)
YMatrix KinematicsKUKAiiwa::forwardKinematics(const std::vector<double>& q) {
    if (q.size() != numOfAxis) {
        throw std::invalid_argument("Joint angles must have 7 elements");
    }
    
    double th1 = q[0], th2 = q[1], th3 = q[2], th4 = q[3];
    double th5 = q[4], th6 = q[5], th7 = q[6];
    
    // Pre-calculate trigonometric functions
    double s1 = sin(th1), c1 = cos(th1);
    double s2 = sin(th2), c2 = cos(th2);
    double s3 = sin(th3), c3 = cos(th3);
    double s4 = sin(th4), c4 = cos(th4);
    double s5 = sin(th5), c5 = cos(th5);
    double s6 = sin(th6), c6 = cos(th6);
    double s7 = sin(th7), c7 = cos(th7);
    
    YMatrix T(4, 4);
    
    // Implementing the full transformation matrix from MATLAB
    // Row 1
    T[0][0] = s7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - 
                c7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + 
                    c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)));
    
    T[0][1] = c7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) + 
                s7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + 
                    c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)));
    
    T[0][2] = c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - 
                s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
    
    T[0][3] = d7*T[0][2] + d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + d3*c1*s2;
    
    // Row 2  
    T[1][0] = c7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + 
                    c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - 
                s7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3));
    
    T[1][1] = -c7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3)) - 
                s7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + 
                    c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)));
    
    T[1][2] = s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - 
                c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
    
    T[1][3] = d3*s1*s2 - d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - d7*T[1][2];
    
    // Row 3
    T[2][0] = c7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4)) - 
                s7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3);
    
    T[2][1] = -c7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) - 
                s7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4));
    
    T[2][2] = s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4);
    
    T[2][3] = d1 + d5*(c2*c4 + c3*s2*s4) + d7*T[2][2] + d3*c2;
    
    // Row 4
    T[3][0] = 0; T[3][1] = 0; T[3][2] = 0; T[3][3] = 1;
    
    return T*EE2TCP;
}
    
// Complete Jacobian calculation 
YMatrix KinematicsKUKAiiwa::calculateJacobian(const std::vector<double>& q) {
    if (q.size() != numOfAxis) {
        throw std::invalid_argument("Joint angles must have 7 elements");
    }
    
    double th1 = q[0], th2 = q[1], th3 = q[2], th4 = q[3];
    double th5 = q[4], th6 = q[5], th7 = q[6];
    
    // Pre-calculate trigonometric functions
    double s1 = sin(th1), c1 = cos(th1);
    double s2 = sin(th2), c2 = cos(th2);
    double s3 = sin(th3), c3 = cos(th3);
    double s4 = sin(th4), c4 = cos(th4);
    double s5 = sin(th5), c5 = cos(th5);
    double s6 = sin(th6), c6 = cos(th6);
    
    YMatrix J(6, numOfAxis);
    
    // First row (Jv_x - x-direction linear velocity)
    J[0][0] = d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - d3*s1*s2;
    
    J[0][1] = c1*(d5*(c2*c4 + c3*s2*s4) + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d3*c2);
    
    J[0][2] = d5*c3*s1*s4 + d5*c1*c2*s3*s4 + d7*c3*c6*s1*s4 + d7*s1*s3*s5*s6 + d7*c1*c2*c6*s3*s4 - d7*c1*c2*c3*s5*s6 - d7*c3*c4*c5*s1*s6 - d7*c1*c2*c4*c5*s3*s6;
    
    J[0][3] = -(c1*c3 - c2*s1*s3)*(d5*(c2*c4 + c3*s2*s4) + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))) - s2*s3*(d7*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + d5*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2));
    
    J[0][4] = d7*(c2*c4 + c3*s2*s4)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d7*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4));
    
    J[0][5] = -d7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) - d7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4));
    
    J[0][6] = 0;
    
    // Second row (Jv_y - y-direction linear velocity)
    J[1][0] = d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + d3*c1*s2;
    
    J[1][1] = s1*(d5*(c2*c4 + c3*s2*s4) + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) + d3*c2);
    
    J[1][2] = d5*c2*s1*s3*s4 - d7*c1*c3*c6*s4 - d5*c1*c3*s4 - d7*c1*s3*s5*s6 + d7*c1*c3*c4*c5*s6 + d7*c2*c6*s1*s3*s4 - d7*c2*c3*s1*s5*s6 - d7*c2*c4*c5*s1*s3*s6;
    
    J[1][3] = -(c3*s1 + c1*c2*s3)*(d5*(c2*c4 + c3*s2*s4) + d7*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))) - s2*s3*(d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) + d5*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2));
    
    J[1][4] = d7*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))*(c2*c4 + c3*s2*s4) - d7*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4));
    
    J[1][5] = -d7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3)*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))) - d7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4));
    
    J[1][6] = 0;
    
    // Third row (Jv_z - z-direction linear velocity)
    J[2][0] = 0;
    
    J[2][1] = d5*c2*c3*s4 - d5*c4*s2 - d3*s2 - d7*c4*c6*s2 + d7*c2*c3*c6*s4 + d7*c2*s3*s5*s6 - d7*c5*s2*s4*s6 - d7*c2*c3*c4*c5*s6;
    
    J[2][2] = -s2*(d5*s3*s4 + d7*c6*s3*s4 - d7*c3*s5*s6 - d7*c4*c5*s3*s6);
    
    J[2][3] = d5*c3*c4*s2 - d5*c2*s4 - d7*c2*c6*s4 + d7*c3*c4*c6*s2 + d7*c2*c4*c5*s6 + d7*c3*c5*s2*s4*s6;
    
    J[2][4] = d7*s6*(c5*s2*s3 - c2*s4*s5 + c3*c4*s2*s5);
    
    J[2][5] = -d7*(c2*c4*s6 - c2*c5*c6*s4 + c3*s2*s4*s6 - c6*s2*s3*s5 + c3*c4*c5*c6*s2);
    
    J[2][6] = 0;
    
    // Fourth row (Jw_x - x-direction angular velocity)
    J[3][0] = 0;
    J[3][1] = -s1;
    J[3][2] = c1*s2;
    J[3][3] = c3*s1 + c1*c2*s3;
    J[3][4] = s4*(s1*s3 - c1*c2*c3) + c1*c4*s2;
    J[3][5] = s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1 + c1*c2*s3);
    J[3][6] = c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));
    
    // Fifth row (Jw_y - y-direction angular velocity)
    J[4][0] = 0;
    J[4][1] = c1;
    J[4][2] = s1*s2;
    J[4][3] = c2*s1*s3 - c1*c3;
    J[4][4] = c4*s1*s2 - s4*(c1*s3 + c2*c3*s1);
    J[4][5] = c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4);
    J[4][6] = s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)) - c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2);
    
    // Sixth row (Jw_z - z-direction angular velocity)
    J[5][0] = 1;
    J[5][1] = 0;
    J[5][2] = c2;
    J[5][3] = -s2*s3;
    J[5][4] = c2*c4 + c3*s2*s4;
    J[5][5] = c5*s2*s3 - s5*(c2*s4 - c3*c4*s2);
    J[5][6] = s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4);
    
    return J;
}