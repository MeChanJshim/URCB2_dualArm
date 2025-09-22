#include "Y2Kinematics/Kinematics.hpp"

// Constructor for Kinematics class
Kinematics::Kinematics(double SamplingTime_, size_t numOfAxis_, const YMatrix& EE2TCP_)
:dt(SamplingTime_), numOfAxis(numOfAxis_), EE2TCP(EE2TCP_) {
    // Joint limits
    q_min.assign(numOfAxis,-2*M_PI);
    q_max.assign(numOfAxis,2*M_PI);
    qd_min.assign(numOfAxis,(q_min[0]/2)/dt);
    qd_max.assign(numOfAxis,(q_max[0]/2)/dt);
}

// Enhanced QP-based IK solver
std::vector<double> Kinematics::solve_IK(const std::vector<double>& q_current, 
                                const YMatrix& target_HTM_) {
    
    // 0. Transform target HTM to match EE2TCP
    auto target_HTM = target_HTM_*EE2TCP.inverse();

    // 1. Calculate current end-effector pose
    YMatrix current_HTM = forwardKinematics(q_current);
    current_HTM = current_HTM*EE2TCP.inverse();

    /* 정밀도 제거 */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            current_HTM[i][j] = roundToNthDecimal(current_HTM[i][j], QP_precision);
            target_HTM[i][j] = roundToNthDecimal(target_HTM[i][j], QP_precision);
        }
    }
    
    // 2. Calculate pose errors
    std::vector<double> e_pos(3);
    for (int i = 0; i < 3; i++) {
        e_pos[i] = target_HTM[i][3] - current_HTM[i][3];
    }
    
    YMatrix R_target = target_HTM.extract(0, 0, 3, 3);
    YMatrix R_current = current_HTM.extract(0, 0, 3, 3);
    YMatrix R_err = R_target * R_current.transpose();
    // auto R_TarSpatial = R_target.toSpatialAngle();
    // auto R_CurSpatial = R_current.toSpatialAngle();
    // SpatialAngle spatial_error = R_TarSpatial - R_CurSpatial;
    
    // std::vector<double> e_rot = {spatial_error.x, spatial_error.y, spatial_error.z};
    
    std::vector<double> e_rot(3);
    e_rot[0] = 0.5 * (R_err[2][1] - R_err[1][2]);
    e_rot[1] = 0.5 * (R_err[0][2] - R_err[2][0]);
    e_rot[2] = 0.5 * (R_err[1][0] - R_err[0][1]);
    
    std::vector<double> Delta_x_des(6);
    for (int i = 0; i < 3; i++) {
        Delta_x_des[i] = Kp_pos * e_pos[i];
        Delta_x_des[i+3] = Kp_rot * e_rot[i];
    }
    
    // 3. Calculate Jacobian
    YMatrix J = calculateJacobian(q_current);
    
    // 4. Setup QP matrices
    YMatrix I = YMatrix::identity(numOfAxis);
    YMatrix JtJ = J.transpose() * J;
    
    // H = 2*omega_p*I + 2*alpha*(J'*J + lambda*I)
    YMatrix H1 = I * (2.0 * omega_p);
    YMatrix H2 = (JtJ + I * lambda) * (2.0 * alpha);
    YMatrix H = H1 + H2;
    
    // f = -2*omega_p*q_current
    std::vector<double> f(numOfAxis);
    for (int i = 0; i < numOfAxis; i++) {
        f[i] = -2.0 * omega_p * q_current[i];
    }
    
    // Equality constraint: J*q = Delta_x_des + J*q_current
    YMatrix q_matrix(numOfAxis, 1);
    for (int i = 0; i < numOfAxis; i++) {
        q_matrix[i][0] = q_current[i];
    }
    YMatrix Jq = J * q_matrix;
    
    std::vector<double> beq(6);
    for (int i = 0; i < 6; i++) {
        beq[i] = Delta_x_des[i] + Jq[i][0];
    }
    
    // Joint limits
    std::vector<double> lb(numOfAxis), ub(numOfAxis);
    for (int i = 0; i < numOfAxis; i++) {
        double lb_angle = q_min[i];
        double ub_angle = q_max[i];
        double lb_vel = q_current[i] + dt * qd_min[i];
        double ub_vel = q_current[i] + dt * qd_max[i];
        
        lb[i] = std::max(lb_angle, lb_vel);
        ub[i] = std::min(ub_angle, ub_vel);
    }
    
    // 5. Solve QP
    QPSolver::QPResult result = qp_solver.solve(H, f, J, beq, lb, ub, QP_tolerance);
    
    if (!result.success) {
        std::cerr << "Warning: QP solver failed with status " << result.status << std::endl;
        return q_current;  // Return current angles if failed
    }
    
    return result.solution;
}
// std::vector<double> Kinematics::solve_IK(const std::vector<double>& q_current, 
//                                             const YMatrix& target_HTM_) {
    
//     // 0. Transform target HTM to match EE2TCP
//     auto target_HTM = target_HTM_*EE2TCP.inverse();

//     // 1. Calculate current end-effector pose
//     YMatrix current_HTM = forwardKinematics(q_current);
    
//     // 2. Calculate pose errors - POSITION은 기존 방식 유지
//     std::vector<double> e_pos(3);
//     for (int i = 0; i < 3; i++) {
//         e_pos[i] = target_HTM[i][3] - current_HTM[i][3];  // ✅ 위치는 그대로
//     }
    
//     // 3. ORIENTATION만 개선된 방식 사용 ⭐
//     YMatrix R_target = target_HTM.extract(0, 0, 3, 3);
//     YMatrix R_current = current_HTM.extract(0, 0, 3, 3);
    
//     std::vector<double> e_rot(3);
    
//     // 방향 오차가 큰지 확인
//     YMatrix R_err = R_target * R_current.transpose();
//     double trace = R_err[0][0] + R_err[1][1] + R_err[2][2];
//     double angle_error = acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
    
//     if (angle_error > 0.3) {  // 약 17도 이상의 큰 방향 오차
//         // 큰 방향 변화: Spatial Angle 방식 사용
//         SpatialAngle target_spatial = R_target.toSpatialAngle();
//         SpatialAngle current_spatial = R_current.toSpatialAngle();
//         SpatialAngle spatial_error = target_spatial - current_spatial;
        
//         // 최단 경로 회전 보장
//         if (spatial_error.magnitude() > M_PI) {
//             double scale = M_PI / spatial_error.magnitude();
//             spatial_error = spatial_error * scale;
//         }
        
//         e_rot[0] = spatial_error.x;
//         e_rot[1] = spatial_error.y;
//         e_rot[2] = spatial_error.z;
        
//         std::cout << "Large orientation change detected: " << angle_error << " rad. Using spatial angle method." << std::endl;
        
//     } else {
//         // 작은 방향 변화: 기존 방식 사용
//         e_rot[0] = 0.5 * (R_err[2][1] - R_err[1][2]);
//         e_rot[1] = 0.5 * (R_err[0][2] - R_err[2][0]);
//         e_rot[2] = 0.5 * (R_err[1][0] - R_err[0][1]);
//     }
    
//     // 4. 적응형 제어 게인 (방향 오차가 클 때 게인 감소)
//     double adaptive_Kp_rot = Kp_rot;
//     if (angle_error > 0.5) {
//         adaptive_Kp_rot *= 0.3;  // 큰 오차일 때 게인 감소
//     } else if (angle_error > 0.2) {
//         adaptive_Kp_rot *= 0.6;  // 중간 오차일 때 게인 약간 감소
//     }
    
//     std::vector<double> Delta_x_des(6);
//     for (int i = 0; i < 3; i++) {
//         Delta_x_des[i] = Kp_pos * e_pos[i];           // 위치는 기존 게인
//         Delta_x_des[i+3] = adaptive_Kp_rot * e_rot[i]; // 방향은 적응형 게인
//     }
    
//     // 5. 적응형 댐핑 (큰 방향 오차일 때 댐핑 증가)
//     double adaptive_lambda = lambda;
//     if (angle_error > 0.5) {
//         adaptive_lambda *= 5.0;  // 큰 방향 오차일 때 더 큰 댐핑
//     }
    
//     // 6. Calculate Jacobian
//     YMatrix J = calculateJacobian(q_current);
    
//     // 7. Setup QP matrices (적응형 댐핑 적용)
//     YMatrix I = YMatrix::identity(numOfAxis);
//     YMatrix JtJ = J.transpose() * J;
    
//     // H = 2*omega_p*I + 2*alpha*(J'*J + adaptive_lambda*I)
//     YMatrix H1 = I * (2.0 * omega_p);
//     YMatrix H2 = (JtJ + I * adaptive_lambda) * (2.0 * alpha);
//     YMatrix H = H1 + H2;
    
//     // f = -2*omega_p*q_current
//     std::vector<double> f(numOfAxis);
//     for (int i = 0; i < numOfAxis; i++) {
//         f[i] = -2.0 * omega_p * q_current[i];
//     }
    
//     // Equality constraint: J*q = Delta_x_des + J*q_current
//     YMatrix q_matrix(numOfAxis, 1);
//     for (int i = 0; i < numOfAxis; i++) {
//         q_matrix[i][0] = q_current[i];
//     }
//     YMatrix Jq = J * q_matrix;
    
//     std::vector<double> beq(6);
//     for (int i = 0; i < 6; i++) {
//         beq[i] = Delta_x_des[i] + Jq[i][0];
//     }
    
//     // Joint limits (기존과 동일)
//     std::vector<double> lb(numOfAxis), ub(numOfAxis);
//     for (int i = 0; i < numOfAxis; i++) {
//         double lb_angle = q_min[i];
//         double ub_angle = q_max[i];
//         double lb_vel = q_current[i] + dt * qd_min[i];
//         double ub_vel = q_current[i] + dt * qd_max[i];
        
//         lb[i] = std::max(lb_angle, lb_vel);
//         ub[i] = std::min(ub_angle, ub_vel);
//     }
    
//     // 8. Solve QP
//     QPSolver::QPResult result = qp_solver.solve(H, f, J, beq, lb, ub);
    
//     if (!result.success) {
//         std::cerr << "Warning: QP solver failed with status " << result.status << std::endl;
//         return q_current;  // Return current angles if failed
//     }
    
//     return result.solution;
// }    

// Utility methods
double Kinematics::roundToNthDecimal(double value, int n) {
    double multiplier = pow(10.0, n);
    return round(value * multiplier) / multiplier;
}

void Kinematics::setControlGains(double kp_pos, double kp_rot) {
    Kp_pos = kp_pos;
    Kp_rot = kp_rot;
}

void Kinematics::setQPWeights(double omega_p_val, double alpha_val, double lambda_val) {
    omega_p = omega_p_val;
    alpha = alpha_val;
    lambda = lambda_val;
} 

void Kinematics::setJointLimits(const std::vector<double>& q_min, 
                    const std::vector<double>& q_max,
                    const std::vector<double>& qd_min, 
                    const std::vector<double>& qd_max) {

    if (q_min.size() != numOfAxis) {
        throw std::invalid_argument("Joint limits must have " + std::to_string(numOfAxis) + " elements");
    }
    this->q_min = q_min;
    this->q_max = q_max;
    this->qd_min = qd_min;
    this->qd_max = qd_max;
}

// Print pose in a readable format
void Kinematics::printPose(const YMatrix& pose, const std::string& label) {
    std::cout << label << " Pose:\n";
    for (const auto& row : pose) {
        for (const auto& val : row) {
            std::cout << val << " ";
        }
        std::cout << "\n";
    }
}

