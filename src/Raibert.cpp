#include "Raibert.hpp"

Raibert::Raibert(double k_gain, double step_time, double nominal_step_len)
    : k(k_gain), T_step(step_time), nominal_step_length(nominal_step_len), v_desired(0.0) {}

void Raibert::DesiredVel(double v_des) { v_desired = v_des;}

Eigen::Vector3d Raibert::ComputeNextFootPos(
    const Eigen::Vector3d& com_pos,
    const Eigen::Vector3d& com_vel,
    bool is_left_foot)
{
    Eigen::Vector3d foot_pos = Eigen::Vector3d::Zero();
    double dx = 0.5 * v_desired * T_step + k * (com_vel.x() - v_desired);
    foot_pos.x() = com_pos.x() + dx;
    foot_pos.y() = com_pos.y() + (is_left_foot ? 0.1 : -0.1); // 보폭 폭 조절
    foot_pos.z() = 0.0;  // 지면 착지로 고정

    return foot_pos;
}