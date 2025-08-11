#include "dynamixel_controller.hpp"

// Constructor
DxlController::DxlController(Dxl *dxl_ptr) : dxl_ptr_(dxl_ptr)
{
}

// 관절 각도 [rad]
Eigen::VectorXd DxlController::GetJointTheta()
{
    Eigen::VectorXd th = dxl_ptr_->GetThetaAct();
    th_cont_ = th;
    return th_cont_;
}

// 관절 각속도 [rad/s]
Eigen::VectorXd DxlController::GetThetaDot()
{
    Eigen::VectorXd th_dot = dxl_ptr_->GetThetaDot();
    th_dot_cont_ = th_dot;
    return th_dot_cont_;
}


//Getter() : 각도의 차이와 이동평균필터를 이용해 각속도 계산 
// 이동평균필터 이용 각속도 계산
Eigen::VectorXd DxlController::GetThetaDotMAF()
{
    Eigen::VectorXd th_dot_est = dxl_ptr_->GetThetaDotEstimated();
    th_dot_cont_ = th_dot_est;

    // 이동평균필터 (윈도우 마지막 행에 최신값 입력)
    maf_.topRows(kWindowSize - 1) = maf_.bottomRows(kWindowSize - 1);
    maf_.row(kWindowSize - 1) = th_dot_cont_.transpose();

    th_dot_mov_avg_filtered_ = maf_.colwise().mean().transpose();
    return th_dot_mov_avg_filtered_;
}

// 토크 [Nm]
Eigen::VectorXd DxlController::GetTorque()
{
    return torque_cont_;
}

// **************************** SETTERS ******************************** //

// 목표 토크 설정 [Nm]
void DxlController::SetTorque(const Eigen::VectorXd& tau)
{
    torque_cont_ = tau;
    dxl_ptr_->SetTorqueRef(torque_cont_);
}

// 목표 각도 설정 [rad]
void DxlController::SetPosition(const Eigen::VectorXd& theta)
{
    th_cont_ = theta;
    dxl_ptr_->SetThetaRef(th_cont_);
}
