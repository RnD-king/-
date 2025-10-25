#include "rclcpp/rclcpp.hpp"                 // ROS 2 기본 헤더
#include "dynamixel.hpp"                     // 사용자 정의: 다이나믹셀 제어 클래스
#include "callback.hpp"                      // 사용자 정의: 콜백 연산 모듈
// #include "dynamixel_controller.hpp"       // (선택적 주석 처리)
#include "BRP_Kinematics.hpp"
#include "NewPattern2.hpp"                   // 사용자 정의: 보행 궤적 생성기

#include <chrono>
#include <memory>
#include <cstdio>    // For FILE*

using namespace std::chrono_literals;
using std::chrono::steady_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

int print_counter_ = 0;

class MainNode : public rclcpp::Node
{
public:
    MainNode()
    : Node("main_node")
    {
        // FTDI USB latency timer 설정 (주의: 여전히 sudo 필요)
        system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

        // 객체 초기화
        dxl_ = std::make_shared<Dxl>();
        trajectory_ = std::make_shared<Trajectory>();
        ik_ = std::make_shared<IK_Function>();
        pick_ = std::make_shared<Pick>();

        // dxl_ctrl_ = std::make_shared<Dxl_Controller>(dxl_.get());
        callback_ = std::make_shared<Callback>(trajectory_.get(), ik_.get(), dxl_.get(), pick_.get());

        VectorXd theta_zero = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
        dxl_->MoveToTargetSmoothCos(theta_zero, 150, 10);

        callback_->Set();
        dxl_->MoveToTargetSmoothCos(callback_->All_Theta, 150, 10);
        std::cout << "[Info] Start is half!!!!!!" << std::endl;

        last_time_ = steady_clock::now();  // 타이머 시작 시간 기록

        // 타이머: 100Hz (10ms 간격) 제어 루프
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&MainNode::ControlLoop, this)
        );
    }

private:
    void ControlLoop()
    {
        // === 실질 주기 측정 ===
        auto now = steady_clock::now();
        auto duration = duration_cast<microseconds>(now - last_time_).count();
        double hz = 1e6 / duration;  // microseconds → Hz
        last_time_ = now;

        print_counter_++;
        if (print_counter_ % 100 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "[Control Hz] %.2f Hz", hz);
        }

        // === 기존 제어 루틴 ===
        callback_->SelectMotion();                     // 모션 선택
        callback_->TATA();
        callback_->Write_All_Theta();                  // 목표 θ 계산
        dxl_->SetThetaRef(callback_->All_Theta);       // θ 전달
        dxl_->syncWriteTheta();                        // θ 전송
        
    }






    // 구성 요소들
    std::shared_ptr<Dxl> dxl_;
    std::shared_ptr<Trajectory> trajectory_;
    std::shared_ptr<IK_Function> ik_;
    std::shared_ptr<Pick> pick_;
    // std::shared_ptr<Dxl_Controller> dxl_ctrl_;
    std::shared_ptr<Callback> callback_;

    // ROS 2 타이머
    rclcpp::TimerBase::SharedPtr timer_;

    // 루프 시간 기록
    steady_clock::time_point last_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                            // ROS 2 초기화
    rclcpp::spin(std::make_shared<MainNode>());          // 메인 노드 실행
    rclcpp::shutdown();                                  // 종료 처리
    return 0;
}
