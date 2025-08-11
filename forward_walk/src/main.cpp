// // 다이나믹셀 제어, 콜백 처리, 컨트롤러, 보행 패턴 생성 관련 클래스 헤더
// #include "dynamixel.hpp"
// #include "callback.hpp"
// #include "dynamixel_controller.hpp"
// #include "Walkingpattern_generator.hpp"

// // IMU 센서 데이터를 저장할 파일 포인터
// FILE *imu_accel;
// FILE *imu_gyro;

// int main(int argc, char **argv)
// {
//     // ROS 1 노드 초기화
//     ros::init(argc, argv, "Main_node");
//     ros::Time::init();             // ROS 시간 초기화
//     ros::Rate loop_rate(100);     // 루프 주기: 100Hz
//     ros::NodeHandle nh;           // ROS 노드 핸들 생성

//     // FTDI USB-to-Serial 장치의 latency timer를 1ms로 설정하여 통신 지연 최소화
//     // sudo 권한 필요. 보안상 udev rule 사용하는 것이 더 좋음
//     system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

//     // 주요 제어 객체 생성
//     Dxl dxl;                          // 다이나믹셀 제어 클래스
//     Trajectory trajectory;           // 보행 궤적 생성기
//     IK_Function IK_;                 // 역기구학 계산기
//     Pick pick;                       // 보행 외 추가 동작용 클래스 (예: 팔, 허리 등)

//     // 보조 제어기: dxl 제어 인터페이스 생성 (Dxl 객체 참조 전달)
//     Dxl_Controller dxl_ctrl(&dxl);

//     // 콜백 처리 객체 생성: 궤적, IK, Dxl, Pick 등 모든 제어 연계
//     Callback callback(&trajectory, &IK_, &dxl, &pick);
//     // callback.Set_Callback();  ← 현재 주석 처리됨. ROS Subscriber 연결 등으로 추정

//     // ROS 메인 루프 시작 (100Hz)
//     while (ros::ok())
//     {
//         // 현재 시점의 보행 궤적, IK 등을 기반으로 모든 관절의 목표 각도 계산
//         callback.Write_All_Theta();

//         // 계산된 목표 각도(All_Theta)를 Dxl 클래스에 전달
//         dxl.SetThetaRef(callback.All_Theta);

//         // 목표 각도들을 모든 다이나믹셀 모터에 동기화 전송
//         dxl.syncWriteTheta();

//         // ROS 콜백 처리 (Subscriber 등)
//         ros::spinOnce();

//         // 루프 주기 유지 (10ms 대기)
//         loop_rate.sleep();

//     }

//     // 루프 종료 시 로그 출력
//     ROS_INFO("Main_node!");

//     // Dxl 객체 소멸자 호출 (불필요함: main 종료 시 자동 소멸)
//     dxl.~Dxl();

//     // 종료 코드 반환
//     return 0;
// }










#include "rclcpp/rclcpp.hpp"                 // ROS 2 기본 헤더
#include "dynamixel.hpp"                     // 사용자 정의: 다이나믹셀 제어 클래스
#include "callback.hpp"                      // 사용자 정의: 콜백 연산 모듈
#include "dynamixel_controller.hpp"          // 사용자 정의: θ 제어 인터페이스
#include "BRP_Kinematics.hpp"
#include "NewPattern2.hpp"
   // 사용자 정의: 보행 궤적 생성기

#include <chrono>
#include <memory>
#include <cstdio>    // For FILE*

using namespace std::chrono_literals;



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

        dxl_ctrl_ = std::make_shared<Dxl_Controller>(dxl_.get());
        callback_ = std::make_shared<Callback>(trajectory_.get(), ik_.get(), dxl_.get(), pick_.get());

        // 타이머: 100Hz (10ms 간격) 주기 제어
        timer_ = this->create_wall_timer(
            10ms,
            std::bind(&MainNode::ControlLoop, this)
        );
    }

private:
    void ControlLoop()
    {
        
        callback_->SelectMotion();
        // 모든 관절의 목표 각도 계산
        callback_->Write_All_Theta();

        // 목표 θ 전달 및 다이나믹셀에 전송
        dxl_->SetThetaRef(callback_->All_Theta);
        dxl_->syncWriteTheta();
    }

    // 구성 요소들 (shared_ptr로 메모리 관리)
    std::shared_ptr<Dxl> dxl_;
    std::shared_ptr<Trajectory> trajectory_;
    std::shared_ptr<IK_Function> ik_;
    std::shared_ptr<Pick> pick_;
    std::shared_ptr<Dxl_Controller> dxl_ctrl_;
    std::shared_ptr<Callback> callback_;

    // ROS 2 타이머
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                            // ROS 2 초기화
    rclcpp::spin(std::make_shared<MainNode>());          // 메인 노드 실행 (타이머 포함)
    rclcpp::shutdown();                                  // 종료 처리
    return 0;
}































// #include "dynamixel.hpp"
// #include "callback.hpp"
// #include "dynamixel_controller.hpp"
// #include "Walkingpattern_generator.hpp"


// FILE *imu_accel;
// FILE *imu_gyro;


// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "Main_node");
//     ros::Time::init();
//     ros::Rate loop_rate(100);
//     ros::NodeHandle nh;

//     // Execute the system command
//     system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

    
//     Dxl dxl;
//     Trajectory trajectory;
//     IK_Function IK_;

//     Pick pick;

//     Dxl_Controller dxl_ctrl(&dxl);
//     Callback callback(&trajectory, &IK_, &dxl, &pick);
//     // callback.Set_Callback();


    
//     while (ros::ok())
//     {
//         callback.Write_All_Theta();
//         dxl.SetThetaRef(callback.All_Theta);
//         dxl.syncWriteTheta();

        

//         ros::spinOnce();
//         loop_rate.sleep();


//     }

//     ROS_INFO("Main_node!");

//     dxl.~Dxl();
//     return 0;
// }








// #include "dynamixel.hpp"
// #include "callback.hpp"
// #include "dynamixel_controller.hpp"
// #include "Walkingpattern_generator.hpp"
// #include <iostream>
// #include <thread>
// #include <chrono>


// int main()
// {
//     // 객체 생성
//     Dxl dxl;
//     Trajectory trajectory;
//     IK_Function IK_;
//     Pick pick;
//     Dxl_Controller dxl_ctrl(&dxl);
//     Callback callback(&trajectory, &IK_, &dxl, &pick);

//     // 100Hz로 루프 설정 (10ms 주기)
//     const int loop_rate_hz = 100;
//     const std::chrono::milliseconds loop_period(1000 / loop_rate_hz);

//     // 메인 루프
//     while (true)
//     {
//         // 모든 관절 각도 갱신
//         callback.Write_All_Theta();

//         // Dynamixel에 새로운 각도값 설정
//         dxl.SetThetaRef(callback.All_Theta);
//         dxl.syncWriteTheta();

//         // 주기적인 대기 (100Hz)
//         std::this_thread::sleep_for(loop_period);
//     }

//     // 객체 소멸 (자동으로 호출됨)
//     dxl.~Dxl();
//     return 0;
// }












// #include "dynamixel.hpp"
// #include "callback.hpp"
// #include "dynamixel_controller.hpp"
// #include "Walkingpattern_generator.hpp"
// #include <chrono>
// #include <thread>
// #include <iostream>


// int main() {
//     // ROS 초기화 부분을 없애고, 그냥 C++ 표준 코드로 변경
//     // 대신 main 함수에서 주기적인 실행을 위한 타이머를 사용합니다.

//     Dxl dxl;
//     Trajectory trajectory;
//     IK_Function IK_;
//     Pick pick;

//     Dxl_Controller dxl_ctrl(&dxl);
//     Callback callback(&trajectory, &IK_, &dxl, &pick);

//     // 주기적인 타이머를 구현하여 기존 loop_rate와 비슷하게 동작하도록 설정
//     const int loop_rate = 100; // 100Hz로 루프를 실행
//     std::chrono::milliseconds loop_interval(10); // 10ms 간격으로 루프 실행

//     while (true) {
//         // 타이머 간격만큼 대기
//         std::this_thread::sleep_for(loop_interval);

//         callback.Write_All_Theta();
//         // 기존 ROS에서 사용한 방식처럼 동일한 동작
//         callback.Write_All_Theta();
//         dxl.SetThetaRef(callback.All_Theta);
//         dxl.syncWriteTheta();

//         // 추가적으로 필요하면 종료 조건을 넣을 수 있음
//         // 예: 키 입력을 받아 종료하는 방법
//         // if (exit_condition) break;
//     }

//     std::cout << "Main_node!" << std::endl;

//     // Dxl 객체 소멸
//     dxl.~Dxl();
//     return 0;
// }


