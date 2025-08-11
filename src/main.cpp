#include "dynamixel.hpp"
#include "callback.hpp"
#include "dynamixel_controller.hpp"
#include "WalkingPattern.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <memory>

class MainNode : public rclcpp::Node
{
public:
    MainNode()
    : Node("main_node")
    {
        // (시리얼 latency 타이머 조정은 system 명령으로 ROS2에서도 사용 가능)
        system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer");

        // 객체 생성 (전부 heap에 할당)
        dxl_ = std::make_shared<Dxl>();
        trajectory_ = std::make_shared<Trajectory>();
        IK_ = std::make_shared<IK_Function>();
        pick_ = std::make_shared<Pick>();

        dxl_ctrl_ = std::make_shared<Dxl_Controller>(dxl_.get());
        callback_ = std::make_shared<Callback>(trajectory_.get(), IK_.get(), dxl_.get(), pick_.get());

        // Timer: 100Hz (10ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MainNode::TimerCallback, this)
        );

        // 만약 JointState 퍼블리셔 쓰려면 여기에 추가
        // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    ~MainNode()
    {
        // 리소스 정리 필요하면 여기에
        // dxl_는 스마트포인터라 자동 소멸
    }

private:
    void TimerCallback()
    {
        // 각 관절 목표 위치 계산 및 명령 송신
        callback_->Write_All_Theta();
        dxl_->SetThetaRef(callback_->All_Theta);
        dxl_->syncWriteTheta();

        // // JointState 퍼블리시 예시 (필요시 주석 해제)
        // auto msg = sensor_msgs::msg::JointState();
        // msg.header.stamp = this->now();
        // std::vector<std::string> joint_name = {"j1", "j2", "j3"};
        // for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
        // {
        //     msg.name.push_back(joint_name.at(i));
        //     // dxl_->syncReadTheta();
        //     // msg.position.push_back(dxl_->th_[i]);
        // }
        // joint_state_pub_->publish(msg);

        // // FSR 예시
        // // dxl_->FSR_flag();
        // // dxl_->syncWriteTheta();
        // // std::cout << callback_->fsr_value << std::endl;

        // // IMU 예시
        // // dxl_->Quaternino2RPY();

        // // 파일 저장 예시
        // // if (imu_accel) fprintf(imu_accel, "%d %.lf %.lf %.lf\n", t, callback_->Accel(0), callback_->Accel(1), callback_->Accel(2));
        // // if (imu_gyro) fprintf(imu_gyro, "%d %.lf %.lf %.lf\n", t, callback_->Gyro(0), callback_->Gyro(1), callback_->Gyro(2));
    }

    // ROS2에서는 파일 포인터도 필요하면 클래스 멤버로 선언
    FILE *imu_accel = nullptr;
    FILE *imu_gyro = nullptr;

    // ROS2 node 구조에서는 스마트포인터 권장
    std::shared_ptr<Dxl> dxl_;
    std::shared_ptr<Trajectory> trajectory_;
    std::shared_ptr<IK_Function> IK_;
    std::shared_ptr<Pick> pick_;
    std::shared_ptr<Dxl_Controller> dxl_ctrl_;
    std::shared_ptr<Callback> callback_;

    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
