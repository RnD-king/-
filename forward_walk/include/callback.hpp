// #ifndef CALLBACK_H
// #define CALLBACK_H

// #include <iostream>
// #include "ros/ros.h"
// #include "sensor_msgs/JointState.h"
// #include "std_msgs/Float32.h"
// #include "std_msgs/Int32.h"
// #include "std_msgs/Float32MultiArray.h"
// #include "std_msgs/UInt8.h"
// #include "std_msgs/Bool.h"
// #include <tf/tf.h>
// #include <boost/thread.hpp>

// #include "dynamixel.hpp"
// #include "Walkingpattern_generator.hpp"

// // #include "ahra/SendMotion.h"

// using Eigen::VectorXd;

// class Callback
// {
// private:
//   double turn_angle = 0;
//   int arm_indext = 0;
//   double z_c = 1.2 * 0.28224;
//   double g = 9.81;
//   double omega_w;
//   double _dt = 0.01;
//   MatrixXd Huddle_RL_theta;
//   MatrixXd Huddle_LL_theta;

// public:
//   enum Motion_Index
//   {
//     InitPose = 0,
//     Forward_2step = 1,
//     Left_2step = 2,
//     Step_in_place = 3,
//     Right_2step = 4,
//     Forward_Nstep = 5,
//     Huddle_Jump = 6,
//     ForWard_fast4step = 7,
//     FWD_UP = 8,
//     BWD_UP = 9,
//     Forward_Halfstep = 10,
//     Left_Halfstep = 11,
//     Right_Halfstep = 12,
//     Back_Halfstep = 13,
//     Back_2step = 33,
//     Forward_1step = 14,
//     Left_6step = 15,
//     Right_6step = 16,
//     START = 50,
//     NONE = 99,

//     //농구
//     Shoot = 17,
//     Ready_to_throw = 18,
//     Right_1step = 19,
//     Left_1step = 20,
//     Grab = 30,
//     FINISH = 100,

//     Huddle_Jump_V2 = 55,

//     Gen2_Walking = 111,
//     Picking_Ball = 112,

//   };

//   string Str_InitPose = "InitPose";
//   string Str_Forward_2step = "Forward_2step";
//   string Str_Forward_1step = "Forward_1step";
//   string Str_Left_2step = "Left_2step";
//   string Str_Step_in_place = "Step_in_place";
//   string Str_Right_2step = "Right_2step";
//   string Str_ForWard_fast4step = "ForWard_fast4step";
//   string Str_Forward_Nstep = "Forward_Nstep";
//   string Str_Huddle_Jump = "Huddle_Jump";
//   string Str_Huddle_Jump_V2 = "Huddle_Jump_V2";
//   string Str_Forward_Halfstep = "Forward_Halfstep";
//   string Str_Left_Halfstep = "Left_Halfstep";
//   string Str_Right_Halfstep = "Right_Halfstep";
//   string Str_Back_Halfstep = "Back_Halfstep";
//   string Str_Back_2step = "Back_2step";
//   string Str_Left_6step = "Left_6step";
//   string Str_Right_6step = "Right_6step";
//   string Str_FWD_UP = "FWD_UP";
//   string Str_BWD_UP = "BWD_UP";
//   string Str_START = "START";
//   string Str_NONE = "NONE";

  
//   string Str_TEST_MODE = "TEST_MODE";

//   string Str_Gen2_Walking = "Gen2_Walking";


//   string Str_Shoot = "Shoot";
//   string Str_Ready_to_throw = "Ready_to_throw";
//   string Str_Left_1step = "Left_1step";
//   string Str_Right_1step = "Right_1step";

//   string Str_Picking_Ball = "Picking_Ball";
//   string Str_FINISH = "FINISH";

//   Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *Pick_ptr);

//   Trajectory *trajectoryPtr;
//   IK_Function *IK_Ptr;
//   Dxl *dxlPtr;
//   Pick* Pick_Ptr;

//   ros::NodeHandle nh;
//   // Function

//   // virtual void SelectMotion(const std_msgs::UInt8::ConstPtr &msg);

//   // ********************************************** Function ************************************************** //
//   virtual void Write_All_Theta();
//   virtual void Check_FSR();
//   void Calculate_Real_CP(int indext, double vx, double vy);
//   void Calculate_ZMP_from_CP(int indext);
//   void Set_Callback();

//   sensor_msgs::JointState joint_state;

// //   // ********************************************** IMU Thread ************************************************** //
// //   void IMUThread();
// //   ros::Subscriber IMU_Velocity_Complementary_x_subscriber_; ///< Gets IMU Sensor data from Move_Decision_node
// //   ros::Subscriber IMU_Velocity_Complementary_y_subscriber_; ///< Gets IMU Sensor data from Move_Decision_node
// //   ros::Subscriber IMU_Velocity_Complementary_z_subscriber_; ///< Gets IMU Sensor data from Move_Decision_node
// //   virtual void VelocityCallback(const std_msgs::Float32::ConstPtr &IMU);

//   // ********************************************** Callback Thread ************************************************** //

//   virtual void callbackThread();
//   ros::Publisher joint_state_publisher_;    ///< Publishes joint states from reads
//   ros::Subscriber joint_state_subscriber_;  ///< Gets joint states for writes
// //   ros::Subscriber FSR_L_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_L
// //   ros::Subscriber FSR_R_sensor_subscriber_; ///< Gets FSR Sensor data from Arduino FSR_R
//   ros::Subscriber Start_subscriber_;        ///< Start

//   virtual void JointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_command);
// //   virtual void L_FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR);
// //   virtual void R_FSRsensorCallback(const std_msgs::UInt8::ConstPtr &FSR);
//   virtual void StartMode(const std_msgs::Bool::ConstPtr &start);

//   /////////Service callbacek
//   ros::ServiceClient client_SendMotion = nh.serviceClient<ahra::SendMotion>("/Move_decision/SendMotion");
//   ahra::SendMotion srv_SendMotion;
//   virtual int generateUniqueRequestID();

//   virtual void SelectMotion();
//   virtual void Move_UD_NeckAngle();
//   virtual void Move_RL_NeckAngle();
//   virtual void TATA();
//   virtual void Emergency();
//   virtual void Motion_Info();
//   virtual void RecieveMotion();

//   // Variable
//   const int SPIN_RATE;
//   VectorXd Goal_joint_ = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
// //   uint8_t L_value = 0;
// //   uint8_t R_value = 0;
// //   uint8_t fsr_value[2] = {L_value, R_value};
//   double rl_neckangle = 0;
//   double ud_neckangle = 0;
//   double tmp_turn_angle = 0;
//   bool emergency_ = 1; // True : Keep going , False : Emergency

//   double vel_x = 0;
//   double vel_y = 0;
//   double vel_z = 0;

//   // PRINT
//   int error_counter = 0;
//   bool error_printed = false;

//   int8_t mode = 99; // NONE
//   double walkfreq = 1.48114;
//   double walktime = 2 / walkfreq;
//   int freq = 100;
//   int walktime_n = walktime * freq;
//   int indext = 0;
//   int upper_indext = 0;
//   int check_indext = 0;
//   int stop_indext = 0;


//   bool turn_left = false;
//   bool turn_right = false;

//   int emergency = 99;
//   bool on_emergency = false;

//   double angle = 0;
//   int index_angle = 0;

//   double step = 0;
//   double RL_th2 = 0;
//     double LL_th2 = 0;
//   double RL_th1 = 0;
// 	double LL_th1 = 0;
//   double HS = 0;
//   double SR = 0;

//   MatrixXd RL_motion;
//   MatrixXd LL_motion;
//   MatrixXd RL_motion0;
//   MatrixXd LL_motion0;
//   MatrixXd RL_motion1;
//   MatrixXd LL_motion1;
//   MatrixXd RL_motion2;
//   MatrixXd LL_motion2;
//   MatrixXd RL_motion3;
//   MatrixXd LL_motion3;
//   MatrixXd RL_motion4;
//   MatrixXd LL_motion4;
//   MatrixXd RL_motion5;
//   MatrixXd LL_motion5;
//   MatrixXd RL_motion6;
//   MatrixXd LL_motion6;
//   MatrixXd RL_motion7;
//   MatrixXd LL_motion7;
//   VectorXd All_Theta = MatrixXd::Zero(NUMBER_OF_DYNAMIXELS, 1);

//   // CP
//   double Real_CP_Y = 0;
//   double Real_CP_X = 0;
//   double xZMP_from_CP = 0;
//   double yZMP_from_CP = 0;
//   double Real_zmp_y_accel = 0;

//   // tf2::Quaternion quaternion;
// };

// #endif // CALLBACK_H



















// #ifndef CALLBACK_H
// #define CALLBACK_H

// // 기본 입출력 및 ROS 관련 메시지, 변환, 쓰레드 등 포함
// #include <iostream>
// #include "ros/ros.h"
// #include "sensor_msgs/JointState.h"
// #include "std_msgs/Float32.h"
// #include "std_msgs/Int32.h"
// #include "std_msgs/Float32MultiArray.h"
// #include "std_msgs/UInt8.h"
// #include "std_msgs/Bool.h"
// #include <tf/tf.h>
// #include <boost/thread.hpp>

// #include "dynamixel.hpp"
// #include "Walkingpattern_generator.hpp"
// // #include "ahra/SendMotion.h"  // 필요 시 사용되는 커스텀 서비스

// using Eigen::VectorXd;  // Eigen 벡터를 간단히 사용하기 위한 선언

// class Callback
// {
// private:
//   // 제어 및 상태 관련 내부 변수들
//   double turn_angle = 0;     // 현재 회전 각도
//   int arm_indext = 0;        // 팔 동작 단계 인덱스
//   double z_c = 1.2 * 0.28224; // ZMP 제어용 로봇 중심 높이 (z_c)
//   double g = 9.81;           // 중력가속도
//   double omega_w;            // ZMP 제어용 자연 주파수
//   double _dt = 0.01;         // 제어 루프 시간 간격 (초)
//   MatrixXd Huddle_RL_theta;  // 허들 동작 시 오른쪽 다리 관절값
//   MatrixXd Huddle_LL_theta;  // 허들 동작 시 왼쪽 다리 관절값

// public:

//   // 생성자
//   Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *Pick_ptr);


//   // ROS 통신용 노드 핸들
//   ros::NodeHandle nh;

//   // ********************************************** Function ************************************************** //
//   virtual void Write_All_Theta();           // 모든 관절 각도 쓰기

//   // ROS 메시지 콜백 함수
//   virtual void StartMode(const std_msgs::Bool::ConstPtr &start);

//   // ************ ROS 서비스 호출 (모션 요청) ************
//   ros::ServiceClient client_SendMotion = nh.serviceClient<ahra::SendMotion>("/Move_decision/SendMotion");
//   ahra::SendMotion srv_SendMotion;


//   // 모션 제어 함수들
//   virtual void SelectMotion();            // 동작 선택

//   // ********************************************** 내부 상태 변수 ********************************************** //
//   const int SPIN_RATE;                    // ROS 루프 스핀 주기


//   double rl_neckangle = 0;                // 오른쪽 목 관절 각도
//   double ud_neckangle = 0;                // 상하 목 관절 각도
//   double tmp_turn_angle = 0;              // 회전 임시 각도
//   bool emergency_ = 1;                    // 긴급 제어 상태 플래그 (true: 정상, false: 긴급정지)

//   // 속도 및 자세 추정 관련 변수
//   double vel_x = 0;
//   double vel_y = 0;
//   double vel_z = 0;

//   // 오류 카운터
//   int error_counter = 0;
//   bool error_printed = false;

//   // 모션/보행 상태 추적용 변수
//   int8_t mode = 99;                       // 현재 모드 (NONE 등)
//   double walkfreq = 1.48114;             // 보행 주파수
//   double walktime = 2 / walkfreq;        // 한 스텝 소요 시간
//   int freq = 100;                        // 제어 주기 (Hz)
//   int walktime_n = walktime * freq;      // 스텝당 제어 반복 횟수
//   int indext = 0;                        // 현재 단계 인덱스
//   int upper_indext = 0;                 // 상체 동작 인덱스
//   int check_indext = 0;                 // 보행 체크 인덱스
//   int stop_indext = 0;                  // 정지 조건 확인 인덱스
//     int go = 0

//   // 회전 제어 관련
//   bool turn_left = false;
//   bool turn_right = false;

//   // 긴급 상태
//   int emergency = 99;
//   bool on_emergency = false;

//   // 방향 관련
//   double angle = 0;
//   int index_angle = 0;

//   // 동작 파라미터
//   double step = 0;
//   double RL_th2 = 0, LL_th2 = 0;
//   double RL_th1 = 0, LL_th1 = 0;
//   double HS = 0;  // 허리 스윙?
//   double SR = 0;  // Shoulder Rotation?

//   // 보행 모션 데이터 저장용 행렬 (양다리, 0~7 단계)
//   MatrixXd RL_motion, LL_motion;
//   MatrixXd RL_motion0, LL_motion0;
//   MatrixXd RL_motion1, LL_motion1;
//   MatrixXd RL_motion2, LL_motion2;
//   MatrixXd RL_motion3, LL_motion3;
//   MatrixXd RL_motion4, LL_motion4;
//   MatrixXd RL_motion5, LL_motion5;
//   MatrixXd RL_motion6, LL_motion6;
//   MatrixXd RL_motion7, LL_motion7;

//   // 전체 관절 각도 저장
//   VectorXd All_Theta = MatrixXd::Zero(NUMBER_OF_DYNAMIXELS, 1);

//   // ZMP (Zero Moment Point) 제어용 변수
//   double Real_CP_Y = 0;
//   double Real_CP_X = 0;
//   double xZMP_from_CP = 0;
//   double yZMP_from_CP = 0;
//   double Real_zmp_y_accel = 0;
// };

// #endif // CALLBACK_H




















// #ifndef CALLBACK_H
// #define CALLBACK_H

// // 기본 입출력 및 ROS 2 관련 메시지, 변환 등 포함
// #include <iostream>
// #include "rclcpp/rclcpp.hpp"                   // ROS 2 헤더
// #include "sensor_msgs/msg/joint_state.hpp"      // ROS 2 메시지 타입
// #include "std_msgs/msg/float32.hpp"             // ROS 2 메시지 타입
// #include "std_msgs/msg/int32.hpp"               // ROS 2 메시지 타입
// #include "std_msgs/msg/float32_multi_array.hpp" // ROS 2 메시지 타입
// #include "std_msgs/msg/u_int8.hpp"              // ROS 2 메시지 타입
// #include "std_msgs/msg/bool.hpp"                // ROS 2 메시지 타입
// #include <tf2/LinearMath/Transform.h>            // tf 관련 라이브러리

// #include "dynamixel.hpp"
// #include "Walkingpattern_generator.hpp" // Trajectory, IK_Function, Pick 클래스가 정의되어 있어야 함

// // Eigen 라이브러리 헤더 포함: MatrixXd, VectorXd 사용을 위해 필수
// #include <eigen3/Eigen/Dense>

// // DEG2RAD는 일반적으로 별도의 헤더 파일이나 전역 상수로 정의되지만,
// // 만약 다른 곳에 없다면 여기에 정의하는 것을 고려해 볼 수 있습니다.
// // #define DEG2RAD 0.017453292519943

// // 전방 선언 (Forward Declarations): Callback 클래스 내에서 포인터로 사용되는 클래스들
// // 이 클래스들의 완전한 정의가 필요하기 전에 컴파일러에게 존재를 알려줍니다.
// // 실제 사용되는 .cpp 파일에서는 해당 클래스의 헤더를 include 해야 합니다.
// class Trajectory;
// class IK_Function;
// class Dxl;
// class Pick; // Walkingpattern_generator.hpp에 Trajectory 클래스가 포함되어 있을 가능성이 높습니다.
//             // Pick 클래스가 별도로 정의되어 있다면 해당 헤더도 포함해야 합니다.


// // Callback 클래스는 ROS 2 노드의 기능을 사용하기 위해 rclcpp::Node를 상속받아야 합니다.
// class Callback : public rclcpp::Node
// {
// private:
//   // 제어 및 상태 관련 내부 변수들
//   double turn_angle = 0;     // 현재 회전 각도
//   int arm_indext = 0;        // 팔 동작 단계 인덱스
//   // g와 z_c는 상수이므로 const로 선언하는 것이 좋습니다.
//   const double z_c = 1.2 * 0.28224; // ZMP 제어용 로봇 중심 높이 (z_c)
//   const double g = 9.81;           // 중력가속도
//   double omega_w;            // ZMP 제어용 자연 주파수
//   double _dt = 0.01;         // 제어 루프 시간 간격 (초)

// public:
//   // 생성자
//   // 생성자 인자 이름과 멤버 변수 이름이 동일하여 발생할 수 있는 모호성을 피하기 위해
//   // 인자 이름에 '_in' 접미사를 추가했습니다.
//   Callback(Trajectory *trajectoryPtr_in, IK_Function *IK_Ptr_in, Dxl *dxlPtr_in, Pick *Pick_Ptr_in);

//   // ROS 2 노드 핸들은 Callback 클래스가 rclcpp::Node를 상속받으므로 더 이상 필요하지 않습니다.
//   // rclcpp::Node::SharedPtr nh; // 삭제

//   // ********************************************** Function ************************************************** //
//   virtual void Write_All_Theta();           // 모든 관절 각도 쓰기

//   // ROS 메시지 콜백 함수
//   // ROS 2에서는 메시지 타입에 대해 SharedPtr를 사용해야 합니다.
//   virtual void StartMode(const std_msgs::msg::Bool::SharedPtr start);


//   // 모션 제어 함수들
//   virtual void SelectMotion();            // 동작 선택

//   // ********************************************** 멤버 변수들 (생성자에서 초기화되는 것 포함) ********************************************** //

//   // 생성자에서 초기화되는 포인터 멤버 변수들 (이전에 누락되어 에러를 유발했던 부분)
//   Trajectory *trajectoryPtr;
//   IK_Function *IK_Ptr;
//   Dxl *dxlPtr;
//   Pick *Pick_Ptr;

//   const int SPIN_RATE;                    // ROS 2 루프 스핀 주기 (생성자에서 초기화됩니다)

//   double rl_neckangle = 0;                // 오른쪽 목 관절 각도
//   double ud_neckangle = 0;                // 상하 목 관절 각도
//   double tmp_turn_angle = 0;              // 회전 임시 각도
//   bool emergency_ = true;                 // 긴급 제어 상태 플래그 (0/1 대신 true/false 권장)

//   // 속도 및 자세 추정 관련 변수
//   double vel_x = 0;
//   double vel_y = 0;
//   double vel_z = 0;

//   // 오류 카운터
//   int error_counter = 0;
//   bool error_printed = false;

//   // 모션/보행 상태 추적용 변수
//   int8_t mode = 99;                       // 현재 모드 (NONE 등)
//   double walkfreq = 1.48114;             // 보행 주파수
//   double walktime = 2 / walkfreq;        // 한 스텝 소요 시간
//   int freq = 100;                        // 제어 주기 (Hz)
//   int walktime_n = walktime * freq;      // 스텝당 제어 반복 횟수
//   int indext = 0;                        // 현재 단계 인덱스
//   int upper_indext = 0;                 // 상체 동작 인덱스
//   int check_indext = 0;                 // 보행 체크 인덱스
//   int stop_indext = 0;                  // 정지 조건 확인 인덱스
//   int go = 0;                            // 동작 시작 여부

//   // 회전 제어 관련
//   bool turn_left = false;
//   bool turn_right = false;

//   // 긴급 상태
//   int emergency = 99; // 이 변수도 bool 타입으로 사용하는 것이 더 명확합니다.
//   bool on_emergency = false;

//   // 방향 관련
//   double angle = 0;
//   int index_angle = 0;

//   // 동작 파라미터 (Write_All_Theta에서 사용됨)
//   double step = 0;
//   double RL_th2 = 0, LL_th2 = 0;
//   double RL_th1 = 0, LL_th1 = 0;
//   double HS = 0;  // 허리 스윙?
//   double SR = 0;  // Shoulder Rotation?

//   // 보행 모션 데이터 저장용 행렬 (이들은 Trajectory 클래스의 멤버 변수일 가능성이 높습니다.
//   // Callback 클래스에서 직접 이들을 제어한다면 여기에 두지만, 중복될 수 있습니다.)
//   // MatrixXd RL_motion, LL_motion;
//   // MatrixXd RL_motion0, LL_motion0;
//   // ... (나머지 RL_motionN, LL_motionN)

//   // 전체 관절 각도 저장 (Write_All_Theta에서 All_Theta[22]까지 접근하므로 23개 요소가 필요)
//   // NUMBER_OF_DYNAMIXELS가 정의되어 있다면 해당 값을 사용하고, 아니면 23으로 고정합니다.
//   // Dynamixel.hpp에 NUMBER_OF_DYNAMIXELS가 정의되어 있을 가능성이 높습니다.
//   Eigen::VectorXd All_Theta = Eigen::VectorXd::Zero(23); // VectorXd::Zero 사용

//   // ZMP (Zero Moment Point) 제어용 변수
//   double Real_CP_Y = 0;
//   double Real_CP_X = 0;
//   double xZMP_from_CP = 0;
//   double yZMP_from_CP = 0;
//   double Real_zmp_y_accel = 0;

//   // ROS 2 Subscription 객체 (callback.cpp에서 create_subscription 사용 시 필요)
//   rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Start_subscriber_;

//   // ahra 서비스 클라이언트 및 요청 객체 (ahra 서비스 사용 시 주석 해제)
//   // rclcpp::Client<ahra_interfaces::srv::SendMotion>::SharedPtr client_SendMotion;
//   // ahra_interfaces::srv::SendMotion::Request::SharedPtr srv_SendMotion;
// };

// #endif // CALLBACK_H













// #ifndef CALLBACK_HPP
// #define CALLBACK_HPP

// #include <memory>
// #include <Eigen/Dense>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include "dynamixel.hpp"
// #include "Walkingpattern_generator.hpp"


// // Forward declarations for classes used in the callback
// class Trajectory;
// class IK_Function;
// class Dxl;
// class Pick;

// class Callback
// {
// public:
//     // Constructor declaration
//     Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *Pick_Ptr);

//     // Function to start the mode based on incoming subscription
//     void StartMode(const std_msgs::msg::Bool::SharedPtr start);

//     // Function to select motion for the robot
//     void SelectMotion();

//     // Function to write all theta values (angles for robot's joints)
//     void Write_All_Theta();

// private:
//     // Trajectory and IK-related pointers
//     Trajectory *trajectoryPtr;
//     IK_Function *IK_Ptr;
//     Dxl *dxlPtr;
//     Pick *Pick_Ptr;

//     // Internal states for emergency and mode control
//     int emergency = 0;
//     int go = 0;
//     int indext = 0;

//     // Omega value for walking dynamics
//     double omega_w;

//     // Matrices for robot joint trajectories
//     Eigen::MatrixXd Ref_RL_x, Ref_LL_x, Ref_RL_y, Ref_LL_y, Ref_RL_z, Ref_LL_z;
//     Eigen::MatrixXd Ref_WT_th, Ref_RA_th, Ref_LA_th, Ref_NC_th;

//     // All theta values for controlling joints (36 in total)
//     double All_Theta[36];

//     // Subscriber for start mode
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Start_subscriber_;

//     // Constants for specific joint adjustments (set in `Write_All_Theta`)
//     const double RL_th1 = 10.0, RL_th2 = 17.0; // Degrees
//     const double LL_th1 = 10.0, LL_th2 = 17.0; // Degrees
//     const double DEG2RAD = 3.14159 / 180.0;
//     const double SR = 10.0, HS = 15.0; // Specific adjustments
//     const double step = 1.0;

//     // Other helper methods or constants can be added as needed
// };

// #endif // CALLBACK_HPP
















// #ifndef CALLBACK_H
// #define CALLBACK_H

// #include <iostream>
// #include <rclcpp/rclcpp.hpp>
// #include "std_msgs/msg/bool.hpp"  // std_msgs::msg::Bool 사용
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "std_msgs/msg/float32.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "std_msgs/msg/float32_multi_array.hpp"
// #include "std_msgs/msg/u_int8.hpp"
// #include <tf2/LinearMath/Quaternion.h>
// #include <boost/thread.hpp>
// #include <Eigen/Dense>  // Eigen 관련 헤더 추가


// #include "dynamixel.hpp"
// #include "Walkingpattern_generator.hpp"

// using Eigen::VectorXd;  // Eigen 벡터를 간단히 사용하기 위한 선언

// class Callback : public rclcpp::Node
// {
// private:
//     double turn_angle = 0;     
//     int arm_indext = 0;        
//     double z_c = 1.2 * 0.28224; 
//     double g = 9.81;           
//     double omega_w;            
//     double _dt = 0.01;         
//     MatrixXd Huddle_RL_theta;  
//     MatrixXd Huddle_LL_theta;  

//     Trajectory *trajectoryPtr;    // Trajectory 객체를 가리키는 포인터
//     IK_Function *IK_Ptr;          // IK_Function 객체를 가리키는 포인터
//     Dxl *dxlPtr;                  // Dxl 객체를 가리키는 포인터
//     Pick *Pick_Ptr;               // Pick 객체를 가리키는 포인터

// public:
//     Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *Pick_Ptr);

//     // ROS 메시지 콜백 함수
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Start_subscriber_;
//     virtual void StartMode(const std_msgs::msg::Bool::SharedPtr start);

//     // 모션 제어 함수들
//     virtual void SelectMotion();            
//     virtual void Write_All_Theta();           

//     const int SPIN_RATE = 100;                   

//     double rl_neckangle = 0;                
//     double ud_neckangle = 0;                
//     double tmp_turn_angle = 0;              
//     bool emergency_ = 1;                    
//     double vel_x = 0;
//     double vel_y = 0;
//     double vel_z = 0;
//     int error_counter = 0;
//     bool error_printed = false;

//     int8_t mode = 99;                       
//     double walkfreq = 1.48114;             
//     double walktime = 2 / walkfreq;        
//     int freq = 100;                        
//     int walktime_n = walktime * freq;      
//     int indext = 0;                        
//     int upper_indext = 0;                 
//     int check_indext = 0;                 
//     int stop_indext = 0;                  
//     int go = 0;

//     bool turn_left = false;
//     bool turn_right = false;

//     int emergency = 99;
//     bool on_emergency = false;

//     double angle = 0;
//     int index_angle = 0;

//     double step = 0;
//     double RL_th2 = 0, LL_th2 = 0;
//     double RL_th1 = 0, LL_th1 = 0;
//     double HS = 0;  
//     double SR = 0;  

//     MatrixXd RL_motion, LL_motion;
//     MatrixXd RL_motion0, LL_motion0;
//     MatrixXd RL_motion1, LL_motion1;
//     MatrixXd RL_motion2, LL_motion2;
//     MatrixXd RL_motion3, LL_motion3;
//     MatrixXd RL_motion4, LL_motion4;
//     MatrixXd RL_motion5, LL_motion5;
//     MatrixXd RL_motion6, LL_motion6;
//     MatrixXd RL_motion7, LL_motion7;

//     VectorXd All_Theta = MatrixXd::Zero(NUMBER_OF_DYNAMIXELS, 1);

//     double Real_CP_Y = 0;
//     double Real_CP_X = 0;
//     double xZMP_from_CP = 0;
//     double yZMP_from_CP = 0;
//     double Real_zmp_y_accel = 0;
// };

// #endif // CALLBACK_H










































#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"  // std_msgs::msg::Bool 사용
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <boost/thread.hpp>
#include <Eigen/Dense>  // Eigen 관련 헤더 추가

#include "dynamixel.hpp"
#include "BRP_Kinematics.hpp"
#include "NewPattern2.hpp"


using Eigen::VectorXd;  // Eigen 벡터를 간단히 사용하기 위한 선언

class Callback : public rclcpp::Node
{
private:
    double turn_angle = 0;     
    int arm_indext = 0;        
    double z_c = 1.2 * 0.28224; 
    double g = 9.81;           
    double omega_w;            
    double _dt = 0.01;         
    MatrixXd Huddle_RL_theta;  
    MatrixXd Huddle_LL_theta;  

    Trajectory *trajectoryPtr;    // Trajectory 객체를 가리키는 포인터
    IK_Function *IK_Ptr;          // IK_Function 객체를 가리키는 포인터
    Dxl *dxlPtr;                  // Dxl 객체를 가리키는 포인터
    Pick *Pick_Ptr;               // Pick 객체를 가리키는 포인터


    double Goal_joint_[NUMBER_OF_DYNAMIXELS];

public:
    Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *Pick_Ptr);
    

    // ROS 메시지 콜백 함수
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Start_subscriber_;
    virtual void StartMode(const std_msgs::msg::Bool::SharedPtr start);

    // 모션 제어 함수들
    virtual void SelectMotion();            
    virtual void Write_All_Theta();           

    const int SPIN_RATE = 100;                   

    double rl_neckangle = 0;                
    double ud_neckangle = 0;                
    double tmp_turn_angle = 0;              
    bool emergency_ = 1;                    
    double vel_x = 0;
    double vel_y = 0;
    double vel_z = 0;
    int error_counter = 0;
    bool error_printed = false;

    int8_t mode = 99;                       
    double walkfreq = 1.48114;             
    double walktime = 2 / walkfreq;        
    int freq = 100;                        
    int walktime_n = walktime * freq;      
    int indext = 0;                        
    int upper_indext = 0;                 
    int check_indext = 0;                 
    int stop_indext = 0;                  
    int go = 0;
    int re = 0;

    bool turn_left = false;
    bool turn_right = false;

    int emergency = 99;
    bool on_emergency = false;

    double angle = 0;
    int index_angle = 0;

    double step = 0;
    double RL_th2 = 0, LL_th2 = 0;
    double RL_th1 = 0, LL_th1 = 0;
    double HS = 0;  
    double SR = 0;  

    MatrixXd RL_motion, LL_motion;
    MatrixXd RL_motion0, LL_motion0;
    MatrixXd RL_motion1, LL_motion1;
    MatrixXd RL_motion2, LL_motion2;
    MatrixXd RL_motion3, LL_motion3;
    MatrixXd RL_motion4, LL_motion4;
    MatrixXd RL_motion5, LL_motion5;
    MatrixXd RL_motion6, LL_motion6;
    MatrixXd RL_motion7, LL_motion7;

    VectorXd All_Theta = MatrixXd::Zero(NUMBER_OF_DYNAMIXELS, 1);

    double Real_CP_Y = 0;
    double Real_CP_X = 0;
    double xZMP_from_CP = 0;
    double yZMP_from_CP = 0;
    double Real_zmp_y_accel = 0;

    void callbackThread();
};

#endif // CALLBACK_H
