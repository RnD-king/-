#include "callback.hpp"

// ROS2에서는 전역 변수 권장하지 않지만, 임시로 유지
bool flgflg = 0;
FILE* Trajectory_all = nullptr;

// 생성자: ROS2는 Node를 외부에서 받아와 사용 (의존성 주입 패턴)
// node_ptr은 std::shared_ptr<rclcpp::Node>
Callback::Callback(Trajectory* trajectory_ptr, IK_Function* ik_ptr, Dxl* dxl_ptr, Pick* pick_ptr, std::shared_ptr<rclcpp::Node> node_ptr)
    : trajectory_ptr_(trajectory_ptr),
      ik_ptr_(ik_ptr),
      dxl_ptr_(dxl_ptr),
      pick_ptr_(pick_ptr),
      spin_rate_(100),
      node_ptr_(node_ptr)
{
    // ROS2: thread 대신 timer/멀티스레드 executor 사용 권장 (임시 thread 예시)
    std::thread queue_thread([this]() { this->callbackThread(); });
    std::thread imu_thread([this]() { this->IMUThread(); });
    queue_thread.detach();
    imu_thread.detach();

    // Eigen 초기화
    trajectory_ptr_->Ref_RL_x = Eigen::MatrixXd::Zero(1, 675);
    trajectory_ptr_->Ref_LL_x = Eigen::MatrixXd::Zero(1, 675);
    trajectory_ptr_->Ref_RL_y = -0.06 * Eigen::MatrixXd::Ones(1, 675);
    trajectory_ptr_->Ref_LL_y = 0.06 * Eigen::MatrixXd::Ones(1, 675);
    trajectory_ptr_->Ref_RL_z = Eigen::MatrixXd::Zero(1, 675);       // Axis-z = ground -> set 0
    trajectory_ptr_->Ref_LL_z = Eigen::MatrixXd::Zero(1, 675);
    trajectory_ptr_->Turn_Trajectory = Eigen::VectorXd::Zero(135);
    omega_w_ = sqrt(g_ / z_c_);

    pick_ptr_->Ref_WT_th = Eigen::MatrixXd::Zero(1, 675);
    pick_ptr_->Ref_RA_th = Eigen::MatrixXd::Zero(4, 675);
    pick_ptr_->Ref_LA_th = Eigen::MatrixXd::Zero(4, 675);
    pick_ptr_->Ref_NC_th = Eigen::MatrixXd::Zero(2, 675);

    Trajectory_all = fopen("/home/ryu/Trajectory/trajectoryall.dat", "w");
}

// sensor_msgs::JointState → sensor_msgs::msg::JointState
void Callback::JointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr joint_command)
{
    for (int i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        goal_joint_[i] = joint_command->position[i];
        dxl_ptr_->SetThetaRef(goal_joint_);
    }
}

// std_msgs::UInt8 → std_msgs::msg::UInt8
void Callback::L_FSRsensorCallback(const std_msgs::msg::UInt8::SharedPtr fsr)
{
    r_value_ = fsr->data; // Left_foot_FSR
}

void Callback::R_FSRsensorCallback(const std_msgs::msg::UInt8::SharedPtr fsr)
{
    l_value_ = fsr->data; // Right_foot_FSR
    // RCLCPP_INFO(node_ptr_->get_logger(), "%d", l_value_);
}

void Callback::StartMode(const std_msgs::msg::Bool::SharedPtr start)
{
    if (start->data) {
        indext_ = 0;
        trajectory_ptr_->Ref_RL_x = Eigen::MatrixXd::Zero(1, 30);
        trajectory_ptr_->Ref_LL_x = Eigen::MatrixXd::Zero(1, 30);
        trajectory_ptr_->Ref_RL_y = -0.06 * Eigen::MatrixXd::Ones(1, 30);
        trajectory_ptr_->Ref_LL_y = 0.06 * Eigen::MatrixXd::Ones(1, 30);
        trajectory_ptr_->Ref_RL_z = Eigen::MatrixXd::Zero(1, 30);
        trajectory_ptr_->Ref_LL_z = Eigen::MatrixXd::Zero(1, 30);
        pick_ptr_->Ref_WT_th = Eigen::MatrixXd::Zero(1, 30);
        pick_ptr_->Ref_RA_th = Eigen::MatrixXd::Zero(4, 30);
        pick_ptr_->Ref_LA_th = Eigen::MatrixXd::Zero(4, 30);
        pick_ptr_->Ref_NC_th = Eigen::MatrixXd::Zero(2, 30);

        srv_SendMotion_.request.st_finish = true;
        // srv_SendMotion_.response.select_motion = Motion_Index::Right_Halfstep; 

        flgflg = 1;
        srv_SendMotion_.response.select_motion = Motion_Index::Shoot;

        emergency_ = 0;
        srv_SendMotion_.request.ta_finish = true;
    } else {
        indext_ = 0;
        trajectory_ptr_->Ref_RL_x = Eigen::MatrixXd::Zero(1, 30);
        trajectory_ptr_->Ref_LL_x = Eigen::MatrixXd::Zero(1, 30);
        trajectory_ptr_->Ref_RL_y = -0.06 * Eigen::MatrixXd::Ones(1, 30);
        trajectory_ptr_->Ref_LL_y = 0.06 * Eigen::MatrixXd::Ones(1, 30);
        trajectory_ptr_->Ref_RL_z = Eigen::MatrixXd::Zero(1, 30);
        trajectory_ptr_->Ref_LL_z = Eigen::MatrixXd::Zero(1, 30);
        pick_ptr_->Ref_WT_th = Eigen::MatrixXd::Zero(1, 30);
        pick_ptr_->Ref_RA_th = Eigen::MatrixXd::Zero(4, 30);
        pick_ptr_->Ref_LA_th = Eigen::MatrixXd::Zero(4, 30);
        pick_ptr_->Ref_NC_th = Eigen::MatrixXd::Zero(2, 30);

        flgflg = 1;
        srv_SendMotion_.response.select_motion = Motion_Index::Ready_to_throw;

        // srv_SendMotion_.response.turn_angle = -15;
        emergency_ = 0;
        srv_SendMotion_.request.ta_finish = true;
        // RCLCPP_INFO(node_ptr_->get_logger(), "Picking_Ball!");
    }
}


/////////////////////////////////////////////// About Subscribe IMUThread ///////////////////////////////////////////////

// sensor_msgs::msg::Float32::SharedPtr로 변경
void Callback::VelocityCallback(const std_msgs::msg::Float32::SharedPtr imu_msg)
{
    // 실제 센서에 따라 data 값 분리 필요
    vel_x_ = imu_msg->data;
    vel_y_ = imu_msg->data;
    vel_z_ = imu_msg->data;
    // RCLCPP_ERROR(node_ptr_->get_logger(), "X : %f", vel_x_);
}

// ROS2: Subscription 방식
void Callback::IMUThread()
{
    // rclcpp::Node에서 subscription 생성 (nh → node_ptr_)
    imu_velocity_complementary_x_sub_ = node_ptr_->create_subscription<std_msgs::msg::Float32>(
        "/filtered/Velocity_Complementary/x", 10,
        std::bind(&Callback::VelocityCallback, this, std::placeholders::_1));
    imu_velocity_complementary_y_sub_ = node_ptr_->create_subscription<std_msgs::msg::Float32>(
        "/filtered/Velocity_Complementary/y", 10,
        std::bind(&Callback::VelocityCallback, this, std::placeholders::_1));
    imu_velocity_complementary_z_sub_ = node_ptr_->create_subscription<std_msgs::msg::Float32>(
        "/filtered/Velocity_Complementary/z", 10,
        std::bind(&Callback::VelocityCallback, this, std::placeholders::_1));

    // ROS2에서는 loop_rate/while 보다는 timer 사용 권장, 임시 예시
    rclcpp::Rate loop_rate(200);
    Set_Callback();
    while (rclcpp::ok())
    {
        // Calculate_Real_CP(indext_, vel_x_, vel_y_);
        rclcpp::spin_some(node_ptr_);
        loop_rate.sleep();
        // RCLCPP_INFO(node_ptr_->get_logger(), "%lf", vel_x_);
    }
}

// Unique ID 함수는 그대로 사용 가능
int Callback::generateUniqueRequestID()
{
    srand(static_cast<unsigned int>(std::time(0)));
    return (rand() % 10000) + 1;
}


void Callback::callbackThread()
{
    ros::NodeHandle nh(ros::this_node::getName());

    joint_state_publisher_ = nh.advertise<sensor_msgs::JointState>("KWJ_joint_states", 100);
    joint_state_subscriber_ = nh.subscribe("KWJ_desired_joint_states", 1000, &Callback::JointStatesCallback, this);

    FSR_L_sensor_subscriber_ = nh.subscribe("/FSR_L", 1000, &Callback::L_FSRsensorCallback, this);
    FSR_R_sensor_subscriber_ = nh.subscribe("/FSR_R", 1000, &Callback::R_FSRsensorCallback, this);
    Start_subscriber_ = nh.subscribe("/START", 1000, &Callback::StartMode, this);

    srv_SendMotion.request.SM_finish = false;
    srv_SendMotion.request.TA_finish = false;
    srv_SendMotion.request.UD_finish = false;
    srv_SendMotion.request.RL_finish = true;
    srv_SendMotion.request.EM_finish = true;
    srv_SendMotion.request.ST_finish = true;
    srv_SendMotion.request.walkcount = 0;


    ros::Rate loop_rate(SPIN_RATE);
//     while (nh.ok())
//     {
//         srv_SendMotion.request.request_id = generateUniqueRequestID(); // generate a unique request ID

//         if (flgflg) //client_SendMotion.call(srv_SendMotion)
//         {
//                         flgflg = false;
//                                     srv_SendMotion.response.success = true;
//             if (srv_SendMotion.response.success)
//             {
//                 Motion_Info();
//                 RecieveMotion();
//             }
//             else if (!error_printed)
//             {
//                 if (error_counter < 3) // Check the counter
//                 {
//                     // ROS_INFO("\n");
//                     // ROS_ERROR("Failed to call service");
//                     // ROS_INFO("\n");
//                     error_printed = true; // Set the flag to true
//                     error_counter++;      // Increase the counter
//                 }
//             }
//         }

//         ros::spinOnce();
//         loop_rate.sleep();
//         // usleep(1000);
//     }
// }
    while (nh.ok())
    {
        srv_SendMotion.request.request_id = generateUniqueRequestID(); // generate a unique request ID

        if (client_SendMotion.call(srv_SendMotion))
        {
            if (srv_SendMotion.response.success)
            {
                Motion_Info();
                RecieveMotion();
            }
            else if (!error_printed)
            {
                if (error_counter < 3) // Check the counter
                {
                    // ROS_INFO("\n");
                    // ROS_ERROR("Failed to call service");
                    // ROS_INFO("\n");
                    error_printed = true; // Set the flag to true
                    error_counter++;      // Increase the counter
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        // usleep(1000);
    }
}
/////////////////////////////////////////////// About Client Callback ///////////////////////////////////////////////

void Callback::RecieveMotion()
{
    // ROS2에서는 response/req 멤버를 직접 쓰기보다는 멤버에 저장하거나, 
    // 서비스 콜백에서 받아온 response를 구조체로 따로 저장하는 방식을 추천!
    // 여기선 기존 구조를 최대한 유지해서 바꿈.

    // ROS2 로깅 스타일
    // RCLCPP_INFO(this->get_logger(), "SM Request : %s", srv_send_motion_request_->sm_finish ? "true" : "false");

    SelectMotion();
    TATA();
    Move_UD_NeckAngle();
    // Move_RL_NeckAngle();
    // Emergency();
}

void Callback::Move_RL_NeckAngle()
{
    // 예시: 서비스 응답 데이터를 어떻게 저장하느냐에 따라 달라질 수 있음
    // 아래는 srv_send_motion_response_ 멤버에 서비스 콜 결과가 저장된다고 가정
    double res_rl_neck = srv_send_motion_response_->rl_neckangle;
    rl_neckangle = res_rl_neck;
    pick_Ptr->NC_th[2] = rl_neckangle * DEG2RAD;
    // RCLCPP_WARN(this->get_logger(), "RL_NECK : %.3f", rl_neckangle);
}

void Callback::Move_UD_NeckAngle()
{
    double res_ud_neck = srv_send_motion_response_->ud_neckangle;
    ud_neckangle = 90 - res_ud_neck;
    pick_Ptr->NC_th[1] = ud_neckangle * DEG2RAD;
    srv_send_motion_request_->ud_finish = true;
    // RCLCPP_WARN(this->get_logger(), "UD_NECK : %.3f", res_ud_neck);
}

void Callback::TATA()
{
    double res_turn_angle = srv_send_motion_response_->turn_angle;

    if (res_turn_angle != 0)
    {
        turn_angle = res_turn_angle * DEG2RAD;
        trajectoryPtr->Make_turn_trajectory(turn_angle);
        index_angle = 0;
    }
    RCLCPP_WARN(this->get_logger(), "TURN_ANGLE : %.3f", res_turn_angle);
    RCLCPP_INFO(this->get_logger(), "------------------------- TURN_ANGLE ----------------------------");
}

/// 재민이형 긴급정지에 대한 코드 여기다가 넣으면 됨 ///
void Callback::Emergency()
{
    // ROS2에서는 response를 shared_ptr로 보관할 때가 많음.
    // 예시: srv_send_motion_response_가 std::shared_ptr<ahra::srv::SendMotion::Response>
    // 실제로는 response 객체를 네가 어떻게 관리하는지에 따라 달라질 수 있음

    bool res_emergency = srv_send_motion_response_->emergency;
    emergency_ = res_emergency;

    if (!emergency_) {
        on_emergency = false;
        stop_indext = 0;
    } else {
        on_emergency = true;
        stop_indext = 0;
    }

    RCLCPP_ERROR(rclcpp::get_logger("Callback"), "EMERGENCY : %s", emergency_ ? "True" : "False");
    RCLCPP_INFO(rclcpp::get_logger("Callback"), "------------------------- EMERGENCY ----------------------------");
}

void Callback::Check_FSR()
{
    // 왼발 들 때 오른발 착지 확인
    if (check_indext == 9 && R_value == 0) {
        indext -= 1;
        RCLCPP_INFO(rclcpp::get_logger("Callback"), "Check_FSR: Left foot up, right foot NOT on ground! indext--");
    }
    // 오른발 들 때 왼발 착지 확인
    else if (check_indext == 77 && L_value == 0) {
        indext -= 1;
        RCLCPP_INFO(rclcpp::get_logger("Callback"), "Check_FSR: Right foot up, left foot NOT on ground! indext--");
    }
}


void Callback::Motion_Info()
{
    int8_t res_mode = srv_SendMotion.response.select_motion;
    float res_distance = srv_SendMotion.response.distance;
    std::string tmp_motion;

    switch (res_mode)
    {
    case Motion_Index::InitPose:
        tmp_motion = Str_InitPose;
        break;
    case Motion_Index::Forward_2step:
        tmp_motion = Str_Forward_2step;
        break;
    case Motion_Index::Left_2step:
        tmp_motion = Str_Left_2step;
        break;
    case Motion_Index::Step_in_place:
        tmp_motion = Str_Step_in_place;
        break;
    case Motion_Index::Right_2step:
        tmp_motion = Str_Right_2step;
        break;
    case Motion_Index::ForWard_fast4step:
        tmp_motion = Str_ForWard_fast4step;
        break;
    case Motion_Index::Forward_Nstep:
        tmp_motion = Str_Forward_Nstep;
        break;
    case Motion_Index::Shoot:
        tmp_motion = Str_Shoot;
        break;
    case Motion_Index::Forward_Halfstep:
        tmp_motion = Str_Forward_Halfstep;
        break;
    case Motion_Index::Left_Halfstep:
        tmp_motion = Str_Left_Halfstep;
        break;
    case Motion_Index::Right_Halfstep:
        tmp_motion = Str_Right_Halfstep;
        break;
    case Motion_Index::Back_Halfstep:
        tmp_motion = Str_Back_Halfstep;
        break;
    case Motion_Index::Right_1step:
        tmp_motion = Str_Right_1step;
        break;
    case Motion_Index::Left_1step:
        tmp_motion = Str_Left_1step;
        break;
    case Motion_Index::FWD_UP:
        tmp_motion = Str_FWD_UP;
        break;
    case Motion_Index::BWD_UP:
        tmp_motion = Str_BWD_UP;
        break;
    case Motion_Index::Forward_1step:
        tmp_motion = Str_Forward_1step;
        break;
    case Motion_Index::Right_6step:
        tmp_motion = Str_Right_6step;   // 오타수정: Right_6step -> Str_Right_6step
        break;
    case Motion_Index::Left_6step:
        tmp_motion = Str_Left_6step;    // 오타수정: Left_6step -> Str_Left_6step
        break;
    case Motion_Index::Ready_to_throw:
        tmp_motion = Str_Ready_to_throw;
        break;
    case Motion_Index::NONE:
        tmp_motion = Str_NONE;
        break;
    case Motion_Index::Gen2_Walking:
        tmp_motion = Str_Gen2_Walking;
        break;
    case Motion_Index::Huddle_Jump_V2:
        tmp_motion = Str_Huddle_Jump;
        break;
    case Motion_Index::Picking_Ball:
        tmp_motion = Str_Picking_Ball;
        break;
    case Motion_Index::Back_2step:
        tmp_motion = Str_Back_2step;
        break;
    case Motion_Index::FINISH:
        tmp_motion = Str_FINISH;
        break;
    default:
        tmp_motion = "UNKNOWN";
        break;
    }

    // ROS 2 로그로 변경, 클래스가 rclcpp::Node 상속이면 this->get_logger()를, 아니면 rclcpp::get_logger("Callback") 사용
    auto logger = rclcpp::get_logger("Callback");

    if (res_mode == Motion_Index::Forward_Nstep)
    {
        RCLCPP_WARN(logger, "Motion_Index : %s", tmp_motion.c_str());
        RCLCPP_WARN(logger, "Distance : %f", res_distance);
        RCLCPP_INFO(logger, "------------------------- Select Motion ----------------------------");
    }
    else if (res_mode == Motion_Index::InitPose)
    {
        RCLCPP_ERROR(logger, "Motion_Index : %s", tmp_motion.c_str());
        RCLCPP_INFO(logger, "------------------------- Select Motion ----------------------------");
    }
    else
    {
        RCLCPP_WARN(logger, "Motion_Index : %s", tmp_motion.c_str());
        RCLCPP_INFO(logger, "------------------------- Select Motion ----------------------------");
    }
}

void Callback::SelectMotion()
{
    auto res_mode = srv_SendMotion.response.select_motion;
    auto res_distance = srv_SendMotion.response.distance;

    // Helper 람다: trajectory 초기화
    auto reset_trajectory = [&](int col) {
        trajectoryPtr->Ref_RL_x = Eigen::MatrixXd::Zero(1, col);
        trajectoryPtr->Ref_LL_x = Eigen::MatrixXd::Zero(1, col);
        trajectoryPtr->Ref_RL_y = -0.06 * Eigen::MatrixXd::Ones(1, col);
        trajectoryPtr->Ref_LL_y = 0.06 * Eigen::MatrixXd::Ones(1, col);
        trajectoryPtr->Ref_RL_z = Eigen::MatrixXd::Zero(1, col);
        trajectoryPtr->Ref_LL_z = Eigen::MatrixXd::Zero(1, col);
    };
    // Helper 람다: pick 초기화
    auto reset_pick = [&](int col) {
        pick_Ptr->Ref_WT_th = Eigen::MatrixXd::Zero(1, col);
        pick_Ptr->Ref_RA_th = Eigen::MatrixXd::Zero(4, col);
        pick_Ptr->Ref_LA_th = Eigen::MatrixXd::Zero(4, col);
        pick_Ptr->Ref_NC_th = Eigen::MatrixXd::Zero(2, col);
    };

    indext = 0;
    trajectoryPtr->Change_Freq(2);

    switch (res_mode) {
    case Motion_Index::InitPose:
        mode = Motion_Index::InitPose;
        reset_trajectory(30);
        reset_pick(30);
        break;

    case Motion_Index::Forward_2step:
        mode = Motion_Index::Forward_2step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Straight_start(0.05, 0.25, 0.05);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2);
        IK_Ptr->Set_Angle_Compensation(135);
        trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        break;

    case Motion_Index::Forward_1step:
        mode = Motion_Index::Forward_1step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Straight(0.05, 0.15, 0.05);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2);
        IK_Ptr->Set_Angle_Compensation(135);
        trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        break;

    case Motion_Index::Left_2step:
        mode = Motion_Index::Left_2step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Left2();
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 2, 0, 0, 1.5, 3.5, 0, 0);
        IK_Ptr->Set_Angle_Compensation(135);
        break;

    case Motion_Index::Step_in_place:
        mode = Motion_Index::Step_in_place;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Step_in_place(0.02, 0.1, 0.025);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 4, 2, 2, 2, 2, 2, -2);
        IK_Ptr->Set_Angle_Compensation(135);
        break;

    case Motion_Index::Right_2step:
        mode = Motion_Index::Right_2step;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Side_Right2();
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(2, 4, 1, 0, 2, 4, -1, 0);
        IK_Ptr->Set_Angle_Compensation(135);
        break;

    case Motion_Index::Forward_Nstep:
        mode = Motion_Index::Forward_Nstep;
        IK_Ptr->Change_Com_Height(30);
        trajectoryPtr->Go_Straight_start(0.05, 2.2, 0.05);
        IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        IK_Ptr->Change_Angle_Compensation(3, 3, 2, 2, 2, 2, 2, -2);
        IK_Ptr->Set_Angle_Compensation(135);
        trajectoryPtr->Stop_Trajectory_straightwalk(0.05);
        break;

    // (이하 나머지 케이스 동일 패턴, 생략 가능. 코드 너무 길면 부분별로 이어서 작성해줌!)

    default:
        // 필요시 디폴트 동작 추가
        break;
    }
}


#include <rclcpp/rclcpp.hpp>
#include <cmath>

constexpr double DEG2RAD = M_PI / 180.0;

void Callback::WriteAllTheta()
{
    double thro = 0.0;
    check_indext = indext % walktime_n;

    // 회전 타이밍 체크
    if (turn_angle > 0 && check_indext == 0 && indext > 0.5 * walktime_n && indext < trajectoryPtr->Ref_RL_x.cols() - walktime_n * 0.5)
        turn_left = true;

    if (turn_angle < 0 && check_indext == 67 && indext > 0.5 * walktime_n && indext < trajectoryPtr->Ref_RL_x.cols() - walktime_n * 0.5)
        turn_right = true;

    if (turn_right || turn_left)
        srv_SendMotion.request.TA_finish = false;

    if (emergency == 0)
    {
        indext++;

        // 모션별 시뮬레이션/보상 처리
        switch (mode)
        {
        case 0:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            break;

        case Motion_Index::Forward_2step:
        case Motion_Index::Step_in_place:
        case Motion_Index::Forward_Halfstep:
        case Motion_Index::Forward_1step:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            if (mode != Motion_Index::Forward_Halfstep)
            {
                if (turn_left)
                {
                    IK_Ptr->LL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    step = IK_Ptr->LL_th[0];
                    index_angle++;
                    if (index_angle > walktime_n - 1)
                    {
                        turn_angle = 0;
                        turn_left = false;
                        srv_SendMotion.request.TA_finish = true;
                    }
                }
                if (turn_right)
                {
                    IK_Ptr->RL_th[0] = trajectoryPtr->Turn_Trajectory(index_angle);
                    step = IK_Ptr->RL_th[0];
                    index_angle++;
                    if (index_angle > walktime_n - 1)
                    {
                        turn_angle = 0;
                        turn_right = false;
                        srv_SendMotion.request.TA_finish = true;
                    }
                }
            }
            break;

        case Motion_Index::Left_2step:
        case Motion_Index::Left_Halfstep:
        case Motion_Index::Left_6step:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Leftwalk(indext);
            break;

        case Motion_Index::Right_2step:
        case Motion_Index::Right_Halfstep:
        case Motion_Index::Right_6step:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Rightwalk(indext);
            break;

        case Motion_Index::Huddle_Jump:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation_Huddle(indext);
            break;

        case Motion_Index::Huddle_Jump_V2:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->Huddle(trajectoryPtr->Ref_RL_x, indext, RL_th1, LL_th1, RL_th2, LL_th2, HS, SR);
            break;

        case Motion_Index::ForWard_fast4step:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
            break;

        case Motion_Index::Forward_Nstep:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            if (indext == 500)
                srv_SendMotion.request.ST_finish = false;
            break;

        case Motion_Index::START:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
            break;

        case Motion_Index::Ready_to_throw:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->UPBD_SET(pick_Ptr->Ref_WT_th, pick_Ptr->Ref_RA_th, pick_Ptr->Ref_LA_th, pick_Ptr->Ref_NC_th, indext);
            break;

        case Motion_Index::Shoot:
            thro = -10;
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->WT_th[0] = 20 * DEG2RAD;
            pick_Ptr->RA_th[0] = 150 * DEG2RAD;
            pick_Ptr->RA_th[1] = 22 * DEG2RAD;
            pick_Ptr->RA_th[2] = -110 * DEG2RAD;
            pick_Ptr->RA_th[3] = 50 * DEG2RAD;
            break;

        case Motion_Index::FINISH:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->UPBD_SET(pick_Ptr->Ref_WT_th, pick_Ptr->Ref_RA_th, pick_Ptr->Ref_LA_th, pick_Ptr->Ref_NC_th, indext);
            break;

        case Motion_Index::Gen2_Walking:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
            pick_Ptr->Fast_Arm_Swing(indext, trajectoryPtr->Return_Walktime_n(), trajectoryPtr->Return_Sim_n());
            break;

        case Motion_Index::Picking_Ball:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            pick_Ptr->Picking(trajectoryPtr->Ref_RL_x, indext, RL_th2, LL_th2);
            break;

        case Motion_Index::Back_Halfstep:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            break;

        case Motion_Index::Back_2step:
            IK_Ptr->BRP_Simulation(
                trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z,
                trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Angle_Compensation(indext, trajectoryPtr->Ref_RL_x.cols());
            break;

        default:
            break;
        }

        // line to hoop counter
        if ((mode == Motion_Index::ForWard_fast4step || mode == Motion_Index::Forward_2step || mode == Motion_Index::Forward_1step) && indext == 100)
            srv_SendMotion.request.walkcount++;

        if (indext > 0)
            RCLCPP_INFO(rclcpp::get_logger("Callback"), "indext: %d", indext);

        if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
        {
            indext--;
            srv_SendMotion.request.SM_finish = true;
        }
        else
        {
            srv_SendMotion.request.SM_finish = false;
        }
    }
}


namespace {
constexpr double DEG2RAD = M_PI / 180.0; // 또는 0.017453292519943
}

void Callback::WriteAllTheta()
{
    // 하체(1~12)
    All_Theta[0] = -IK_Ptr->RL_th[0];
    All_Theta[1] = IK_Ptr->RL_th[1] - RL_th1 * DEG2RAD - 3 * DEG2RAD;
    All_Theta[2] = IK_Ptr->RL_th[2] - RL_th2 * DEG2RAD - 17 * DEG2RAD;
    All_Theta[3] = -IK_Ptr->RL_th[3] + 40 * DEG2RAD;
    All_Theta[4] = -IK_Ptr->RL_th[4] + 24.5 * DEG2RAD;
    All_Theta[5] = -IK_Ptr->RL_th[5] + SR * DEG2RAD - 2 * DEG2RAD;

    All_Theta[6] = -IK_Ptr->LL_th[0];
    All_Theta[7] = IK_Ptr->LL_th[1] + LL_th1 * DEG2RAD + 2 * DEG2RAD;
    All_Theta[8] = -IK_Ptr->LL_th[2] + LL_th2 * DEG2RAD + 17 * DEG2RAD;
    All_Theta[9] = IK_Ptr->LL_th[3] - 40 * DEG2RAD;
    All_Theta[10] = IK_Ptr->LL_th[4] - HS * DEG2RAD - 21.22 * DEG2RAD;
    All_Theta[11] = -IK_Ptr->LL_th[5] + SR * DEG2RAD - 2 * DEG2RAD;

    // 상체(13~23)
    All_Theta[12] = pick_Ptr->WT_th[0] + step;
    All_Theta[13] = pick_Ptr->LA_th[0] + 90 * DEG2RAD;
    All_Theta[14] = pick_Ptr->RA_th[0] - 90 * DEG2RAD;
    All_Theta[15] = pick_Ptr->LA_th[1] - 60 * DEG2RAD;
    All_Theta[16] = pick_Ptr->RA_th[1] + 60 * DEG2RAD;
    All_Theta[17] = pick_Ptr->LA_th[2] - 90 * DEG2RAD;
    All_Theta[18] = pick_Ptr->RA_th[2] + 90 * DEG2RAD;
    All_Theta[19] = pick_Ptr->LA_th[3] - 45 * DEG2RAD;
    All_Theta[20] = pick_Ptr->RA_th[3] + 45 * DEG2RAD;
    All_Theta[21] = pick_Ptr->NC_th[0];      // Neck RL
    All_Theta[22] = pick_Ptr->NC_th[1] + 6 * DEG2RAD; // Neck UD

    // 필요시 출력
    // RCLCPP_INFO(this->get_logger(), "All_Theta[14]: %f", All_Theta[14]);
}

void Callback::SetCallback(int size)
{
    // 기본값 10으로, 인자를 안 주면 10
    if (size <= 0) size = 10;
    trajectoryPtr->Go_Straight(0.05, 0.5, 0.05);
    trajectoryPtr->Ref_RL_x = Eigen::MatrixXd::Zero(1, size);
    trajectoryPtr->Ref_LL_x = Eigen::MatrixXd::Zero(1, size);
    trajectoryPtr->Ref_RL_y = -0.06 * Eigen::MatrixXd::Ones(1, size);
    trajectoryPtr->Ref_LL_y = 0.06 * Eigen::MatrixXd::Ones(1, size);
    trajectoryPtr->Ref_RL_z = Eigen::MatrixXd::Zero(1, size);
    trajectoryPtr->Ref_LL_z = Eigen::MatrixXd::Zero(1, size);
}
