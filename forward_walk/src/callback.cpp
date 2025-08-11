#include "callback.hpp"

bool flgflg = 0;
FILE *Trajectory_all;

Callback::Callback(Trajectory *trajectoryPtr, IK_Function *IK_Ptr, Dxl *dxlPtr, Pick *Pick_Ptr)
    : Node("callback_node_call"),  // Node 생성 시 노드 이름을 추가
      trajectoryPtr(trajectoryPtr),
      IK_Ptr(IK_Ptr),
      dxlPtr(dxlPtr),
      Pick_Ptr(Pick_Ptr),
      SPIN_RATE(100)     
{
    // ROS 2에서의 Node 객체 생성
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("callback_node");
    
    // ROS 2에서 boost::thread 대신 std::thread 사용
    std::thread queue_thread(&Callback::callbackThread, this);
    queue_thread.detach();  // 비동기식 실행

    // ROS 2의 subscription 생성
    Start_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("/START", 10, std::bind(&Callback::StartMode, this, std::placeholders::_1));
    
    trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 675);
    trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 675);
    trajectoryPtr->Turn_Trajectory = VectorXd::Zero(135);
    omega_w = sqrt(g / z_c);

    Pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 675);
    Pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 675);
    Pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 675);
    Pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 675);
    emergency = 1;
    indext = 1;
    go = 0;

    RCLCPP_INFO(this->get_logger(), "Emergency value: %d", emergency);
    RCLCPP_INFO(this->get_logger(), "Callback activated");
}


void Callback::StartMode(const std_msgs::msg::Bool::SharedPtr start)
{
    RCLCPP_DEBUG(this->get_logger(), "StartMode called with data: %d", start->data);
    RCLCPP_INFO(this->get_logger(), "StartMode activated?");
    if (start->data)
    {
        indext = 0;
        trajectoryPtr->Ref_RL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_x = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_RL_y = -0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_LL_y = 0.06 * MatrixXd::Ones(1, 30);
        trajectoryPtr->Ref_RL_z = MatrixXd::Zero(1, 30);
        trajectoryPtr->Ref_LL_z = MatrixXd::Zero(1, 30);
        Pick_Ptr->Ref_WT_th = MatrixXd::Zero(1, 30);
        Pick_Ptr->Ref_RA_th = MatrixXd::Zero(4, 30);
        Pick_Ptr->Ref_LA_th = MatrixXd::Zero(4, 30);
        Pick_Ptr->Ref_NC_th = MatrixXd::Zero(2, 30);

        emergency = 0;
        go = 1;

        RCLCPP_INFO(this->get_logger(), "StartMode activated with true data!");
    }
}

void Callback::callbackThread()
{
    // ROS 2의 spin 사용 대신 루프에서 메시지 처리
    rclcpp::Rate loop_rate(SPIN_RATE);
    
    while (rclcpp::ok())
    {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }
}

void Callback::SelectMotion()
{
    if(re == 0)
    {
        re = 1;
        indext = 0;
        indext = 0;
    
        trajectoryPtr->Change_Freq(2);
        
        
        IK_Ptr->Change_Com_Height(30);
        
        //trajectoryPtr->Freq_Change_Straight(0.05, 0.2, 0.05, 1);
        trajectoryPtr->Picking_Motion(300, 150, 0.165);

        // IK_Ptr->Change_Com_Height(30);
        // trajectoryPtr->Freq_Change_Straight(0.05, 0.2, 0.05, 1);
        // IK_Ptr->Get_Step_n(trajectoryPtr->Return_Step_n());
        // IK_Ptr->Change_Angle_Compensation(3, 3, 2, 2, 3, 2, 2, -2);   
        // IK_Ptr->Set_Angle_Compensation(67);
    }
}

void Callback::Write_All_Theta()
{
    if (emergency == 0)
    {
        indext += 1;

        if (go == 1)
        {
            IK_Ptr->BRP_Simulation(trajectoryPtr->Ref_RL_x, trajectoryPtr->Ref_RL_y, trajectoryPtr->Ref_RL_z, trajectoryPtr->Ref_LL_x, trajectoryPtr->Ref_LL_y, trajectoryPtr->Ref_LL_z, indext);
            IK_Ptr->Fast_Angle_Compensation(indext);
        }


        if (indext >= trajectoryPtr->Ref_RL_x.cols() && indext != 0)
        {
            indext -= 1;
            re = 0;
        }

    }

    // All_Theta 계산 및 저장
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
    All_Theta[11] = -IK_Ptr->LL_th[5] + SR * DEG2RAD;

    // upper_body
    All_Theta[12] = Pick_Ptr->WT_th[0] + step + 0 * DEG2RAD;  // waist
    All_Theta[13] = Pick_Ptr->LA_th[0] + 90 * DEG2RAD; // L_arm
    All_Theta[14] = Pick_Ptr->RA_th[0] - 90 * DEG2RAD; // R_arm
    All_Theta[15] = Pick_Ptr->LA_th[1] - 60 * DEG2RAD; // L_arm
    All_Theta[16] = Pick_Ptr->RA_th[1] + 60 * DEG2RAD; // R_arm
    All_Theta[17] = Pick_Ptr->LA_th[2] - 90 * DEG2RAD; // L_elbow
    All_Theta[18] = Pick_Ptr->RA_th[2] + 90 * DEG2RAD; // R_elbow
    All_Theta[19] = Pick_Ptr->LA_th[3] - 0 * DEG2RAD; // L_hand
    All_Theta[20] = Pick_Ptr->RA_th[3] + 0 * DEG2RAD; // R_hand
    All_Theta[21] = Pick_Ptr->NC_th[0] + 0 * DEG2RAD; // neck_RL
    All_Theta[22] = Pick_Ptr->NC_th[1] + 6 * DEG2RAD; // neck_UD

    // 디버깅 정보 출력
    for (int i = 0; i < All_Theta.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "All_Theta[%d] = %f", i, All_Theta[i]*180/3.14);
    }
}

