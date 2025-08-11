#include "dynamixel.hpp"
#include <rclcpp/rclcpp.hpp>

// Constructor
Dxl::Dxl()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    port_handler_ = dynamixel::PortHandler::getPortHandler(kDeviceName);
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(kProtocolVersion);

    if (!port_handler_->openPort())
        RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to open the port!");
    else 
        RCLCPP_INFO(rclcpp::get_logger("dynamixel"), "Succeeded to open the port!");

    if (!port_handler_->setBaudRate(kBaudrate))
        RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to set the baudrate!");
    else 
        RCLCPP_INFO(rclcpp::get_logger("dynamixel"), "Succeeded to set the baudrate!");

    int16_t current_mode = SetPresentMode(mode_);

    if (current_mode == static_cast<int16_t>(DynamixelOperatingMode::kCurrentControlMode))
    {
        for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
        {
            dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, dxl_id_[i],
                static_cast<uint16_t>(DynamixelStandardRegisterTable::kOperatingMode),
                static_cast<uint8_t>(DynamixelOperatingMode::kCurrentControlMode), &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to set torque control mode for Dynamixel ID: %d", dxl_id_[i]);
            else
                RCLCPP_INFO(rclcpp::get_logger("dynamixel"), "Set torque control mode for Dynamixel ID: %d", dxl_id_[i]);
        }
    }
    else if (current_mode == static_cast<int16_t>(DynamixelOperatingMode::kPositionControlMode))
    {
        for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
        {
            dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, dxl_id_[i],
                static_cast<uint16_t>(DynamixelStandardRegisterTable::kOperatingMode),
                static_cast<uint8_t>(DynamixelOperatingMode::kPositionControlMode), &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
                RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to set position control mode for Dynamixel ID: %d", dxl_id_[i]);
            else
                RCLCPP_INFO(rclcpp::get_logger("dynamixel"), "Set position control mode for Dynamixel ID: %d", dxl_id_[i]);
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Invalid mode set. Neither Current nor Position Control mode was set.");
    }

    // Torque Enable
    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, dxl_id_[i],
            static_cast<uint16_t>(DynamixelStandardRegisterTable::kTorqueEnable), 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to enable torque for Dynamixel ID %d", dxl_id_[i]);
        else
            RCLCPP_INFO(rclcpp::get_logger("dynamixel"), "Torque enabled for Dynamixel ID: %d", dxl_id_[i]);
    }

    // LED ON
    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, dxl_id_[i],
            static_cast<uint16_t>(DynamixelStandardRegisterTable::kLED), 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to enable LED for Dynamixel ID %d", dxl_id_[i]);
        else
            RCLCPP_INFO(rclcpp::get_logger("dynamixel"), "LED enabled for Dynamixel ID: %d", dxl_id_[i]);
    }

    // PID Gain 예시 설정
    Eigen::VectorXd pid_gain(3);
    pid_gain << 850, 0, 0; // P, I, D
    SetPIDGain(pid_gain);
}

// Destructor
Dxl::~Dxl()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Torque Disable
    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        dxl_comm_result = packet_handler_->write1ByteTxRx(port_handler_, dxl_id_[i],
            static_cast<uint16_t>(DynamixelStandardRegisterTable::kTorqueEnable),
            static_cast<uint8_t>(DynamixelOperatingMode::kCurrentControlMode), &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to disable torque for Dynamixel ID %d", dxl_id_[i]);
        else
            RCLCPP_INFO(rclcpp::get_logger("dynamixel"), "Torque disabled for Dynamixel ID: %d", dxl_id_[i]);
    }
    // LED Disable
    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        packet_handler_->write1ByteTxRx(port_handler_, dxl_id_[i],
            static_cast<uint16_t>(DynamixelStandardRegisterTable::kLED), 0, &dxl_error);
    }
    port_handler_->closePort();
}

// 각도 읽기 (raw -> rad)
void Dxl::syncReadTheta()
{
    dynamixel::GroupSyncRead groupSyncRead(port_handler_, packet_handler_,
        static_cast<uint16_t>(DynamixelStandardRegisterTable::kPresentPosition), 4);
    for(uint8_t i = 0; i < kNumberOfDynamixels; i++) groupSyncRead.addParam(dxl_id_[i]);
    groupSyncRead.txRxPacket();

    for(uint8_t i = 0; i < kNumberOfDynamixels; i++)
        position_[i] = groupSyncRead.getData(dxl_id_[i], static_cast<uint16_t>(DynamixelStandardRegisterTable::kPresentPosition), 4);
    groupSyncRead.clearParam();

    for(uint8_t i = 0; i < kNumberOfDynamixels; i++)
        th_[i] = convertValue2Radian(position_[i]) - kPi - zero_manual_offset_[i];
}

Eigen::VectorXd Dxl::GetThetaAct()
{
    syncReadTheta();
    return th_;
}

// 속도 읽기 (raw)
void Dxl::syncReadThetaDot()
{
    dynamixel::GroupSyncRead groupSyncRead(port_handler_, packet_handler_,
        static_cast<uint16_t>(DynamixelStandardRegisterTable::kPresentVelocity), 4);
    for (uint8_t i = 0; i < kNumberOfDynamixels; i++) groupSyncRead.addParam(dxl_id_[i]);
    groupSyncRead.txRxPacket();
    for(uint8_t i = 0; i < kNumberOfDynamixels; i++)
        velocity_[i] = groupSyncRead.getData(dxl_id_[i], static_cast<uint16_t>(DynamixelStandardRegisterTable::kPresentVelocity), 4);
    groupSyncRead.clearParam();
}

// 각속도 [rad/s]
Eigen::VectorXd Dxl::GetThetaDot()
{
    Eigen::VectorXd vel_(kNumberOfDynamixels);
    for(uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        if(velocity_[i] > 4294900000)
            vel_[i] = (velocity_[i] - 4294967295) * 0.003816667;
        else
            vel_[i] = velocity_[i] * 0.003816667;
    }
    return vel_;
}

// 현재 모드 getter
int16_t Dxl::GetPresentMode()
{
    return this->mode_;
}

// 각속도 추정
void Dxl::CalculateEstimatedThetaDot(int dt_us)
{
    th_dot_est_ = (th_last_ - th_) / (-dt_us * 0.00001);
    th_last_ = th_;
}

Eigen::VectorXd Dxl::GetThetaDotEstimated()
{
    return th_dot_est_;
}

// 전류값 [mA]
void Dxl::SyncReadCurrent()
{
    dynamixel::GroupSyncRead groupSyncRead(port_handler_, packet_handler_,
        static_cast<uint16_t>(DynamixelStandardRegisterTable::kPresentCurrent), 2);
    for(uint8_t i = 0; i < kNumberOfDynamixels; i++) groupSyncRead.addParam(dxl_id_[i]);
    groupSyncRead.txRxPacket();
    for(uint8_t i = 0; i < kNumberOfDynamixels; i++)
        current_[i] = groupSyncRead.getData(dxl_id_[i], static_cast<uint16_t>(DynamixelStandardRegisterTable::kPresentCurrent), 2);
    groupSyncRead.clearParam();
    for(uint8_t i = 0; i < kNumberOfDynamixels; i++)
        cur_[i] = convertValue2Current(current_[i]);
}

Eigen::VectorXd Dxl::GetCurrent()
{
    SyncReadCurrent();
    return cur_;
}

// Goal Position sync write
void Dxl::syncWriteTheta()
{
    dynamixel::GroupSyncWrite gSyncWriteTh(port_handler_, packet_handler_,
        static_cast<uint16_t>(DynamixelStandardRegisterTable::kGoalPosition), 4);

    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        ref_th_value_ = ref_th_ * kRadToValue;
        uint8_t parameter[4];
        getParam(static_cast<int32_t>(ref_th_value_[i]), parameter);
        gSyncWriteTh.addParam(dxl_id_[i], parameter);
    }
    gSyncWriteTh.txPacket();
    gSyncWriteTh.clearParam();
}

// 세타목표값 설정
void Dxl::SetThetaRef(const Eigen::VectorXd& theta)
{
    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        ref_th_[i] = theta[i] + kPi;
    }
}

// 토크 sync write
void Dxl::syncWriteTorque()
{
    dynamixel::GroupSyncWrite groupSyncWriter(port_handler_, packet_handler_,
        static_cast<uint16_t>(DynamixelStandardRegisterTable::kGoalCurrent), 2);

    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        ref_torque_value_[i] = torqueToValue(ref_torque_[i], i);
        if(ref_torque_value_[i] > 1000) ref_torque_value_[i] = 1000;
        else if(ref_torque_value_[i] < -1000) ref_torque_value_[i] = -1000;
    }
    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        uint8_t parameter[4];
        getParam(ref_torque_value_[i], parameter);
        groupSyncWriter.addParam(dxl_id_[i], parameter);
    }
    groupSyncWriter.txPacket();
    groupSyncWriter.clearParam();
}

// 토크 목표값 설정
void Dxl::SetTorqueRef(const Eigen::VectorXd& a_torque)
{
    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
        ref_torque_[i] = a_torque[i];
}

// PID gain setter
void Dxl::SetPIDGain(const Eigen::VectorXd& PID_Gain)
{
    uint8_t dxl_error = 0;

    if (PID_Gain.size() != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "PID_Gain should have exactly 3 elements: P, I, D");
        return;
    }

    uint16_t P_gain = static_cast<uint16_t>(PID_Gain(0));
    uint16_t I_gain = static_cast<uint16_t>(PID_Gain(1));
    uint16_t D_gain = static_cast<uint16_t>(PID_Gain(2));

    for (uint8_t i = 0; i < kNumberOfDynamixels; i++)
    {
        // P
        int result = packet_handler_->write2ByteTxRx(port_handler_, dxl_id_[i],
            static_cast<uint16_t>(DynamixelStandardRegisterTable::kPositionPGain), P_gain, &dxl_error);
        if (result != COMM_SUCCESS)
            RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to set P gain for DXL ID: %d", dxl_id_[i]);
        // I
        result = packet_handler_->write2ByteTxRx(port_handler_, dxl_id_[i],
            static_cast<uint16_t>(DynamixelStandardRegisterTable::kPositionIGain), I_gain, &dxl_error);
        if (result != COMM_SUCCESS)
            RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to set I gain for DXL ID: %d", dxl_id_[i]);
        // D
        result = packet_handler_->write2ByteTxRx(port_handler_, dxl_id_[i],
            static_cast<uint16_t>(DynamixelStandardRegisterTable::kPositionDGain), D_gain, &dxl_error);
        if (result != COMM_SUCCESS)
            RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Failed to set D gain for DXL ID: %d", dxl_id_[i]);
    }
}

// 현재 모드 설정
int16_t Dxl::SetPresentMode(int16_t mode)
{
    if (mode == static_cast<int16_t>(DynamixelOperatingMode::kCurrentControlMode))
    {
        mode_ = static_cast<int16_t>(DynamixelOperatingMode::kCurrentControlMode);
        return mode_;
    }
    else if (mode == static_cast<int16_t>(DynamixelOperatingMode::kPositionControlMode))
    {
        mode_ = static_cast<int16_t>(DynamixelOperatingMode::kPositionControlMode);
        return mode_;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("dynamixel"), "Invalid mode requested. Defaulting to Current Control Mode.");
        mode_ = static_cast<int16_t>(DynamixelOperatingMode::kCurrentControlMode);
        return mode_;
    }
}

// 토크 → raw
int32_t Dxl::torqueToValue(double torque, uint8_t index)
{
    int32_t value_ = static_cast<int32_t>(torque * torque2value_[index]);
    return value_;
}

// Raw data → Radian
float Dxl::convertValue2Radian(int32_t value)
{
    float radian = static_cast<float>(value) / kRadToValue;
    return radian;
}

// Raw data → Current (mA)
float Dxl::convertValue2Current(int32_t value)
{
    float current_ = value * 3.36f;
    return current_;
}

void Dxl::Loop(bool RxTh, bool RxThDot, bool TxTorque)
{
    if(RxTh) syncReadTheta();
    if(RxThDot) syncReadThetaDot();
    // if(TxTorque) syncWriteTorque();
}

void Dxl::initActuatorValues()
{
    for (int i = 0; i < kNumberOfDynamixels; i++)
        torque2value_[i] = kTorqueToValueMx106;
    for (int i = 0; i < kNumberOfDynamixels; i++)
        zero_manual_offset_[i] = 0;
}

// 기타 (IMU, FSR 등 ROS1 의존 코드 미사용부 주석 처리)

void Dxl::getParam(int32_t data, uint8_t *param)
{
    param[0] = DXL_LOBYTE(DXL_LOWORD(data));
    param[1] = DXL_HIBYTE(DXL_LOWORD(data));
    param[2] = DXL_LOBYTE(DXL_HIWORD(data));
    param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}
