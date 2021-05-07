/**
 * @file AllLib.hpp
 * @brief ライブラリの全ファイルをインクルードする<br>
 * デバッグ・テスト用
 * @author Horie
 * @date 2020/9
 * @details 使用時はコンパイル後ファイルサイズに気をつけるように
 */
#pragma once


/*global*/
#include "CANWrapper.hpp"
#include "Global.hpp"

/*General library*/
#include "TimeScheduler.hpp"
#include "Coordinate.hpp"
#include "Odmetry.hpp"

/*STM32 Peripheral*/
#include "Flash.hpp"
#include "HALCallbacks/HALCallbacks.hpp"

/*Chassis*/
//#include "Chassis/Chassis.hpp"
#include "Chassis/OmniChassis.hpp"
#include "Chassis/SteerChassis.hpp"
#include "Chassis/SteerChassisSp.hpp"

/*Controller*/
#include "Controller/DualShock.hpp"

/*ControlSystem*/
//#include "ControlSystem/Controller.hpp"
//#include "ControlSystem/PID/PIDBase.hpp"
#include "ControlSystem/PID/PosPID.hpp"
#include "ControlSystem/PID/VecPID.hpp"

/*Motor*/
//#include "Motor/Motor.hpp"
#include "Motor/DirectDutyMotor.hpp"
#include "Motor/CurrentControlMotor.hpp"
#include "Motor/ReiwaMD.hpp"
#include "Motor/MD2021.hpp"
//#include "Motor/SteerDriver.hpp"
#include "Motor/MD2021Steer.hpp"
#include "Motor/DriveWheel.hpp"

/*PowerSupply*/
#include "PowerSupply/HiguchiBoard.hpp"
#include "PowerSupply/PowerSupply2021.hpp"

/*Sensor*/
//#include "Sensor/Encoder/Encoder.hpp"
#include "Sensor/Encoder/IncEncoder.hpp"
#include "Sensor/Encoder/AbsEncoder.hpp"
#include "Sensor/Encoder/EncoderWheel.hpp"
//#include "Sensor/IMU/IMU.hpp"
#include "Sensor/IMU/R13x0.hpp"
#include "Sensor/ToFBoard.hpp"

/*Unit*/
#include "Unit/UnitCore.hpp"//All unit in

/*Etc*/
#include "Etc/Buzzer.hpp"
#include "Etc/SolenoidDriver.hpp"


/* 2021 */
#include "CANBusProtocol.hpp"
