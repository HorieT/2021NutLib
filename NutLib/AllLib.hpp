/**
 * @file AllLib.hpp
 * @brief ライブラリの全ファイルをインクルードする<br>
 * 主にデバッグ用
 * @author Horie
 * @date 2020/9
 * @details 使用時はコンパイル後ファイルサイズに気をつけるように
 */
#pragma once


/*global*/
#include "NutLib/CANWrapper.hpp"
#include "Global.hpp"

/*General library*/
#include "TimeScheduler.hpp"
#include "PID.hpp"
#include "Coordinate.hpp"
#include "Odmetry.hpp"

/*STM32 Peripheral*/
#include "Flash.hpp"

/*Chassis*/
#include "Chassis/Chassis.hpp"
#include "Chassis/OmniChassis.hpp"

/*Controller*/
#include "Controller/DualShock.hpp"

/*Motor*/
#include "Motor/Motor.hpp"
#include "Motor/DirectDutyMotor.hpp"
#include "Motor/ReiwaMD.hpp"

/*PowerSupply*/
#include "PowerSupply/HiguchiBoard.hpp"

/*Sensor*/
#include "Sensor/Encoder/Encoder.hpp"
#include "Sensor/Encoder/EncoderWheel.hpp"
#include "Sensor/Encoder/IncEncoder.hpp"
#include "Sensor/Encoder/AbsEncoder.hpp"
#include "Sensor/IMU/IMU.hpp"
#include "Sensor/IMU/R13x0.hpp"

/*Unit*/
#include "Unit/UnitCore.hpp"//All unit in

/*Etc*/
#include "Etc/Buzzer.hpp"
