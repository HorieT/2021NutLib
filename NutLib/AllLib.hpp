/*
 * 全ライブラリを入れる
 * 基本デバッグ用
 *
 */
#pragma once


/*global*/
#include "Global.hpp"

/*General library*/
#include "TimeScheduler.hpp"
#include "PID.hpp"
#include "Coordinate.hpp"
#include "Odmetry.hpp"

/*STM32 Peripheral*/
#include "CAN.hpp"
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

/*Etc*/
#include "Etc/Buzzer.hpp"
