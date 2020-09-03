/*
 * 令和MDのクラス
 * ほぼ過去コードの移植なので大部分が規約違反コード
 */
#pragma once

#include "Motor.hpp"
#include "../CAN.hpp"
#include <memory>

namespace nut{
class ReiwaMD : public Motor{
public:
	typedef enum{
		VoltageRef = 0x00,
		CurrentRef,
		SpeedRef,
		PositionRef,
		ZeroReturn,
		SetModeDisable,
		SetModeVoltage,
		SetModeSpeed,
		SetModePosition,
		ChangeEncoderInc,
		ChangeEncoderAbs,
		ResetIncEnc,
		ResetAbsEnc,
		SetModeCurrent,

		Motor_ReadStatus = 0x10,
	}Motor_DataType_t;



private:
	const std::shared_ptr<CAN> _can;
	const uint16_t _id;
	std::array<uint8_t, 8> _rx_data;
	Motor_DataType_t _control_type = SetModeDisable;

	static constexpr uint32_t  CAN_MOTOR_STDID_MASK = 0x0400U;
	static constexpr uint32_t  CAN_MOTOR_STDID_FILTER = 0x0400U;
	typedef union{//unionによるキャスト及びビットフィールドの配置はCの仕様で未定義なのでやめようね！
		struct{
			Motor_DataType_t dataType	: 5;
			unsigned channel			: 5;//5+5←？？？？？？？？？？
			unsigned forMotor 			: 1;
		};
		uint16_t StdID;
	}Motor_StdID_t;


protected:
	/*
	 * メッセージ送信関数
	 */
	template<class T>
	void SendData(Motor_DataType_t type, T data) {
		static_assert(sizeof(T) <= 8,"Data size is over.");
		std::array<uint8_t, sizeof(T)> buff = {0};
		Motor_StdID_t txStdID;

		txStdID.forMotor = 1;
		txStdID.channel = _id;
		txStdID.dataType = type;

		std::memcpy(buff.data(), &data, sizeof(T));
		_can->Transmit<sizeof(T)>(txStdID.StdID, buff);
	}



	/*
	 * 周期制御関数
	 */
	virtual void ScheduleTask() override{
		if(_move_type == MoveType::stop)return;
		switch(_move_type){
		case MoveType::duty:
			SendData<float>(VoltageRef, 0.240f * _target_duty);
			break;
		case MoveType::rpm:
			SendData<float>(SpeedRef, _target_rpm * 120.0f * M_PI);
			break;
		case MoveType::rad:
		case MoveType::stop:
		default:
			return;
		}

	}



public:
	ReiwaMD(uint32_t period, std::shared_ptr<CAN> can, uint16_t id)
		: Motor(period), _can(can), _id(id & 0x1F){

	}
	virtual ~ReiwaMD(){
		Stop();
	}


	/*
	 * 初期化
	 */
	virtual void Init()override{
		//ダミー関数
	}


	/*
	 * 制御開始
	 */
	virtual bool Start() override{
		ResetTarget();
		_target_duty = 0.0f;
		_move_type = MoveType::duty;
		_scheduler.Set();
		return true;
	}

	/*
	 * 制御停止
	 */
	virtual void Stop() override{
		_move_type = MoveType::stop;
		SendControlMode(SetModeDisable);
		_scheduler.Erase();
		ResetParam();
	}



	/*
	 * セッター
	 */
	virtual bool SetDuty(float duty) override{
		if(std::fabs(duty) > 100.0f)return false;
		if(_control_type != SetModeVoltage)return false;
		_target_duty = duty;
		_move_type = MoveType::duty;
		return true;
	}
	virtual bool SetRPM(float rpm) override{
		if(_control_type != SetModeSpeed)return false;
		_target_rpm = rpm;
		_move_type = MoveType::rpm;
		return true;
	}
	virtual bool SetRad(float rad, float top_rpm)override{
		if(_control_type != SetModePosition)return false;
		_target_rad = rad;
		_target_rpm = top_rpm;
		_move_type = MoveType::rad;
		return true;
	}
	virtual bool SetRPMPID(float kp, float ki, float kd) override{
		return false;
	}
	virtual bool SetRadPID(float kp, float ki, float kd) override{
		return false;
	}
	virtual bool ResetRadOrigin(float rad) override{
		/*まだ*/
		return false;
	}


	/*
	 * 制御モード変更
	 */
	void SendControlMode(Motor_DataType_t mode) {
		if(mode >= SetModeDisable && mode <= SetModePosition){
			Motor_StdID_t txStdID;

			txStdID.forMotor = 1;
			txStdID.channel = _id;
			txStdID.dataType = mode;

			_control_type = mode;

			switch(_control_type){
			case Motor_DataType_t::SetModeVoltage:
				_move_type = MoveType::duty;
				break;
			case Motor_DataType_t::SetModeSpeed:
				_move_type = MoveType::rpm;
				break;
			default:
				_move_type = MoveType::stop;
				break;
			}

			_can->Transmit(txStdID.StdID);
		}

	}


	/*
	 * CAN受信関数
	 */
	bool ReadCanData(CAN_HandleTypeDef* hcan, const CAN_RxHeaderTypeDef& RxHeader, const std::array<uint8_t, 8> data){
		if(hcan == _can->GetHandle()){
			if(RxHeader.RTR == CAN_RTR_DATA &&
							((RxHeader.StdId & CAN_MOTOR_STDID_MASK) == (CAN_MOTOR_STDID_FILTER & CAN_MOTOR_STDID_MASK))){
				_rx_data = data;
				return true;
			}
		}

		return false;
	}
};
}
