/**
 * @file ReiwaMD.hpp
 * @brief 令和MD
 * @details 過去ライブラリの移植
 * @author Horie
 * @date 2020/9
 * @attention そのまま移植しているのでコーディングルールを逸脱しています
 */
#pragma once

#include "../CANWrapper.hpp"
#include "Motor.hpp"
#include <memory>

namespace nut{
/**
 * @brief 令和MDクラス
 * @attention そのまま移植しているのでコーディングルールの逸脱、未定義動作が含まれます
 */
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
	const std::shared_ptr<CANWrapper> _can;
	const uint16_t _id;
	std::array<uint8_t, 8> _rx_data;
	Motor_DataType_t _control_type = SetModeDisable;

	static constexpr uint32_t  CAN_MOTOR_STDID_MASK = 0x0400U;
	static constexpr uint32_t  CAN_MOTOR_STDID_FILTER = 0x0400U;
	typedef union{//union�ɂ��L���X�g�y�уr�b�g�t�B�[���h�̔z�u��C�̎d�l�Ŗ���`�Ȃ̂ł�߂悤�ˁI
		struct{
			Motor_DataType_t dataType	: 5;
			unsigned channel			: 5;//5+5���H�H�H�H�H�H�H�H�H�H
			unsigned forMotor 			: 1;
		};
		uint16_t StdID;
	}Motor_StdID_t;


protected:
	/**
	 * @brief CAN送信関数
	 * @tparam T 送信データ型
	 * @param[in] type 送信データタイプ
	 * @param[in] data 送信データ型
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



	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		if(_move_type == MoveType::stop)return;
		switch(_move_type){
		case MoveType::duty:
			SendData<float>(VoltageRef, 0.240f * _target_duty);
			break;
		case MoveType::radps:
			SendData<float>(SpeedRef, _target_radps);
			break;
		case MoveType::rad:
		case MoveType::stop:
		default:
			return;
		}

	}



public:
	/**
	 * @brief コンストラク
	 * @param[in] period 周期
	 * @param[in] can canのヘルパインスタンス
	 * @param[in] id 5bitのモータid
	 */
	ReiwaMD(uint32_t period, std::shared_ptr<CANWrapper> can, uint16_t id)
		: Motor(period), _can(can), _id(id & 0x1F){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~ReiwaMD(){
		Stop();
	}


	/**
	 * @brief 初期化関数
	 * @details ダミー関数です
	 */
	virtual void Init()override{
	}


	/**
	 * @brief 制御スタート
	 * @details これを呼び出した後にSendControlMode()してください
	 */
	virtual bool Start() override{
		ResetTarget();
		_target_duty = 0.0f;
		_move_type = MoveType::duty;
		_scheduler.Set();
		return true;
	}
	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() override{
		_move_type = MoveType::stop;
		SendControlMode(SetModeDisable);
		_scheduler.Erase();
		ResetParam();
	}



	/**
	 * @brief Duty制御
	 * @param[in] duty 百分率
	 * @return Duty制御可能かどうか
	 */
	virtual bool SetDuty(float duty) override{
		if(std::fabs(duty) > 100.0f)return false;
		if(_control_type != SetModeVoltage)return false;
		_target_duty = duty;
		_move_type = MoveType::duty;
		return true;
	}
	/**
	 * @brief 速度制御
	 * @param[in] rpm RPM
	 * @return 速度制御可能かどうか
	 */
	virtual bool SetRadps(float radps) override{
		if(_control_type != SetModeSpeed)return false;
		_target_radps = radps;
		_move_type = MoveType::radps;
		return true;
	}
	/**
	 * @brief 角度制御
	 * @param[in] rad 角度[rad]
	 * @param[in] top_rpm 最大速度[rpm]
	 * @return 角度制御可能かどうか
	 */
	virtual bool SetRad(float rad, float top_radps)override{
		if(_control_type != SetModePosition)return false;
		_target_rad = rad;
		_target_radps = top_radps;
		_move_type = MoveType::rad;
		return true;
	}



	/**
	 * @brief 速度制御ゲインセット
	 * @details ゲインセットできません
	 * @param[in] kp Pゲイン
	 * @param[in] ki Iゲイン
	 * @param[in] kd Dゲイン
	 * @return false
	 */
	virtual bool SetRadpsPID(float kp, float ki, float kd, float op_limit = infinityf(), float i_limit = infinityf()) override{
		return false;
	}
	/**
	 * @brief 角度制御ゲインセット
	 * @details ゲインセットできません
	 * @param[in] kp Pゲイン
	 * @param[in] ki Iゲイン
	 * @param[in] kd Dゲイン
	 * @return false
	 */
	virtual bool SetRadPID(float kp, float ki, float kd, float op_limit = infinityf(), float i_limit = infinityf()) override{
		return false;
	}
	/**
	 * @brief 角度原点リセット
	 * @details 未実装です
	 * @return false
	 */
	virtual bool ResetRadOrigin(float rad) override{

		return false;
	}



	/**
	 * @brief 制御モード変更
	 * @details これを呼び出す前にStart()してください
	 * @param[in] mode 制御モード
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
				_move_type = MoveType::radps;
				break;
			default:
				_move_type = MoveType::stop;
				break;
			}

			_can->Transmit(txStdID.StdID);
		}

	}


	/**
	 * @brief CAN受信関数
	 * @details HAL_CAN_RxFifo0MsgPendingCallback()またはHAL_CAN_RxFifo1MsgPendingCallback()内で呼び出してください
	 * @param[in] hcan canハンドル
	 * @param[in] RxHeader 受信ヘッダ
	 * @param[in] data 受信データ
	 * @return 受信パケットがこのモータに該当するかどうか
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
