/*
 * motor基底クラス
 * 単位系要相談
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include <array>

namespace nut{
class Motor{
protected:
	enum class MoveType : uint8_t{
		stop = 0U,
		duty,
		rpm,
		rad
	};
	MoveType _move_type = MoveType::stop;

	TimeScheduler<void> _scheduler;

	//セットパラメータ
	std::array<float, 3> _rpm_pid{0.0f};
	std::array<float, 3> _rad_pid{0.0f};
	float _target_duty = 0.0f;//%単位
	float _target_rpm = 0.0f;
	float _target_rad = 0.0f;

	//ゲットパラメータ
	float _now_rpm = 0.0f;
	float _now_rad = 0.0f;


	/*
	 * 内部パラメータリセット
	 */
	virtual void ResetParam(){
		_target_duty = 0.0f;//%単位
		_target_rpm = 0.0f;
		_target_rad = 0.0f;

		_now_rpm = 0.0f;
		_now_rad = 0.0f;
	}


	/*
	 * 周期制御関数
	 */
	virtual void ScheduleTask() = 0;
public:
	Motor(uint32_t period) : _scheduler([this]{ScheduleTask();}, period){}
	virtual ~Motor(){}

	/*
	 * 初期化
	 */
	virtual void Init() = 0;


	/*
	 * 制御開始
	 */
	virtual bool Start() = 0;

	/*
	 * 制御停止
	 */
	virtual void Stop() = 0;



	/*
	 * セッター
	 */
	virtual bool SetDuty(float duty) {
		if(std::fabs(duty) > 100.0f)return false;
		_target_duty = duty;
		_move_type = MoveType::duty;
		return true;
	}
	virtual bool SetRPM(float rpm) {
		_target_rpm = rpm;
		_move_type = MoveType::rpm;
		return true;
	}
	virtual bool SetRad(float rad, float top_rpm){
		_target_rad = rad;
		_target_rpm = top_rpm;
		_move_type = MoveType::rad;
		return true;
	}
	virtual bool SetRPMPID(float kp, float ki, float kd) {
		_rpm_pid = {kp, ki, kd};
		return true;
	}

	virtual bool SetRPMPID(const std::array<float, 3>& param){
		_rpm_pid = param;
		return true;
	}
	virtual bool SetRadPID(float kp, float ki, float kd) {
		_rad_pid = {kp, ki, kd};
		return true;
	}

	virtual bool SetRadPID(const std::array<float, 3>& param){
		_rad_pid = param;
		return true;
	}
	virtual bool ResetRadOrigin(float rad) = 0;

	/*
	 * ゲッター
	 */
	virtual float GetDuty() const{
		return _target_duty;
	}
	virtual float GetRPM() const{
		return _now_rpm;
	}
	virtual float GetRad()const{
		return _now_rad;
	}
	virtual const std::array<float, 3>& GetRPMPID() const{
		return _rpm_pid;
	}
	virtual const std::array<float, 3>& GetRadPID() const{
		return _rad_pid;
	}
};
}
