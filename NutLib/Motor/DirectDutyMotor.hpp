/*
 * duty入力タイプのモーターのクラス
 * エンコーダなしでも使える(ヌルポ入れといて)
 */
#pragma once

#include "Motor.hpp"
#include <memory>

namespace nut{
class DirectDutyMotor : public Motor{
private:
	TIM_HandleTypeDef* const _htim;//PWM
	const uint32_t _channel;//PWM
	GPIO_TypeDef* const _gpio_port;
	const uint16_t _gpio_pin;
	const std::shared_ptr<Encoder> _encoder;



	virtual void ScheduleTask() override{
		if(_move_type == MoveType::stop){
		}
		else{
			float set_duty;
			if(_move_type == MoveType::duty){
				set_duty = _target_duty;
				if(_encoder.get() == NULL){//別途エンコーダ読み出し
					float rad = _encoder->GetRadAndReset() / (2f*static_cast<float>(M_PI));
					_now_rad += rad;
					_now_rpm = rad * 60000f / _scheduler.GetPeriod();
				}
			}
			else{
				if(_encoder.get() == NULL){//エンコーダ無し
					Stop();
					return;
				}
				//エンコーダ読み出し
				float rad = _encoder->GetRadAndReset() / (2f*static_cast<float>(M_PI));
				_now_rad += rad;
				_now_rpm = rad * 60000f / _scheduler.GetPeriod();

				if(_move_type == MoveType::rpm){
					/*まだ*/
				}
				else{
					/*まだ*/
				}
			}

			__HAL_TIM_SetCompare(_htim, _channel, static_cast<uint16_t>((set_duty < 0f ? -set_duty : set_duty) * _htim->Instance->ARR));
		}
	}
public:
	DirectDutyMotor(uint32_t period, TIM_HandleTypeDef* htim, uint32_t channel, GPIO_TypeDef* port, uint16_t pin, const std::shared_ptr<Encoder>& encoder) :
		Motor(period), _htim(htim),_channel(channel), _gpio_port(port), _gpio_pin(pin), _encoder(encoder){}
	virtual ~DirectDutyMotor(){}



	/*
	 * 初期化
	 */
	virtual void Init() {
		_encoder->Init();
	}


	/*
	 * 制御開始
	 */
	virtual bool Start() override{
		_target_rpm = 0;
		_move_type = MoveType::duty;
		__HAL_TIM_SetCompare(_htim, _channel, 0);
		HAL_TIM_PWM_Start(_htim, _channel);
		_scheduler.Set();
		return true;
	}

	/*
	 * 制御停止
	 */
	virtual void Stop() override{
		_move_type = MoveType::stop;
		HAL_TIM_PWM_Stop(_htim, _channel);
		_scheduler.Erase();
		ResetParam();
	}


	/*
	 * セッター
	 * エンコーダの有無に関する変更
	 */
	virtual bool SetRPM(float rpm) override {
		if(_encoder.get() == NULL)return false;
		_target_rpm = rpm;
		_move_type = MoveType::rpm;
		return true;
	}
	virtual bool SetRad(float rad, float top_rpm) override{
		if(_encoder.get() == NULL)return false;
		_target_rad = rad;
		_target_rpm = top_rpm;
		_move_type = MoveType::rad;
		return true;
	}
	virtual bool SetRPMPID(float kp, float ki, float kd) override{
		if(_encoder.get() == NULL)return false;
		_rpm_pid = {kp, ki, kd};
		return true;
	}

	virtual bool SetRPMPID(const std::array<float, 3>& param) override{
		if(_encoder.get() == NULL)return false;
		_rpm_pid = param;
		return true;
	}
	virtual bool SetRadPID(float kp, float ki, float kd) override{
		if(_encoder.get() == NULL)return false;
		_rad_pid = {kp, ki, kd};
		return true;
	}

	virtual bool SetRadPID(const std::array<float, 3>& param) override{
		if(_encoder.get() == NULL)return false;
		_rad_pid = param;
		return true;
	}


	virtual bool ResetRadOrigin(float rad) override{
		if(_move_type == MoveType::rad)return false;
		_now_rad = rad;
		return true;
	}
};
}
