/**
 * @file DirectDutyMotor.hpp
 * @brief Dutyモータ制御
 * @author Horie
 * @date 2020/9
 * @attention まだ一部未実装です
 */
#pragma once

#include "Motor.hpp"
#include "../Sensor/Encoder/Encoder.hpp"
#include <memory>

namespace nut{
/**
 * @brief Duty制御のモータクラス
 */
class DirectDutyMotor : public Motor{
private:
	TIM_HandleTypeDef* const _htim;//!< PWM
	const uint32_t _channel;//!< PWM
	GPIO_TypeDef* const _gpio_port;
	const uint16_t _gpio_pin;
	const std::shared_ptr<Encoder> _encoder;
	float _encoder_ratio = 1.0;


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		if(_move_type == MoveType::stop){
		}
		else{
			float set_duty;
			if(_move_type == MoveType::duty){
				_radps_pid.Reset();
				set_duty = _target_duty;
				//get radian
				if(_encoder.get() != nullptr){
					float rad = _encoder->GetRad();
					_now_radps =
							((rad -_now_rad > M_PI) ?
									rad -_now_rad - M_PI*2.0 :
									((rad -_now_rad < -M_PI) ? rad -_now_rad + M_PI*2.0 : rad -_now_rad)) * 1000.0f / _scheduler.GetPeriod();
					_now_rad = rad;
				}
			}
			else{
				if(_encoder.get() == nullptr){//no encoder
					Stop();
					return;
				}
				//get radian
				float rad = _encoder->GetRad();
				_now_radps = (rad -_now_rad) * 1000.0f / _scheduler.GetPeriod();
				_now_rad = rad;

				if(_move_type == MoveType::radps){
					/*control speed*/
					set_duty = _radps_pid.Calculate(_target_radps - _now_radps, _scheduler.GetPeriod());
				}
				else if(_move_type == MoveType::rad){
					_target_radps = _rad_pid.Calculate(_target_rad - _now_rad,  _scheduler.GetPeriod());
					set_duty = _radps_pid.Calculate(_target_radps - _now_radps, _scheduler.GetPeriod());
				}
				else{
					/* not yet!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
					Stop();
					return;
					/******************************************/
				}
			}

			/* set duty */
			__HAL_TIM_SET_COMPARE(_htim, _channel, static_cast<uint16_t>(std::fabs(set_duty) / 100.0 * _htim->Instance->ARR));
			if(set_duty != 0.0f)HAL_GPIO_WritePin(_gpio_port, _gpio_pin, (set_duty > 0.0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		}
	}

public:
	/**
	 * @brief コンストラク
	 * @param[in] period 周期
	 * @param[in] htim PWM出力タイマ
	 * @param[in] channel PWM出力チャンネル
	 * @param[in] port 回転極性ピンのポート
	 * @param[in] pin 回転極性ピン
	 * @param[in] encoder エンコーダのインスタンス
	 * @details エンコーダを使わない場合はヌルポを入れてください
	 */
	DirectDutyMotor(uint32_t period, TIM_HandleTypeDef* htim, uint32_t channel, GPIO_TypeDef* port, uint16_t pin, const std::shared_ptr<Encoder>& encoder, float encoder_ratio = 1.0) :
		Motor(period), _htim(htim),_channel(channel), _gpio_port(port), _gpio_pin(pin), _encoder(encoder), _encoder_ratio(encoder_ratio){
		_radps_pid.SetLimit(100.0);//duty limit
	}
	virtual ~DirectDutyMotor(){}



	/**
	 * @brief 初期化関数
	 */
	virtual void Init() {
		_encoder->Init();
	}

	bool ChangeEncoder(){
		if(_move_type != MoveType::stop)return false;
	}

	/**
	 * @brief 制御スタート
	 */
	virtual bool Start() override{
		ResetTarget();
		_move_type = MoveType::stop;
		__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		HAL_TIM_PWM_Start(_htim, _channel);
		_scheduler.Set();
		return true;
	}

	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() override{
		_move_type = MoveType::stop;
		HAL_TIM_PWM_Stop(_htim, _channel);
		_scheduler.Erase();
		ResetParam();
	}


	/**
	 * @brief 速度制御
	 * @param[in] rpm RPM
	 * @return 速度制御可能かどうか
	 */
	virtual bool SetRadps(float radps) override {
		if(_encoder.get() == NULL)return false;
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
	virtual bool SetRad(float rad, float top_radps) override{
		if(_encoder.get() == NULL)return false;
		_target_rad = rad;
		_target_radps = top_radps;
		_move_type = MoveType::rad;
		return true;
	}
	/**
	 * @brief 速度制御ゲインセット
	 * @param[in] kp Pゲイン
	 * @param[in] ki Iゲイン
	 * @param[in] kd Dゲイン
	 * @return 速度PID可能かどうか
	 */
	virtual bool SetRadpsPID(float kp, float ki, float kd, float op_limit = infinityf(), float i_limit = infinityf()) override{
		if(_move_type != MoveType::stop)return false;
		if(_encoder.get() == NULL)return false;
		_radps_pid.SetGaine(kp, ki, kd);
		_radps_pid.SetLimit(op_limit);
		_radps_pid.SetLimitI(i_limit);
		return true;
	}
	/**
	 * @brief 角度制御ゲインセット
	 * @param[in] kp Pゲイン
	 * @param[in] ki Iゲイン
	 * @param[in] kd Dゲイン
	 * @return 角度PID可能かどうか
	 */
	virtual bool SetRadPID(float kp, float ki, float kd, float op_limit = infinityf(), float i_limit = infinityf()) override{
		if(_move_type != MoveType::stop)return false;
		if(_encoder.get() == NULL)return false;
		_rad_pid.SetGaine(kp, ki, kd);
		_rad_pid.SetLimit(op_limit);
		_rad_pid.SetLimitI(i_limit);
		return true;
	}



	/**
	 * @brief 角度原点リセット
	 * @return bool 角度原点リセット可能かどうか
	 */
	virtual bool ResetRadOrigin(float rad) override{
		if(_move_type != MoveType::stop)return false;
		if(_encoder.get() == NULL)return false;
		_now_rad = rad;
		return true;
	}


	TIM_HandleTypeDef* GetPWMTimer() const{
		return _htim;
	}
};
}
