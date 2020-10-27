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
	float last_rad = 0.0;


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		//get radian
		if(_encoder.get() != nullptr){
			float rad = _encoder->GetRad();
			float rad_div = (rad - last_rad > M_PI) ?
						rad - last_rad - M_PI*2.0 :
						((rad - last_rad < -M_PI) ? rad - last_rad + M_PI*2.0 : rad - last_rad);
			last_rad = rad;
			_now_radps = rad_div * 1000.0f / _scheduler.GetPeriod();
			_now_rad +=  rad_div;
		}


		if(_move_type == MoveType::stop){
			_now_duty = 0;
			ResetParam();
			_rad_pid.Calculate(0, _scheduler.GetPeriod());
			_radps_pid.Calculate(0, _scheduler.GetPeriod());
			__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		}
		else{
			if(_move_type == MoveType::duty){
				_radps_pid.Reset();
				_now_duty = _target_duty;
			}
			else{
				if(_encoder.get() == nullptr){//no encoder
					Stop();
					return;
				}

				if(_move_type == MoveType::radps){
					/*control speed*/
					_now_duty = _radps_pid.Calculate(_target_radps - _now_radps, _scheduler.GetPeriod());
					_radps_pid.Calculate(0, _scheduler.GetPeriod());
				}
				else if(_move_type == MoveType::radMulti || _move_type == MoveType::radSingle || _move_type == MoveType::radSinglePolarity){
					_target_radps = _rad_pid.Calculate(_target_rad - _now_rad,  _scheduler.GetPeriod());
					_now_duty = _radps_pid.Calculate(_target_radps - _now_radps, _scheduler.GetPeriod());
				}
				else{
					/* not yet!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
					Stop();
					return;
					/******************************************/
				}
			}

			/* set duty */
			__HAL_TIM_SET_COMPARE(_htim, _channel, static_cast<uint16_t>(std::fabs(_now_duty) / 100.0 * _htim->Instance->ARR));
			if(_now_duty != 0.0f)HAL_GPIO_WritePin(_gpio_port, _gpio_pin, (_now_duty > 0.0) ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
		_scheduler.Set();
	}

	bool ChangeEncoder(){
		if(_move_type != MoveType::stop)return false;

		/* not yet  */
		return false;
	}

	/**
	 * @brief 制御スタート
	 */
	virtual bool Start() override{
		ResetTarget();
		_move_type = MoveType::duty;
		__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		HAL_TIM_PWM_Start(_htim, _channel);
		return true;
	}

	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() override{
		_move_type = MoveType::stop;
		ResetParam();
		__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		HAL_TIM_PWM_Stop(_htim, _channel);
	}


	/**
	 * @brief 速度制御
	 * @param[in] rpm RPM
	 * @return 速度制御可能かどうか
	 */
	virtual bool SetRadps(float radps) override {
		if(_encoder.get() == NULL)return false;
		if(_move_type == MoveType::stop) return false;
		_target_radps = radps;
		_move_type = MoveType::radps;
		return true;
	}

	/**
	 * @brief 多回転角度制御
	 * @param[in] rad 角度[rad]
	 * @return 角度制御可能かどうか
	 */
	virtual bool SetRadMulti(float rad){
		if(_encoder.get() == NULL)return false;
		if(_move_type == MoveType::stop) return false;
		_target_rad = rad;
		_move_type = MoveType::radMulti;
		return true;
	}

	/**
	 * @brief 単回転角度制御
	 * @details 近い方向に回転します
	 * @param[in] rad 角度[rad]
	 * @details M_PI ~ -M_PIまで
	 * @return 角度制御可能かどうか
	 */
	virtual bool SetRadSingle(float rad){
		if(_encoder.get() == NULL)return false;
		if(_move_type == MoveType::stop) return false;
		if(std::fabs(rad) > static_cast<float>(M_PI))return false;

		float rad_diff = std::fmod(rad - _now_rad, M_PI*2.0);
		if(std::fabs(rad_diff) >= static_cast<float>(M_PI)){//over rad
			rad_diff = (rad_diff < 0 ? M_PI*2.0 : -M_PI*2.0) + rad_diff;
		}

		_target_rad = _now_rad + rad_diff;
		_move_type = MoveType::radSingle;
		return true;
	}


	/**
	 * @brief 単回転角度制御
	 * @details 指示極性方向に回転します.
	 * @param[in] rad 角度[rad]
	 * @param[in] polarity 極性
	 * @details trueなら正,falsなら負です
	 * @return 角度制御可能かどうか
	 */
	virtual bool SetRadSingle(float rad, bool polarity){
		if(_encoder.get() == NULL)return false;
		if(_move_type == MoveType::stop) return false;
		if(std::fabs(rad) > static_cast<float>(M_PI))return false;

		/* not yet!!!!! */
/*
		float rad_diff = std::fmod(rad - _now_rad, M_PI*2.0);
		if(std::abs(rad_diff) > M_PI)//over rad
			rad_diff = (rad_diff < 0 ? -M_PI : M_PI) - rad_diff;

		_target_rad = _now_rad + rad_diff;
		_move_type = MoveType::radSinglePolarity;
		return true;
		*/

		return false;
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
