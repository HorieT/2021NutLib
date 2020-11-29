/**
 * @file DirectDutyMotor.hpp
 * @brief 電流制御モータ
 * @author Horie
 * @date 2020/10
 */
#pragma once

#include "DirectDutyMotor.hpp"

namespace nut{
/**
 * @brief 電流制御のモータクラス
 * @attention _schedulerを使用しません。
 */
class CurrentControlMotor : public DirectDutyMotor{
private:
	float _controll_period_ms;//ms


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		//get radian
		if(_encoder){
			float rad = _encoder->GetRad();
			float rad_div = (rad - last_rad > M_PI_f) ?
						rad - last_rad - M_2PI_f:
						((rad - last_rad < -M_PI_f) ? rad - last_rad + M_2PI_f : rad - last_rad);
			rad_div *= _encoder_ratio;
			last_rad = rad;
			_now_radps = rad_div * 1000.0f / static_cast<float>(_scheduler.GetPeriod());
			_now_rad +=  rad_div;
		}

		//move control
		if(_move_type == MoveType::stop){
			_now_duty = 0;
			ResetTarget();
			ResetController();
			__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		}
		else{
			if(_move_type == MoveType::duty){
				ResetController();
			}
			else{
				if(_move_type == MoveType::currnet){
					_radps_pid->Reset();
					_rad_pid->Reset();
				}
				else{
					if(_encoder){//no encoder
						Stop();
						return;
					}

					switch(_move_type){
					/*control speed*/
					case MoveType::radps:
						_rad_pid->Calculate(0.0f, _controll_period_ms);
						_target_current = _radps_pid->Calculate(_target_radps - _now_radps, _controll_period_ms);
						break;

						/*control rad*/
					case MoveType::radMulti:
					case MoveType::radSingle:
					case MoveType::radSinglePolarity:
						_target_radps = _rad_pid->Calculate(_target_rad - _now_rad,  _controll_period_ms);
						_target_current = _radps_pid->Calculate(_target_radps - _now_radps, _controll_period_ms);
						break;

					default:
						/* not yet!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
						Stop();
						return;
					}
				}
				_target_duty = _current_pid->Calculate(_target_current - _now_current, _controll_period_ms);
			}

			_now_duty = _target_duty;
			/* set duty */
			__HAL_TIM_SET_COMPARE(_htim, _channel, static_cast<uint32_t>(std::fabs(_now_duty) / 100.0f * _htim->Instance->ARR));
			if(_now_duty != 0.0f)HAL_GPIO_WritePin(_gpio_port, _gpio_pin, (_now_duty > 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		}
	}

public:
	/**
	 * @brief コンストラク
	 * @param[in] period 電流入力周期[ms]
	 * @param[in] htim PWM出力タイマ
	 * @param[in] channel PWM出力チャンネル
	 * @param[in] port 回転極性ピンのポート
	 * @param[in] pin 回転極性ピン
	 * @param[in] encoder エンコーダのインスタンス
	 * @details エンコーダを使わない場合はヌルポを入れてください
	 * @param[in] encoder_ratio エンコーダ一回転に対する実効回転数の比
	 */
	CurrentControlMotor(
			MilliSecond<float> period,
			TIM_HandleTypeDef* htim,
			uint32_t channel,
			GPIO_TypeDef* port,
			uint16_t pin,
			std::shared_ptr<Encoder> encoder,
			float encoder_ratio = 1.0) :
		DirectDutyMotor(0xFFFFFFFF, htim, channel, port, pin, encoder, encoder_ratio){
		_radps_pid->SetLimit(99.0f);//duty limit
		_controll_period_ms = static_cast<float>(period);
	}
	virtual ~CurrentControlMotor(){Deinit();}



	/**
	 * @brief 初期化関数
	 */
	virtual void Init() override{
		if(_is_init)return;
		_is_init = true;
		_encoder->Init();
	}
	/**
	 * @brief 非初期化関数
	 */
	virtual void Deinit() override{
		if(!_is_init)return;
		Stop();
		_is_init = false;
		_encoder->Deinit();
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
		ResetController();
		__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		HAL_TIM_PWM_Stop(_htim, _channel);
	}


	/* control set */

	/**
	 * @brief 電流制御
	 * @param[in] currnet 電流[A]
	 * @return 電流制御可能かどうか
	 */
	virtual bool SetCurrent(float currnet) override{
		if(_move_type == MoveType::stop) return false;
		_target_current = currnet;
		_move_type = MoveType::currnet;
		return true;
	}


	/**
	 * @brief 電流取得割込み呼び出し関数
	 * @details コンストラクタに入力した周期で呼び出してください
	 * @param[in] current 現在の電流[A]
	 */
	virtual bool CurrentEXTI(float current){
		if(!_is_init)return false;
		_now_current = current;
		ScheduleTask();
		return true;
	}
};
}