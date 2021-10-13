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
 */
class CurrentControlMotor : public DirectDutyMotor{
private:
	MilliSecond<float> _current_controll_period;


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		//get radian
		if(_encoder){
			Radian<float> rad = _encoder->GetRad();
			Radian<float> rad_div = 0;
			if(!std::isnan(last_rad)){
				rad_div = NormalizeRadian(rad - last_rad) * _encoder_ratio;
				_now_radps = rad_div.f() / Second<float>(_scheduler.GetPeriod()).f();
			}else{
				rad_div = NormalizeRadian(rad) * _encoder_ratio;
			}
			last_rad = rad.f();
			_now_rad +=  rad_div.f();
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
					if(!_encoder){//no encoder
						Stop();
						return;
					}

					switch(_move_type){
					/*control speed*/
					case MoveType::radps:
						_rad_pid->Calculate(0.0f, static_cast<uint32_t>(_scheduler.GetPeriod()));
						_target_current = _radps_pid->Calculate(_target_radps - _now_radps, static_cast<uint32_t>(_scheduler.GetPeriod()));
						break;

						/*control rad*/
					case MoveType::radMulti:
					case MoveType::radSingle:
					case MoveType::radSinglePolarity:
						_target_radps = _rad_pid->Calculate(_target_rad - _now_rad,  static_cast<uint32_t>(_scheduler.GetPeriod()));
						_target_current = _radps_pid->Calculate(_target_radps - _now_radps, static_cast<uint32_t>(_scheduler.GetPeriod()));
						break;

					default:
						/* not yet!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
						Stop();
						return;
					}
				}
				//_target_duty = _current_pid->Calculate(_target_current - _now_current, static_cast<uint32_t>(_scheduler.GetPeriod()));
			}

			//_now_duty = _target_duty;
			/* set duty */
			//__HAL_TIM_SET_COMPARE(_htim, _channel, static_cast<uint32_t>(std::fabs(_now_duty) / 100.0f * _htim->Instance->ARR));
			//if(_now_duty != 0.0f)HAL_GPIO_WritePin(_gpio_port, _gpio_pin, (_now_duty > 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		}
	}

public:
	/**
	 * @brief コンストラク
	 * @param[in] period_current 電流入力周期[ms]
	 * @param[in] period_controller 制御器周期[ms]
	 * @param[in] htim PWM出力タイマ
	 * @param[in] channel PWM出力チャンネル
	 * @param[in] port 回転極性ピンのポート
	 * @param[in] pin 回転極性ピン
	 * @param[in] encoder エンコーダのインスタンス
	 * @details エンコーダを使わない場合はヌルポを入れてください
	 * @param[in] encoder_ratio エンコーダ一回転に対する実効回転数の比
	 */
	CurrentControlMotor(
			MilliSecond<float> period_current,
			MilliSecond<float> period_controller,
			TIM_HandleTypeDef* htim,
			uint32_t channel,
			GPIO_TypeDef* port,
			uint16_t pin,
			std::shared_ptr<Encoder> encoder,
			float encoder_ratio = 1.0) :
		DirectDutyMotor(period_controller, htim, channel, port, pin, encoder, encoder_ratio),
		_current_controll_period(period_current){
		_radps_pid->SetLimit(99.0f);//duty limit
	}
	virtual ~CurrentControlMotor(){Deinit();}



	/**
	 * @brief 初期化関数
	 */
	virtual void Init() override{
		if(_is_init)return;
		_is_init = true;
		if(_encoder)_encoder->Init();
		_scheduler.Set();
		__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		HAL_TIM_PWM_Start(_htim, _channel);
	}
	/**
	 * @brief 非初期化関数
	 */
	virtual void Deinit() override{
		if(!_is_init)return;
		Stop();
		_is_init = false;
		if(_encoder)_encoder->Deinit();
		_scheduler.Erase();
		__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		HAL_TIM_PWM_Stop(_htim, _channel);
	}


	/**
	 * @brief 制御スタート
	 */
	virtual bool Start() override{
		ResetTarget();
		_move_type = MoveType::duty;
		__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		return true;
	}

	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() override{
		_move_type = MoveType::stop;
		ResetController();
		ResetTarget();
		__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
	}


	/* control set */

	/**
	 * @brief 電流制御
	 * @param[in] currnet 電流[A]
	 * @return 電流制御可能かどうか
	 */
	virtual bool SetCurrent(Ampere<float> currnet) override{
		if(_move_type == MoveType::stop) return false;
		_target_current = currnet.f();
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
		if(_move_type == MoveType::stop) return true;
		if(_move_type != MoveType::duty)_target_duty = _current_pid->Calculate(_target_current - _now_current, _current_controll_period);

		_now_duty = _target_duty;
		/* set duty */
		__HAL_TIM_SET_COMPARE(_htim, _channel, static_cast<uint32_t>(std::fabs(_now_duty) / 100.0f * _htim->Instance->ARR));
		if(_now_duty != 0.0f)HAL_GPIO_WritePin(_gpio_port, _gpio_pin, (_now_duty > 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
		return true;
	}
};
}
