/**
 * @file DirectDutyMotor.hpp
 * @brief Dutyモータ制御
 * @author Horie
 * @date 2020/10
 */
#pragma once

#include "Motor.hpp"
#include "../Sensor/Encoder/Encoder.hpp"
#include <memory>
#include <limits>

namespace nut{
/**
 * @brief Duty制御のモータクラス
 */
class DirectDutyMotor : public Motor{
protected:
	/* Component */
	TIM_HandleTypeDef* const _htim;// PWM
	const uint32_t _channel;// PWM
	GPIO_TypeDef* const _gpio_port;
	const uint16_t _gpio_pin;
	const std::shared_ptr<Encoder> _encoder;
	float _encoder_ratio = 1.0;

	/* for calculation */
	float last_rad = std::numeric_limits<float>::quiet_NaN();



	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		//get radian
		if(_encoder){
			float rad = _encoder->GetRad();
			float rad_div = 0;
			if(!std::isnan(last_rad)){
				rad_div = (rad - last_rad > M_PI_f) ?
						rad - last_rad - M_2PI_f:
						((rad - last_rad < -M_PI_f) ? rad - last_rad + M_2PI_f : rad - last_rad);
				rad_div *= _encoder_ratio;
				_now_radps = rad_div * 1000.0f / static_cast<float>(_scheduler.GetPeriod());
			}else{
				rad_div = (rad> M_PI_f) ?
						rad - M_2PI_f:
						((rad < -M_PI_f) ? rad + M_2PI_f : rad);
				rad_div *= _encoder_ratio;
			}
			last_rad = rad;
			_now_rad +=  rad_div;
		}


		if(_move_type == MoveType::stop){
			_now_duty = 0;
			ResetTarget();
			ResetController();
			__HAL_TIM_SET_COMPARE(_htim, _channel, 0);
		}
		else{
			if(_move_type == MoveType::duty){
				ResetController();
				_now_duty = _target_duty;
			}
			else{
				if(!_encoder){//no encoder
					Stop();
					return;
				}

				switch(_move_type){
				/*control speed*/
				case MoveType::radps:
					_now_duty = _radps_pid->Calculate(_target_radps - _now_radps, static_cast<uint32_t>(_scheduler.GetPeriod()));
					_rad_pid->Calculate(0.0f, static_cast<uint32_t>(_scheduler.GetPeriod()));
					break;

				/*control rad*/
				case MoveType::radMulti:
				case MoveType::radSingle:
				case MoveType::radSinglePolarity:
					_target_radps = _rad_pid->Calculate(_target_rad - _now_rad,  static_cast<uint32_t>(_scheduler.GetPeriod()));
					_now_duty = _radps_pid->Calculate(_target_radps - _now_radps, static_cast<uint32_t>(_scheduler.GetPeriod()));
					break;

				default:
					/* not yet!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
					Stop();
					return;
				}
			}

			/* set duty */
			__HAL_TIM_SET_COMPARE(_htim, _channel, static_cast<uint32_t>(std::fabs(_now_duty) / 100.0f * _htim->Instance->ARR));
			if(_now_duty != 0.0f)HAL_GPIO_WritePin(_gpio_port, _gpio_pin, (_now_duty > 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
	 * @param[in] encoder_ratio エンコーダ一回転に対する実効回転数の比
	 */
	DirectDutyMotor(
			MilliSecond<uint32_t> period,
			TIM_HandleTypeDef* htim,
			uint32_t channel,
			GPIO_TypeDef* port,
			uint16_t pin,
			std::shared_ptr<Encoder> encoder,
			float encoder_ratio = 1.0) :
		Motor(period), _htim(htim),_channel(channel), _gpio_port(port), _gpio_pin(pin), _encoder(encoder), _encoder_ratio(encoder_ratio){
		_radps_pid->SetLimit(99.0f);//duty limit
	}
	virtual ~DirectDutyMotor(){Deinit();}



	/**
	 * @brief 初期化関数
	 */
	virtual void Init() override{
		if(_is_init)return;
		_is_init = true;
		_encoder->Init();
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
		_encoder->Deinit();
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

	/*
	 * @brief エンコーダ変更
	 * @details これ要る？？？
	 */
	bool ChangeEncoder(){
		if(_move_type != MoveType::stop)return false;

		/* not yet  */
		return false;
	}


	/* control set */

	/**
	 * @brief 速度制御
	 * @param[in] radps rad/s
	 * @return 速度制御可能かどうか
	 */
	virtual bool SetRadps(float radps) override {
		if(_move_type == MoveType::stop || !_encoder) return false;
		_target_radps = radps;
		_move_type = MoveType::radps;
		return true;
	}
	/**
	 * @brief 多回転角度制御
	 * @param[in] rad 角度[rad]
	 * @return 角度制御可能かどうか
	 */
	virtual bool SetRadMulti(float rad) override{
		if(_move_type == MoveType::stop || !_encoder) return false;
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
	virtual bool SetRadSingle(float rad) override{
		if(_move_type == MoveType::stop || !_encoder) return false;
		if(std::fabs(rad) > M_PI_f)return false;

		float rad_diff = std::fmod(rad - _now_rad, M_2PI_f);
		if(std::fabs(rad_diff) >= M_PI_f){//over rad
			rad_diff = (rad_diff < 0 ? M_2PI_f : -M_2PI_f) + rad_diff;
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
	virtual bool SetRadSingle(float rad, bool polarity) override{
		if(_move_type == MoveType::stop || !_encoder) return false;
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
	 * @brief 電流制御
	 * @param[in] currnet 電流[A]
	 * @return 電流制御可能かどうか
	 */
	virtual bool SetCurrent(float currnet) override{
		return false;
	}




	/* setter */

	/**
	 * @brief 角度原点リセット
	 * @details 現在角を書き換えます
	 * @param[in] rad 書き換え角度
	 * @return bool 角度原点リセット可能かどうか
	 */
	virtual bool ResetRadOrigin(float rad) override{
		if(_move_type != MoveType::stop || !_encoder)return false;
		_now_rad = rad;
		return true;
	}


	/* getter */

	/*
	 * @brief PWMタイマ取得
	 * @return PWMタイマ
	 */
	TIM_HandleTypeDef* GetPWMTimer() const{
		return _htim;
	}
};
}
