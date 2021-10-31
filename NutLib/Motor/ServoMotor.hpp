/**
 * @brief ラジコンサーボのやつ
 */
#pragma once

#include "../Global.hpp"
#include "../Unit/UnitCore.hpp"
#include "../Etc/PWMch.hpp"


namespace nut{
/**
 * @brief ラジコンサーボクラス
 */
class ServoMotor{
protected:
	PWMch _pwm;
	const Radian<float> _motion_range;
	bool _is_init = false;

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] htim タイマ
	 * @param[in] ch チャンネル
	 * @param[in] motion_range サーボ可動域
	 */
	ServoMotor(TIM_HandleTypeDef* htim, uint32_t ch, Radian<float> motion_range) :
		_pwm(htim, ch), _motion_range(motion_range){
	}
	/**
	 * デストラクタ
	 */
	virtual ~ServoMotor(){

	}


	/**
	 * @brief 初期化
	 */
	bool Init(){
		if(_is_init)return false;

		TorqeOff();
		_pwm.Start();
		_is_init = true;
		return true;
	}

	/**
	 * @brief 角度指示
	 * @param[in] rad 角度
	 */
	virtual void SetRad(Radian<float> rad){
		if(!_is_init)return;
		_pwm.WriteCCR(_pwm.GetTimHnadler()->Instance->ARR / 20.0f * (1.5f + rad.f() / _motion_range.f()));
	}
	/**
	 * @brief トルクオフ
	 */
	virtual void TorqeOff(){
		if(!_is_init)return;
		_pwm.WriteCCR(0);
	}
};
}
