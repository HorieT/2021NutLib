/**
 * @brief ラジコンサーボのやつ
 */
#pragma once

#include "../Global.hpp"
#include "../Unit/UnitCore.hpp"
#include "../Etc/PWMch.hpp"


namespace nut{
/**
 *
 */
class ServoMotor{
protected:
	PWMch _pwm;
	const Radian<float> _motion_range;
	bool _is_init = false;

public:
	ServoMotor(TIM_HandleTypeDef* htim, uint32_t ch, Radian<float> motion_range) :
		_pwm(htim, ch), _motion_range(motion_range){
	}
	virtual ~ServoMotor(){

	}


	bool Init(){
		if(_is_init)return false;

		TorqeOff();
		_pwm.Start();
		_is_init = true;
		return true;
	}


	virtual void SetRad(Radian<float> rad){
		if(!_is_init)return;
		_pwm.WriteCCR(_pwm.GetTimHnadler()->Instance->ARR / 20.0f * (1.5f + rad.f() / _motion_range.f()));
	}
	virtual void TorqeOff(){
		if(!_is_init)return;
		_pwm.WriteCCR(0);
	}
};
}
