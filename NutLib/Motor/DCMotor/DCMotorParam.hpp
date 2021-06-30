/**
 *
 */
#pragma once

#include "../../Global.hpp"
#include "../../Unit/UnitCore.hpp"

namespace nut{
/**
 *
 */
class DCMotorParam{
private:
	const Ohm<float> _s_resistance;
	const Henry<float> _s_inductance;
	const float _torque_constant;		// Nm/A
	const float _loss_torque;			// Nm
	const float _back_emf_constant;		// Vs/rad
	const Second<float> _time_constant;

	Second<float> _current_setting_time = 0.1f;
	float _corner_frequency = 0;		// rad/s
	float _current_p_gain_base = 0;
	float _current_i_gain_base = 0;

public:

	constexpr DCMotorParam(Ohm<float> Rm, Henry<float> Lm, float Kt, float To, float Ke) :
	_s_resistance(Rm),
	_s_inductance(Lm),
	_torque_constant(Kt),
	_loss_torque(To),
	_back_emf_constant(Ke),
	_time_constant(_s_inductance.f() / _s_resistance.f())
	{
		SetCurrentSettingTime(0.1f);
	}
	constexpr DCMotorParam(DCMotorParam param, Second<float> Tc) : DCMotorParam(param){
		SetCurrentSettingTime(Tc);
	}

	constexpr void SetCurrentSettingTime(Second<float> Tc){
		if(Tc <= 0.0)return;
		_current_setting_time = Tc;
		_corner_frequency = 3.0f / _current_setting_time.f();
		_current_p_gain_base = _s_inductance.f() * _corner_frequency;
		_current_i_gain_base = _s_resistance.f() * _corner_frequency;

	}

	constexpr Second<float> GetTm()const{return _time_constant;}
	constexpr float GetWi()const{return _corner_frequency;}
	constexpr float GetKip(Volt<float> volt)const{return _current_p_gain_base * 100.0f / volt.f();}
	constexpr float GetKii(Volt<float> volt)const{return _current_i_gain_base * 100.0f / volt.f();}
};
}
