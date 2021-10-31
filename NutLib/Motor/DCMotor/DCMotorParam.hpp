/**
 *
 */
#pragma once

#include "../../Global.hpp"
#include "../../Unit/UnitCore.hpp"

namespace nut{
/**
 * @brief DCモータパラメータクラス
 * @details 電流PI制御パラメータ算出もサポート
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

	/**
	 * @brief コンストラクタ
	 * @param[in] Rm 巻線抵抗
	 * @param[in] Lm 巻線インダクタンス
	 * @param[in] Kt トルク定数[Nm/A]
	 * @param[in] To 損失トルク[Nm]
	 * @param[in] Ke 誘起電圧定数[Vs/rad]
	 */
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
	/**
	 * @brief コンストラクタ
	 * @param[in] param モータパラメータ
	 * @param[in] Tc 電流整定時間
	 * @details 電流PI制御パラメータの算出に使います
	 */
	constexpr DCMotorParam(DCMotorParam param, Second<float> Tc) : DCMotorParam(param){
		SetCurrentSettingTime(Tc);
	}

	/**
	 * @brief 電流整定時間
	 * @param[in] Tc 電流整定時間
	 * @details 電流PI制御パラメータの算出に使います
	 */
	constexpr void SetCurrentSettingTime(Second<float> Tc){
		if(Tc <= 0.0)return;
		_current_setting_time = Tc;
		_corner_frequency = 3.0f / _current_setting_time.f();
		_current_p_gain_base = _s_inductance.f() * _corner_frequency;
		_current_i_gain_base = _s_resistance.f() * _corner_frequency;

	}

	/**
	 * @brief トルク定数取得
	 * @return トルク定数[Nm/A]
	 */
	constexpr float GetKt() const{return _torque_constant;}
	/**
	 * @brief モータ時定数取得
	 * retun 時定数
	 */
	constexpr Second<float> GetTm()const{return _time_constant;}
	/**
	 * @brief 電流PI制御 折れ点角周波数取得
	 * @return 折れ点周波数[rad/s]
	 */
	constexpr float GetWi()const{return _corner_frequency;}
	/**
	 * @brief 電流PI制御 比例係数取得
	 * @param[in] volt バッテリ電圧
	 * @return 比例係数
	 */
	constexpr float GetKip(Volt<float> volt)const{return _current_p_gain_base * 100.0f / volt.f();}
	/**
	 * @brief 電流PI制御 積分係数取得
	 * @param[in] volt バッテリ電圧
	 * @return 積分係数
	 */
	constexpr float GetKii(Volt<float> volt)const{return _current_i_gain_base * 100.0f / volt.f();}
};
}
