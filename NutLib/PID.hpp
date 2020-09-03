/*
 * PID計算クラス
 */
#pragma once

#include "Global.hpp"
#include <type_traits>

namespace nut{
template<typename T>
class PID{
	static_assert(std::is_floating_point_v<T>, "Type is not floating point.");
private:
	T _value;//出力値
	std::array<T, 3> _deviation = {0.0f, 0.0f, 0.0f};//偏差

protected:
	/*各ゲイン*/
	T _P_gain;
	T _I_gain;
	T _D_gain;

	T _limit;


public:
	PID(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0, T limit = static_cast<T>(0))
		: _value(static_cast<T>(0)), _P_gain(P_gain), _I_gain(I_gain), _D_gain(D_gain),_limit(limit){}
	//copy禁止
	PID(const PID<T>& pid) = delete;
	PID& operator=(const PID<T>& pid) = delete;
	virtual ~ PID(){}



	const T& Calculate(T deviation, uint32_t ms = 1){
		_deviation[2] = _deviation[1];
		_deviation[1] = _deviation[0];
		_deviation[0] = deviation;
		_value +=
				_P_gain * (_deviation[0] - _deviation[1]) +
				_I_gain * static_cast<T>(ms) * (_deviation[0]) +
				_D_gain / static_cast<T>(ms) * (_deviation[0] - 2.0 * _deviation[1] + _deviation[2]);

		if((_limit != static_cast<T>(0)) && (std::abs(_value) > _limit))
			_value = (_value > 0) ? _limit : -_limit;
		return _value;
	}


	inline void SetGaine(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0){
		_value = static_cast<T>(0);

		_P_gain = P_gain;
		_I_gain = I_gain;
		_D_gain = D_gain;
	}
	inline void SetLimit(T limit){
		_value = static_cast<T>(0);

		_limit = limit;
	}


	/*
	 * ゲッター
	 */
	inline const T& GetValue() const{return _value;}
	/*
	 * セッター
	 */
	inline void SetValue(T value){_value = value;}
};
}
