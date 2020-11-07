/*
 *
 *
 */
#pragma once

#include "PIDBase.hpp"



namespace nut{

template<typename T>
class PosPID : public PIDBase<T>{
protected:
	T _I_stack;//!< I操作量スタック
	T _I_limit = std::numeric_limits<T>::infinity();//!< I上限値(絶対値)


public:
	PosPID(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0):
	PIDBase<T>(P_gain, I_gain, D_gain){

	}
	virtual ~PosPID(){

	}



	/**
	 * @brief フィードバック計算
	 * @param[in] deviation 偏差
	 * @param[in] ms 前回計算時からの経過時間
	 * @return 操作量
	 */
	virtual T Calculate(T input, uint32_t ms) override{
		PIDBase<T>::_deviation[1] = PIDBase<T>::_deviation[0];
		PIDBase<T>::_deviation[0] = input;
		T P_value = PIDBase<T>::P() * PIDBase<T>::_deviation[0];
		_I_stack += PIDBase<T>::I() * static_cast<T>(ms) * (PIDBase<T>::_deviation[0] + PIDBase<T>::_deviation[1]) * 0.5 * 0.001;//sに直す
		T D_value = PIDBase<T>::D() / static_cast<T>(ms) * (PIDBase<T>::_deviation[0] - PIDBase<T>::_deviation[1]) * 1000.0;//sに直す
		if(!std::isinf(_I_limit)){
			if(_I_stack > _I_limit)_I_stack = _I_limit;
			else if(_I_stack < -_I_limit)_I_stack = -_I_limit;
		}
		T value = P_value + _I_stack + D_value;

		if(std::abs(value) > PIDBase<T>::_limit)
			value = (value > 0) ? PIDBase<T>::_limit : -PIDBase<T>::_limit;
		return value;
	}

	/**
	 * @brief 操作量リセット
	 */
	virtual void Reset() override{
		PIDBase<T>::_deviation = {0.0};
		_I_stack = static_cast<T>(0);
	}

	virtual void SetLimitI(T limit) final{
		Reset();
		_I_limit = limit;
	}
};
}
