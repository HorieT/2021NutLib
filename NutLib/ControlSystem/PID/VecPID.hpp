/*
 *
 *
 */
#pragma once

#include "PIDBase.hpp"



namespace nut{

template<typename T>
class VecPID : public PIDBase<T>{
protected:
	T _output_value = 0.0;

public:
	VecPID(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0):
	PIDBase<T>(P_gain, I_gain, D_gain){

	}
	virtual ~VecPID(){

	}


	/**
	 * @brief フィードバック計算
	 * @param[in] input 偏差
	 * @param[in] ms 前回計算時からの経過時間
	 * @return 操作量
	 */
	virtual T Calculate(T input, uint32_t ms) override{
		PIDBase<T>::_deviation[2] = PIDBase<T>::_deviation[1];
		PIDBase<T>::_deviation[1] = PIDBase<T>::_deviation[0];
		PIDBase<T>::_deviation[0] = input;
		_output_value += PIDBase<T>::P() * (
				PIDBase<T>::_deviation[0] - PIDBase<T>::_deviation[1] +
				PIDBase<T>::I() * static_cast<T>(ms) * 0.001 * PIDBase<T>::_deviation[0] +
				PIDBase<T>::D() / static_cast<T>(ms) * 1000. * (PIDBase<T>::_deviation[0] - 2.0 * PIDBase<T>::_deviation[1] + PIDBase<T>::_deviation[2]));

		if(std::abs(_output_value) > PIDBase<T>::_limit)
			_output_value = (_output_value > 0) ? PIDBase<T>::_limit : -PIDBase<T>::_limit;
		return _output_value;
	}

	/**
	 * @brief 操作量リセット
	 */
	virtual void Reset() override{
		PIDBase<T>::_deviation = {0.0};
		_output_value = 0.0;
	}
};
}
