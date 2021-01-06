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
	 * @param[in] input 偏差
	 * @param[in] ms 前回計算時からの経過時間
	 * @return 操作量
	 */
	virtual T Calculate(T input, nut::MilliSecond<float> ms) override{
		this->_deviation[1] = this->_deviation[0];
		this->_deviation[0] = input;
		T P_value = this->_P_gain * this->_deviation[0];
		_I_stack += this->_I_gain * static_cast<T>(ms) * (this->_deviation[0] + this->_deviation[1]) * 0.5 * 0.001;//sに直す
		T D_value = this->_D_gain / static_cast<T>(ms) * (this->_deviation[0] - this->_deviation[1]) * 1000.0;//sに直す
		if(!std::isinf(_I_limit)){
			if(_I_stack > _I_limit)_I_stack = _I_limit;
			else if(_I_stack < -_I_limit)_I_stack = -_I_limit;
		}
		T value = P_value + _I_stack + D_value;

		if(std::abs(value) > this->_limit)
			value = (value > 0) ? this->_limit : -this->_limit;
		return value;
	}

	/**
	 * @brief 操作量リセット
	 */
	virtual void Reset() override{
		this->_deviation = {0.0};
		_I_stack = static_cast<T>(0);
	}

	void SetLimitI(T limit){
		Reset();
		_I_limit = limit;
	}
};
}
