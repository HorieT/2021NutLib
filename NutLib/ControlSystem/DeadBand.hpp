/**
 *
 *
 */
#pragma once
#include "Controller.hpp"


namespace nut{
/**
 * @brief 不感帯
 * @details upper~lowerの値を0とする不感帯
 */
template<typename T>
class DeadBand : public Controller<T, 1, 1>{
private:
	T _upper = 1;
	T _lower = -1;

public:
	DeadBand(){

	}
	DeadBand(T upper, T lower){
		if(upper > lower){
			_upper = upper;
			_lower = lower;
		}
	}

	virtual ~DeadBand(){

	}


	/**
	 * @brief 制御器演算
	 * @param[in] input 入力値
	 * @param[in] ms 制御周期時間[ms] 計算に使わない
	 * @return 出力値
	 */
	virtual T Calculate(T input, nut::MilliSecond<float> ms = 0){
		if(input > _upper){
			auto tmp = input - _upper;
			return tmp > _limit ? _limit : tmp;
		}
		else if(input < _lower){
			auto tmp = input - _lower;
			return tmp < -_limit ? -_limit : tmp;
		}
		return 0;
	}

	/**
	 * @brief 演算リセット
	 */
	virtual void Reset() override{
	}



	bool SetUpper(T upper){
		if(upper > _lower){
			_upper = upper;
			return true;
		}
		return false;
	}
	bool SetLower(T lower){
		if(lower < _upper){
			_lower = lower;
			return true;
		}
		return false;
	}

	T GetUpper()const{return _upper;}
	T Getlower()const{return _lower;}
};


}
