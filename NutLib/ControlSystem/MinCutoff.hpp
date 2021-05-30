/**
 *
 *
 */
#pragma once
#include "Controller.hpp"

namespace nut{
/**
 * @brief 最小値カットオフ
 * @details upper~lowerの値を0として出力、その他はそのまま
 */
template<typename T>
class MinCutoff : public Controller<T, 1, 1>{
private:
	T _upper = 1;
	T _lower = -1;



public:
	MinCutoff(){

	}
	MinCutoff(T upper, T lower){
		if(upper > lower){
			_upper = upper;
			_lower = lower;
		}
	}
	virtual ~MinCutoff(){

	}


	/**
	 * @brief 制御器演算
	 * @param[in] input 入力値
	 * @param[in] ms 制御周期時間[ms] 計算に使わない
	 * @return 出力値
	 */
	virtual T Calculate(T input, nut::MilliSecond<float> ms = 0) override{
		if(input > _upper){
			return input > Controller<T, 1, 1>::_limit ? Controller<T, 1, 1>::_limit : input;
		}
		else if(input < _lower){
			return input < -Controller<T, 1, 1>::_limit ? -Controller<T, 1, 1>::_limit : input;
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
