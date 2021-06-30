/**
 *
 */
#pragma once

#include "../Global.hpp"

namespace nut{
/**
 *
 */
class PWMch{
private:
	TIM_HandleTypeDef* const _htim;
	const uint32_t _channel;

public:
	PWMch(TIM_HandleTypeDef* htim, uint32_t ch) :
		_htim(htim), _channel(ch){
	}


	TIM_HandleTypeDef* GetTimHnadler()const{
		return _htim;
	}
	uint32_t GetTimChannel() const{
		return _channel;
	}

	void Start(){
		//HAL_TIM_Base_Start(_htim);
		HAL_TIM_PWM_Start(_htim, _channel);
	}
	void Stop(){
		HAL_TIM_PWM_Stop(_htim, _channel);
	}
	void WriteCCR(uint32_t value){
		__HAL_TIM_SET_COMPARE(_htim, _channel, value);
	}
	void WriteParCCR(float par_value){
		__HAL_TIM_SET_COMPARE(_htim, _channel, _htim->Instance->ARR * par_value / 100.0f);
	}
};
}
