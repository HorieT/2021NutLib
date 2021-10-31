/**
 *
 */
#pragma once

#include "../Global.hpp"

namespace nut{
/**
 *　@brief pwmチャンネルクラス
 */
class PWMch{
private:
	TIM_HandleTypeDef* const _htim;
	const uint32_t _channel;

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] htim タイマ
	 * @param[in] ch チャンネル
	 */
	PWMch(TIM_HandleTypeDef* htim, uint32_t ch) :
		_htim(htim), _channel(ch){
	}


	/**
	 * @brief タイマの取得
	 * @return タイマ
	 */
	TIM_HandleTypeDef* GetTimHnadler()const{
		return _htim;
	}
	/**
	 * @brief チャンネルの取得
	 * @return チャンネル
	 */
	uint32_t GetTimChannel() const{
		return _channel;
	}

	/**
	 * @brief PWMスタート
	 */
	void Start(){
		//HAL_TIM_Base_Start(_htim);
		HAL_TIM_PWM_Start(_htim, _channel);
	}
	/**
	 * @brief PWMストップ
	 */
	void Stop(){
		HAL_TIM_PWM_Stop(_htim, _channel);
	}
	/**
	 * @brief CCR書き込み
	 * @param[in] value CCR値
	 */
	void WriteCCR(uint32_t value){
		__HAL_TIM_SET_COMPARE(_htim, _channel, value);
	}
	/**
	 * @brief CCR書き込み(duty比)
	 * @param[in] par_value duty比[%]
	 */
	void WriteParCCR(float par_value){
		__HAL_TIM_SET_COMPARE(_htim, _channel, _htim->Instance->ARR * par_value / 100.0f);
	}
};
}
