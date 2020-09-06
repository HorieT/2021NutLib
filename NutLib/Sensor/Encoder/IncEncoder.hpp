/**
 * @file IncEncoder.hpp
 * @brief インクリメンタル型エンコーダ
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Encoder.hpp"
#include <cstring>

namespace nut{
/**
 * @brief インクリメンタル型エンコーダクラス
 */
class IncEncoder : public Encoder{
private:
	TIM_HandleTypeDef* const _htim;


	/**
	 * @brief カウントを角度へ換算
	 * @param[in] count カウント
	 * @return 角度[rad]
	 */
	float ConvertRad(uint32_t count) const{
		//32bit counter
		if(_htim->Instance->ARR == 0xFFFFFFFF){
			int32_t signedCount;
			std::memcpy(&signedCount, &count, 4);
			return static_cast<float>(signedCount) * static_cast<float>(M_PI) / static_cast<float>(_resolution);
		}
		//16bit counter
		else if(_htim->Instance->ARR == 0xFFFF){
			int16_t signedCount;
			std::memcpy(&signedCount, &count, 2);
			return static_cast<float>(signedCount) * static_cast<float>(M_PI) / static_cast<float>(_resolution);
		}
		//例外的なカウンタ
		else{
			int32_t signedCount = (count > (_htim->Instance->ARR / 2U)) ?
					static_cast<int32_t>(count - _htim->Instance->ARR) : static_cast<int32_t>(count);
			return static_cast<float>(signedCount) * static_cast<float>(M_PI) / static_cast<float>(_resolution);
		}
	}

public:
	/**
	 * @brief コンストラクタ
	 * @param tim タイマインスタンス
	 * @param resolution 分解能
	 */
	IncEncoder(TIM_HandleTypeDef* tim, uint32_t resolution):
		Encoder(resolution),_htim(tim){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~IncEncoder(){HAL_TIM_Encoder_Stop(_htim, TIM_CHANNEL_ALL);}

	/**
	 * @brief 初期化
	 */
	virtual void Init() override{
		_htim->Instance->CNT = 0;
		HAL_TIM_Encoder_Start(_htim, TIM_CHANNEL_ALL);
	}



	/**
	 * @brief リセット
	 */
	virtual void Reset() override{
		_htim->Instance->CNT = 0;
	}

	/**
	 * @brief 角度取得
	 * @return 角度[rad]
	 */
	virtual float GetRad() const override {
		uint32_t count = _htim->Instance->CNT;
		return ConvertRad(count);
	}
	/**
	 * @brief 角度取得&カウントリセット
	 * @details 周期角度取得精度を上げるためのもの
	 * @return 角度[rad]
	 */
	virtual float GetRadAndReset() override{
		uint32_t count = _htim->Instance->CNT;
		_htim->Instance->CNT = 0;
		return ConvertRad(count);
	}
};
}
