/*
 * インクリメンタル型エンコーダクラス
 */
#pragma once

#include "Encoder.hpp"
#include <cstring>

namespace nut{
class IncEncoder : public Encoder{
private:
	TIM_HandleTypeDef* const _htim;



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
		//例外的なリロードレジスタ処理
		else{
			int32_t signedCount = (count > (_htim->Instance->ARR / 2U)) ?
					static_cast<int32_t>(count - _htim->Instance->ARR) : static_cast<int32_t>(count);
			return static_cast<float>(signedCount) * static_cast<float>(M_PI) / static_cast<float>(_resolution);
		}
	}

public:
	/*
	 * コンストラクタ、デストラクタ
	 */
	IncEncoder(TIM_HandleTypeDef* tim, uint32_t resolution):
		Encoder(resolution),_htim(tim){}
	virtual ~IncEncoder(){HAL_TIM_Encoder_Stop(_htim, TIM_CHANNEL_ALL);}

	/*
	 * 初期化
	 */
	virtual void Init() override{
		_htim->Instance->CNT = 0;
		HAL_TIM_Encoder_Start(_htim, TIM_CHANNEL_ALL);
	}



	/*
	 * 角度リセット
	 */
	virtual void Reset() override{
		_htim->Instance->CNT = 0;
	}

	/*
	 * 角度取得
	 */
	virtual float GetRad() const override {
		uint32_t count = _htim->Instance->CNT;
		return ConvertRad(count);
	}
	/*
	 * 角度取得&リセット
	 */
	virtual float GetRadAndReset() override{
		uint32_t count = _htim->Instance->CNT;
		_htim->Instance->CNT = 0;
		return ConvertRad(count);
	}
};
}
