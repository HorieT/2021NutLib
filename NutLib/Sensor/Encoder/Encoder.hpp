/**
 * @file Encoder.hpp
 * @brief エンコーダ基底
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "../../Global.hpp"


namespace nut{
/**
 * @brief エンコーダ基底純粋仮想クラス
 */
class Encoder{
protected:
	const uint32_t _resolution;
	bool _is_init = false;

public:
	/**
	 * @brief コンストラクタ
	 * @param resolution 分解能
	 */
	Encoder(uint32_t resolution): _resolution(resolution){}
	/**
	 * @brief デストラクタ
	 */
	virtual ~Encoder(){}

	/**
	 * @brief 初期化
	 */
	virtual void Init() = 0;
	/**
	 * @brief 非初期化
	 */
	virtual void Deinit() = 0;


	/**
	 * @brief カウントリセット
	 */
	virtual void Reset() = 0;

	/**
	 * @brief 角度取得
	 * @return 角度[rad]
	 */
	virtual float GetRad() = 0;

	/**
	 * @brief 角度取得&カウントリセット
	 * @details 周期角度取得精度を上げるためのもの
	 * @return 角度[rad]
	 */
	virtual float GetRadAndReset() = 0;
};
}
