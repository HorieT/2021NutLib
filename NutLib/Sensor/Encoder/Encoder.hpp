/*
 * ロータリーエンコーダ基底クラス
 * タイヤ、またはギヤのサイズごと包括
 */
#pragma once

#include "../../Global.hpp"


namespace nut{
class Encoder{
protected:
	const uint32_t _resolution;

public:
	/*
	 * コンストラクタ、デストラクタ
	 */
	Encoder(uint32_t resolution): _resolution(resolution){}
	virtual ~Encoder(){}

	/*
	 * 初期化
	 */
	virtual void Init() = 0;


	/*
	 * 角度リセット
	 */
	virtual void Reset() = 0;

	/*
	 * 角度取得
	 */
	virtual float GetRad() const = 0;
	/*
	 * 角度取得&リセット
	 * インクリメント型ならこちらの方がより正確
	 */
	virtual float GetRadAndReset() = 0;
};
}
