/**
 * @file EncoderWheel.hpp
 * @brief 外径付きエンコーダ
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Encoder.hpp"
#include "../../Coordinate.hpp"
#include <memory>

namespace nut{
/**
 * @brief 外径付きエンコーダクラス
 */
class EncoderWheel{
private:
	const std::shared_ptr<Encoder> _encoder;
	const MilliMeter<float> _diameter;
	Coordinate<float> _position;

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] encoder エンコーダインスタンス
	 * @param[in] diameter 直径[mm]
	 * @param[in] position エンコーダ位置
	 */
	EncoderWheel(const std::shared_ptr<Encoder>& encoder, MilliMeter<float> diameter, Coordinate<float> position = Coordinate<float>{0.0f, 0.0f, 0.0f})
	: _encoder(encoder), _diameter(diameter), _position(position){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~EncoderWheel(){}

	/**
	 * @brief　初期化
	 */
	void Init(){
		_encoder->Init();
	}


	/**
	 * @brief　エンコーダのインスタンス取得
	 * @return エンコーダのインスタンス
	 */
	std::weak_ptr<Encoder> GetEncoder() const{
		return _encoder;
	}

	/**
	 * @brief　リセット
	 */
	void Reset(){
		_encoder->Reset();
	}
	/**
	 * @brief 角度取得
	 * @return 角度[rad]
	 */
	Radian<float> GetRad() const{
		return _encoder->GetRad();
	}
	/**
	 * @brief 角度取得&カウントリセット
	 * @details 周期角度取得精度を上げるためのもの
	 * @return 角度[rad]
	 */
	Radian<float> GetRadAndReset(){
		return _encoder->GetRadAndReset();
	}
	/**
	 * @brief 距離取得
	 * @return 距離[mm]
	 */
	MilliMeter<float> GetDistance() const {
		return _encoder->GetRad().f() * _diameter * 0.5;
	}
	/**
	 * @brief 距離取得&カウントリセット
	 * @details 周期距離取得精度を上げるためのもの
	 * @return 距離[mm]
	 */
	MilliMeter<float> GetDistanceAndReset() {
		return _encoder->GetRadAndReset().f() * _diameter * 0.5;
	}

	/**
	 * @brief 位置取得
	 */
	const Coordinate<float> GetPosition() const{
		return _position;
	}
};
}
