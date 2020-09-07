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
	std::shared_ptr<Encoder> _encoder;
	float _diameter_mm;
	Coordinate<float> _position;

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] encoder エンコーダインスタンス
	 * @param[in] diameter_mm 直径[mm]
	 * @param[in] position エンコーダ位置
	 */
	EncoderWheel(const std::shared_ptr<Encoder>& encoder, float diameter_mm, Coordinate<float> position = Coordinate<float>{0.0f, 0.0f, 0.0f})
	: _encoder(encoder), _diameter_mm(diameter_mm), _position(position){

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
	float GetRad() const{
		return _encoder->GetRad();
	}
	/**
	 * @brief 角度取得&カウントリセット
	 * @details 周期角度取得精度を上げるためのもの
	 * @return 角度[rad]
	 */
	float GetRadAndReset(){
		return _encoder->GetRadAndReset();
	}
	/**
	 * @brief 距離取得
	 * @return 距離[mm]
	 */
	float GetDistance() const {
		return _encoder->GetRad() * _diameter_mm;
	}
	/**
	 * @brief 距離取得&カウントリセット
	 * @details 周期距離取得精度を上げるためのもの
	 * @return 距離[mm]
	 */
	float GetDistanceAndReset() {
		return _encoder->GetRadAndReset() * _diameter_mm;
	}

	/**
	 * @brief 位置取得
	 */
	const Coordinate<float> GetPosition() const{
		return _position;
	}
};
}