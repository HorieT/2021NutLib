/**
 * @file DriveWheel.hpp
 * @brief 駆動輪
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "../Global.hpp"
#include "Motor.hpp"
#include <memory>


namespace nut{
/**
 * @brief 駆動輪クラス
 */
class DriveWheel{
protected:
	const std::shared_ptr<Motor> _motor;
	float _diameter_mm;
	const Coordinate<float> _position;


public:
	/**
	 * @brief コンストラクタ
	 * @param[in] motor モーターインスタンス
	 * @param[in] diameter_mm タイヤ直径[mm]
	 */
	DriveWheel(const std::shared_ptr<Motor>& motor, float diameter_mm, 	Coordinate<float> position = Coordinate<float>())
		: _motor(motor), _diameter_mm(diameter_mm), _position(position){

	}
	/**
	 * @brief デストラクタ
	 */
	~DriveWheel(){

	}

	const Coordinate<float>& GetPos()const {
		return _position;
	}


	/**
	 * @brief 速度入力
	 * @param[in] mps 速度[m/s]
	 */
	void SetMps(float mps){
		_motor->SetRadps(static_cast<float>(mps * 2000.0f / _diameter_mm));
	}
	/**
	 * @brief 速度取得
	 * @return 速度[m/s]
	 */
	float GetMps() const{
		return static_cast<float>(_motor->GetRadps()) * _diameter_mm / 2000.0f;
	}

	/**
	 * @brief モーターインスタンス取得
	 * @return モーターインスタンス
	 */
	const std::shared_ptr<Motor>& GetMotor() const{
		return _motor;
	}
};
}
