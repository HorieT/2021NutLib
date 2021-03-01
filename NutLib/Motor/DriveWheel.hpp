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
	const MilliMeter<float> _diameter;
	const Coordinate<float> _position;


public:
	/**
	 * @brief コンストラクタ
	 * @param[in] motor モーターインスタンス
	 * @param[in] diameter タイヤ直径[mm]
	 * @param[in] position タイヤ位置[mm]
	 */
	DriveWheel(const std::shared_ptr<Motor>& motor, MilliMeter<float> diameter, Coordinate<float> position = Coordinate<float>())
		: _motor(motor), _diameter(diameter), _position(position){

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
		_motor->SetRadps(static_cast<float>(mps * 2000.0f / _diameter.f()));
	}
	/**
	 * @brief 速度取得
	 * @return 速度[m/s]
	 */
	float GetMps() const{
		return static_cast<float>(_motor->GetRadps()) * _diameter.f() / 2000.0f;
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
