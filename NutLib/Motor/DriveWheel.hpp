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


public:
	/**
	 * @brief コンストラクタ
	 * @param[in] motor モーターインスタンス
	 * @param[in] diameter_mm タイヤ直径[mm]
	 */
	DriveWheel(const std::shared_ptr<Motor>& motor, float diameter_mm) : _motor(motor), _diameter_mm(diameter_mm){

	}
	/**
	 * @brief デストラクタ
	 */
	~DriveWheel(){

	}

	/**
	 * @brief 速度入力
	 * @param[in] mps 速度[m/s]
	 */
	void SetMps(float mps){
		_motor->SetRPM(static_cast<float>(mps * 60000.0f / (_diameter_mm * static_cast<float>(M_PI))));
	}
	/**
	 * @brief 速度取得
	 * @return 速度[m/s]
	 */
	float GetMps() const{
		return static_cast<float>(_motor->GetRPM()) / 60.0f * _diameter_mm * static_cast<float>(M_PI);
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
