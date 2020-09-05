/*
 * モーターを用いたホイールのクラス
 */
#pragma once

#include "../Global.hpp"
#include "Motor.hpp"
#include <memory>


namespace nut{
class DriveWheel{
protected:
	const std::shared_ptr<Motor> _motor;
	float _diameter_mm;


public:
	DriveWheel(std::shared_ptr<Motor> motor, float diameter_mm) : _motor(motor), _diameter_mm(diameter_mm){}
	~DriveWheel(){}

	void set_mps(float mps){
		_motor->SetRPM(static_cast<float>(mps * 60000.0f / (_diameter_mm * static_cast<float>(M_PI))));
	}
	float get_mps() const{
		return static_cast<float>(_motor->GetRPM()) / 60.0f * _diameter_mm * static_cast<float>(M_PI);
	}
	const std::shared_ptr<Motor>& GetMotor() const{
		return _motor;
	}
};
}
