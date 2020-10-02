/**
 *
 */
#pragma once

#include "../Global.hpp"
#include "Chassis.hpp"
#include "../Motor/DriveWheel.hpp"
#include <array>
#include <Eigen/Geometry>

namespace nut{
/**
 *
 */
template<uint8_t N>
class SteerChassis : public Chassis{
private:
	const std::array<std::shared_ptr<DriveWheel>, N> _wheel;//!< 駆動輪
	const std::array<std::shared_ptr<Motor>, N> _steering;//<! 操舵


	const std::array<const float, N> _wheel_cos = {0.0};
	const std::array<const float, N> _wheel_sin = {0.0};
	const std::array<const float, N> _wheel_length = {0.0};

	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		std::array<Eigen::Vector2f, N> input;

		if((_target_velocity.x() == 0.0) && (_target_velocity.y() == 0.0) && (_target_velocity.theta() == 0.0)){
			{
				uint8_t i = 0;
				for(auto& w : _wheel){
					w->GetMotor()->SetDuty(0.0);
					_steering[i]->SetDuty(0.0);
					++i;
				}
			}
			return;
		}


		{
		uint8_t i = 0;
		for(auto& in : input){
				in <<
						_target_velocity.x() - _wheel_length[i] * _target_velocity.theta() * _wheel_sin[i],
						_target_velocity.y() + _wheel_length[i] * _target_velocity.theta() * _wheel_cos[i];
				in = Eigen::Rotation2Df(-_wheel[i]->GetPos().theta()) * in;
				++i;
		}
		}
		{
			uint8_t i = 0;
			for(auto& w : _wheel){
				int8_t flag = 1;
				float rad = std::atan2(input[i].y(), input[i].x());
				if(fabs(rad) > M_PI /2){
					input[i] = Eigen::Rotation2Df(M_PI) * input[i];
					rad = std::atan2(input[i].y(), input[i].x());

					flag = -1;
					if(fabs(rad) > M_PI /2){
						w->GetMotor()->SetDuty(0.0);
						_steering[i]->SetDuty(0.0);
						++i;
					continue;
					}
				}
				w->SetMps(flag * input[i].norm());//応急処置
				_steering[i]->SetRad(rad, 10.0);
				++i;
			}
		}
	}


public:
	SteerChassis(
		uint32_t period,
		const std::shared_ptr<Odmetry>& odmetry,
		std::array<std::shared_ptr<DriveWheel>, N> wheel,
		std::array<std::shared_ptr<Motor>, N> steering)
			: Chassis(period, odmetry),
			  _wheel(wheel),
			  _steering(steering){

		uint8_t i = 0;
		for(auto& w_cos : const_cast<std::array<const float, N>&>(_wheel_cos)){
			const_cast<float&>(w_cos) = std::cos(_wheel[i]->GetPos().Angle());
			++i;
		}
		i = 0;
		for(auto& w_sin : const_cast<std::array<const float, N>&>(_wheel_sin)){
			const_cast<float&>(w_sin) = std::sin(_wheel[i]->GetPos().Angle());
			++i;
		}
		i = 0;
		for(auto& w_length : const_cast<std::array<const float, N>&>(_wheel_length)){
			const_cast<float&>(w_length) = _wheel[i]->GetPos().Norm();
			++i;
		}
	}

	virtual ~SteerChassis(){

	}
};
}
