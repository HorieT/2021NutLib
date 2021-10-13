/**
 * @file SteerChassis.hpp
 * @brief 操舵と駆動が独立したステア足周り
 * @author Horie
 * @date 2021/3
 */
#pragma once

#include "../Global.hpp"
#include "SteerChassisBase.hpp"
#include "../Motor/DriveWheel.hpp"
#include <array>
#include <Eigen/Geometry>

namespace nut{

/**
 *　@brief 操舵と駆動が独立の基本的なステア足回り
 */
template<uint8_t N>
class SteerChassis : public SteerChassisBase{
private:
	const std::array<std::shared_ptr<DriveWheel>, N> _wheel;//!< 駆動輪
	const std::array<std::shared_ptr<Motor>, N> _steering;//<! 操舵


	std::array<float, N> _wheel_cos = {0.0};
	std::array<float, N> _wheel_sin = {0.0};
	std::array<Meter<float>, N> _wheel_length = {0.0};



	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		std::array<Eigen::Vector2f, N> input;

		//move
		if((_target_velocity.x() != 0.0) || (_target_velocity.y() != 0.0) || (_target_velocity.theta() != 0.0) || (_mode == MoveMode::steerBreaking)){
			uint8_t i = 0;

			if(_mode != MoveMode::steerBreaking){
				for(auto& in : input){
					in <<
							_target_velocity.x() - _wheel_length[i] * _target_velocity.theta() * _wheel_sin[i],
							_target_velocity.y() + _wheel_length[i] * _target_velocity.theta() * _wheel_cos[i];
					in = Eigen::Rotation2Df(-_wheel[i]->GetPos().theta()) * in;
					++i;
				}
			}


			i = 0;
			for(auto& w : _wheel){
				int8_t coff = 1;
				float rad =
						(_mode == MoveMode::reset) ? 0.0
								: ((_mode == MoveMode::steerBreaking) ? ((w->GetPos().Angle() > 0) ? w->GetPos().Angle() - M_PI / 2.0 : w->GetPos().Angle() + M_PI / 2.0)
										: std::atan2(input[i].y(), input[i].x()));


				if(_mode == MoveMode::nomal || _mode == MoveMode::steerOnry){
					float rad_diff = std::fmod(rad - _steering[i]->GetTagRad(), M_PI * 2.0);


					if(std::abs(rad_diff) > RAD_DIFF_LIM){//over rad
						//coff = -1;
						rad_diff = (M_PI * (rad_diff < 0 ? 1.0 : -1.0)) + std::fmod(rad_diff, M_PI);

					}


					rad = _steering[i]->GetTagRad() + rad_diff;
				}

				/* input */
				if(_mode == MoveMode::nomal)w->SetMps(coff * input[i].norm());
				else w->SetMps(0.0);

				_steering[i]->SetRadSingle(rad, 10.0);
				++i;
			}
			return;
		}
		else {// no move

			uint8_t i = 0;
			for(auto& w : _wheel){
				w->GetMotor()->SetDuty(0.0);
				_steering[i]->SetDuty(0.0);
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
			: SteerChassisBase(period, odmetry),
			  _wheel(wheel),
			  _steering(steering){

		uint8_t i = 0;
		for(auto& w_cos : _wheel_cos){
			w_cos = cos(_wheel[i]->GetPos().Angle());
			++i;
		}
		i = 0;
		for(auto& w_sin : _wheel_sin){
			w_sin = sin(_wheel[i]->GetPos().Angle());
			++i;
		}
		i = 0;
		for(auto& w_length : _wheel_length){
			w_length = _wheel[i]->GetPos().Norm();
			++i;
		}
	}

	virtual ~SteerChassis(){

	}
};
}
