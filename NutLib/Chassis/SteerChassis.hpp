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
public:
	enum class MoveMode : uint8_t{
		nomal = 0,
		reset,
		steerOnry,
		steerBreaking
	};

private:
	static constexpr float RAD_DIFF_LIM = M_PI / 2.0;

	const std::array<std::shared_ptr<DriveWheel>, N> _wheel;//!< 駆動輪
	const std::array<std::shared_ptr<Motor>, N> _steering;//<! 操舵


	const std::array<const float, N> _wheel_cos = {0.0};
	const std::array<const float, N> _wheel_sin = {0.0};
	const std::array<const float, N> _wheel_length = {0.0};

	MoveMode _mode = MoveMode::nomal;
	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		std::array<Eigen::Vector2f, N> input;

		//move
		if((_target_velocity.x() != 0.0) || (_target_velocity.y() != 0.0) || (_target_velocity.theta() != 0.0)){
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

				_steering[i]->SetRad(rad, 10.0);
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





	/**
	 * @brief 速度入力
	 * @param[in] velocity 速度[m/s],[rad/s]
	 * @return 速度入力可能か
	 */
	virtual bool SetVelocity(Coordinate<float> velocity, MoveMode mode){
		_mode = mode;
		_target_velocity = velocity;
		return true;
	}

	/**
	 * @brief 速度入力
	 * @param[in] velocity_mps 速度[m/s]
	 * @param[in] rot_radps [rad/s]
	 * @return 速度入力可能か
	 */
	virtual bool SetVelocity(Eigen::Vector2f velocity_mps, float rot_radps, MoveMode mode){
		_mode = mode;
		_target_velocity.x() = velocity_mps.x();
		_target_velocity.y() = velocity_mps.y();
		_target_velocity.theta() = rot_radps;
		return true;
	}
};
}
