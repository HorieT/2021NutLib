/**
 *
 */
#pragma once

#include "../Global.hpp"
#include "SteerChassis.hpp"
#include "../Motor/SteerDriver.hpp"
#include <array>
#include <limits>
#include <Eigen/Geometry>

namespace nut{
/**
 *
 */
template<uint8_t N>
class SteerChassisSp : public Chassis{
private:
	using MoveMode = SteerOpration::MoveMode;
	static constexpr float RAD_DIFF_LIM = M_PI / 2.0;

	const std::array<std::shared_ptr<SteerDriver>, N> _steering;
	const std::array<Coordinate<float>, N> _wheel_position;//!< 駆動輪位置
	const float _diameter_mm;


	const std::array<const float, N> _wheel_cos = {0.0};
	const std::array<const float, N> _wheel_sin = {0.0};
	const std::array<const float, N> _wheel_length = {0.0};

	MoveMode _mode = MoveMode::nomal;
	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		std::array<Eigen::Vector2f, N> input;

		if(_mode == MoveMode::reset){
			for(auto& st : _steering){
				st->SetMove(0.0, std::numeric_limits<float>::quiet_NaN());
			}
			return;
		}

		//move
		if((_target_velocity.x() != 0.0) || (_target_velocity.y() != 0.0) || (_target_velocity.theta() != 0.0)){
			uint8_t i = 0;

			if(_mode != MoveMode::steerBreaking){
				for(auto& in : input){
					in <<
							_target_velocity.x() - _wheel_length[i] * _target_velocity.theta() * _wheel_sin[i],
							_target_velocity.y() + _wheel_length[i] * _target_velocity.theta() * _wheel_cos[i];
					in = Eigen::Rotation2Df(-_wheel_position[i].theta()) * in;
					++i;
				}
			}


			i = 0;
			for(auto& st : _steering){
				int8_t coff = 1;
				float rad =
						(_mode == MoveMode::reset) ? 0.0
								: ((_mode == MoveMode::steerBreaking) ? _wheel_position[i].Angle()
										: std::atan2(input[i].y(), input[i].x()));


				/* input */
				st->SetMove(_mode == MoveMode::nomal ? coff * input[i].norm() * 2000.0f / _diameter_mm : 0.0, rad);
				++i;
			}
			return;
		}
		else {// no move
		}
	}


public:
	SteerChassisSp(
		uint32_t period,
		const std::shared_ptr<Odmetry>& odmetry,
		std::array<std::shared_ptr<SteerDriver>, N> steer,
		std::array<Coordinate<float>, N> steer_pos,
		float diameter_mm)
			: Chassis(period, odmetry),
			  _steering(steer),
			  _wheel_position(steer_pos),
			  _diameter_mm(diameter_mm){

		uint8_t i = 0;
		for(auto& w_cos : const_cast<std::array<const float, N>&>(_wheel_cos)){
			const_cast<float&>(w_cos) = std::cos(_wheel_position[i].Angle());
			++i;
		}
		i = 0;
		for(auto& w_sin : const_cast<std::array<const float, N>&>(_wheel_sin)){
			const_cast<float&>(w_sin) = std::sin(_wheel_position[i].Angle());
			++i;
		}
		i = 0;
		for(auto& w_length : const_cast<std::array<const float, N>&>(_wheel_length)){
			const_cast<float&>(w_length) = _wheel_position[i].Norm();
			++i;
		}
	}

	virtual ~SteerChassisSp(){

	}





	/**
	 * @brief 速度入力
	 * @param[in] velocity 速度[m/s],[rad/s]
	 * @param[in] mode 動作モード
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
	 * @param[in] mode 動作モード
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
