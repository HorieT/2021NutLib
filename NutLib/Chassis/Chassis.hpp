/*
 * 足回り基底クラス
 * とりあえず速度入力のみ
 * 自己位置と分離してるから相対座標入力のみ
 */
#pragma once

#include "../Global.hpp"
#include "../Coordinate.hpp"
#include "../TimeScheduler.hpp"

namespace nut{
class Chassis{
protected:
	//拡張に備えて列挙
	enum class MoveType : uint8_t{
		stop = 0U,
		velocity
	};
	MoveType _move_type = MoveType::stop;
	Coordinate<float> _target_velocity = {0f};

	/*
	 * 速度からアクチュエータを操作する関数
	 */
	virtual void InputVelocity() = 0;


public:
	Chassis(){}
	virtual ~Chassis(){}


	virtual bool SetVelocity(Coordinate<float> velocity){
		_target_velocity = velocity;
		return true;
	}
	virtual bool SetVelocity(Eigen::Vector2f velocity_mps, float rot_radps){
		_target_velocity.x = velocity_mps.x();
		_target_velocity.y = velocity_mps.y();
		_target_velocity.theta = rot_radps;
		return true;
	}

	virtual const Coordinate<float> GetVelocity() const{
		return _target_velocity;
	}
};
}
