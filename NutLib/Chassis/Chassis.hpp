/*
 * �������N���X
 * �Ƃ肠�������x���͂̂�
 * ���Ȉʒu�ƕ������Ă邩�瑊�΍��W���͂̂�
 */
#pragma once

#include "../Global.hpp"
#include "../Coordinate.hpp"
#include "../TimeScheduler.hpp"

namespace nut{
class Chassis{
protected:
	//�g���ɔ����ė�
	enum class MoveType : uint8_t{
		stop = 0U,
		velocity
	};
	MoveType _move_type = MoveType::stop;
	Coordinate<float> _target_velocity = {0f};

	/*
	 * ���x����A�N�`���G�[�^�𑀍삷��֐�
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
