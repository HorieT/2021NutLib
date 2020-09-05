/*
 * �I���j�z�C�[���N���X
 * �ԗ֐�T�̑����
 * �ԗւ͓��p�z�u����Ă���O��
 */
#pragma once

#include "../Global.hpp"
#include "../Coordinate.hpp"
#include "../Odmetry.hpp"
#include "Chassis.hpp"
#include "../Motor/DriveWheel.hpp"
#include <memory>


namespace nut{
template<uint8_t T>
class OmniChassis : public Chassis{
	static_assert(T > 1U, "?????????????");

protected:
	std::array<std::shared_ptr<DriveWheel>, T> _wheel;//�e�z�C�[��
	const Coordinate<float> _wheel_position;//�z�C�[���ʒu
	const bool _wheel_polarity;//�z�C�[���ɐ�

	/*�v�Z���\�[�X*/
	const float _wheel_length = 0;
	std::array<std::array<float, 2>, T> _coefficient{{0.0f}};


	virtual void InputVelocity() override{
		std::array<float, T> input;
		float rot_component = _wheel_length * _target_velocity.theta;
		//��r�񐔑����邯�ǎO�����Z�q�̕������������H
		if(_wheel_polarity){
			uint8_t i = 0;
			for(auto& in : input){
				in = _target_velocity.x * _coefficient[i][0] + _target_velocity.y * _coefficient[i][1] + rot_component;
				++i;
			}

		}else{
			uint8_t i = 0;
			for(auto& in : input){
				in = -_target_velocity.x * _coefficient[i][0] - _target_velocity.y * _coefficient[i][1] - rot_component;
				++i;
			}

		}
		{
			uint8_t i = 0;
			for(auto& w : _wheel){
				w->set_mps(input[i]);
				++i;
			}
		}
	}

public:
	OmniChassis(
			uint32_t period,
			std::shared_ptr<Odmetry> odmetry,
			std::array<std::shared_ptr<DriveWheel>, T> wheel,
			Coordinate<float> first_wheel_position,
			bool wheel_polarity = true)
				: Chassis(period, odmetry), _wheel(wheel), _wheel_position(first_wheel_position), _wheel_polarity(wheel_polarity)
	{
		const_cast<float&>(_wheel_length) = _wheel_position.Norm() * 0.001;

		uint8_t i = 0;
		for(auto& c : _coefficient){
			c[0] = -std::cos(_wheel_position.theta + static_cast<float>(M_PI) * 2.0f * static_cast<float>(i) / static_cast<float>(T));
			c[1] = -std::sin(_wheel_position.theta + static_cast<float>(M_PI) * 2.0f * static_cast<float>(i)/ static_cast<float>(T));
			++i;
		}
	}
	virtual ~OmniChassis(){


	}
};
}
