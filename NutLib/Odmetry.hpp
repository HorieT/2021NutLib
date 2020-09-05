/*
 * �I�h���g���N���X
 * 2�G���R�[�_���W���C���݂̂ɑΉ�
 */
#pragma once

#include "Global.hpp"
#include "TimeScheduler.hpp"
#include "Coordinate.hpp"
#include "Sensor/Encoder/EncoderWheel.hpp"
#include "Sensor/IMU/IMU.hpp"
#include <array>
#include <memory>

namespace nut {
class Odmetry{
private:
	TimeScheduler<void> _scheduler;
	const std::shared_ptr<IMU> _imu;
	std::array<const std::shared_ptr<EncoderWheel>, 2> _encoder;//wheel�̕��͎��̂ł��������H

	Coordinate<float> _start_position;
	Coordinate<float> _position;
	Coordinate<float> _velocity;

	/*�v�Z���\�[�X*/
	std::array<float, 2> _encoder_radPos;//�ʒu�p
	std::array<float, 2> _encoder_norm;//���K������
	float _sin_encoder_directiion_diff;
	std::array<float, 2> _cos_encoder_direction;
	std::array<float, 2> _sin_encoder_direction;
	std::array<float, 2> _cos_encoder_rad;
	std::array<float, 2> _sin_encoder_rad;



	void CalcPos(){
		//�e�����ւ̎����l
		std::array<float, 2> distance {
			_encoder.at(0)->GetDistanceAndReset(),
			_encoder.at(1)->GetDistanceAndReset()
		};
		//�O��ʒu
		const Coordinate<float> last_pos = _position;
		//���݈ʒu����p
		Coordinate<float> now_position = _position;
		now_position.theta = _start_position.theta + _imu->GetGlobalAngle().z();
		if(fabs(now_position.theta) > static_cast<float>(M_PI))//�P�ʉ~�Ɏ��߂�
			now_position.theta += (now_position.theta > 0.0f) ? -2.0f*static_cast<float>(M_PI) : 2.0f*static_cast<float>(M_PI);

		//�P�ʎ��ԑ��x
		const float rot_vel = _imu->GetGlobalRot().z() * static_cast<float>(_scheduler.GetPeriod()) / 1000.0f;

		//�����֊e�X�̃x�N�g��
		std::array<Eigen::Vector2f, 2> distance_vec{
			Eigen::Vector2f{distance.at(0) * _cos_encoder_direction.at(0), distance.at(0) * _sin_encoder_direction.at(0)},
			Eigen::Vector2f{distance.at(1) * _cos_encoder_direction.at(1), distance.at(1) * _sin_encoder_direction.at(1)}};


		for(uint8_t i = 0;i < 2;++i){
			//�G���R�[�_��]����
			float rot_len = rot_vel * _encoder_norm.at(i);
			Eigen::Vector2f calc_vec =
					distance_vec.at(i) - Eigen::Vector2f(rot_len * -_sin_encoder_rad.at(i), rot_len * _cos_encoder_rad.at(i));
			distance.at(i) = sqrtf(calc_vec.x() * calc_vec.x() + calc_vec.y() * calc_vec.y()) * cos(_encoder.at(i)->GetPosition().theta - atan2f(calc_vec.y(), calc_vec.x()));
		}
		now_position.x +=
				(distance.at(0) * sin(_encoder.at(1)->GetPosition().theta + last_pos.theta) - distance.at(1) * sin(_encoder.at(0)->GetPosition().theta + last_pos.theta)) /
				(-_sin_encoder_directiion_diff);
		now_position.y +=
				(distance.at(0) * cos(_encoder.at(1)->GetPosition().theta + last_pos.theta) - distance.at(1) * cos(_encoder.at(0)->GetPosition().theta+ last_pos.theta)) /
				(_sin_encoder_directiion_diff);


		//���
		_position = now_position;
		_velocity = (now_position - last_pos) / static_cast<float>(_scheduler.GetPeriod()) * 1000.0f;
	}
public:
	Odmetry(uint32_t period, const std::shared_ptr<IMU>& imu, const std::shared_ptr<EncoderWheel>& enc1, const std::shared_ptr<EncoderWheel>& enc2, Coordinate<float> start) :
		_scheduler([this]{CalcPos();}, period), _imu(imu), _encoder({enc1, enc2}), _start_position(start){

		_position = _start_position;

		//�e�r���l�v�Z
		for(uint8_t i = 0;i < 2;++i){
			_encoder_radPos.at(i) = _encoder.at(i)->GetPosition().Angle();
			_encoder_norm.at(i) = _encoder.at(i)->GetPosition().Norm();
			_cos_encoder_direction.at(i) = cos(_encoder.at(i)->GetPosition().theta);
			_sin_encoder_direction.at(i) = sin(_encoder.at(i)->GetPosition().theta);
			_cos_encoder_rad.at(i) = cosf(_encoder_radPos.at(i));
			_sin_encoder_rad.at(i) = sinf(_encoder_radPos.at(i));
			_sin_encoder_directiion_diff = sin(_encoder.at(0)->GetPosition().theta - _encoder.at(1)->GetPosition().theta);
		}
	}
	virtual ~Odmetry(){}


	/*
	 * ������
	 */
	void Init(){
		_scheduler.Set();
		_imu->Init();
		_encoder.at(0)->Init();
		_encoder.at(1)->Init();
	}

	/*
	 * getter
	 */
	const Coordinate<float>& GetPosition() const{
		return _position;
	}
	const Coordinate<float>& GetVelocity() const{
		return _velocity;
	}

	/*
	 * �ʒu���Z
	 */
	void ResetPosition(const Coordinate<float>& position){
		_position = position;
	}
};
}
