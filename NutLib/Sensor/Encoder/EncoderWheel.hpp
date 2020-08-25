/*
 * �ʒu���t���G���R�[�_�N���X
 */
#pragma once

#include "Encoder.hpp"
#include "../../Coordinate.hpp"
#include <memory>

namespace nut{
class EncoderWheel{
private:
	std::shared_ptr<Encoder> _encoder;
	float _diameter_mm;
	Coordinate<float> _position;

public:
	EncoderWheel(const std::shared_ptr<Encoder>& encoder, float diameter_mm, Coordinate<float> position = Coordinate<float>{0.0f, 0.0f, 0.0f})
	: _encoder(encoder), _diameter_mm(diameter_mm), _position(position){}

	virtual ~EncoderWheel(){}

	void Init(){
		_encoder->Init();
	}


	std::weak_ptr<Encoder> GetEncoder() const{
		return _encoder;
	}

	/*
	 * �p�x&�������Z�b�g
	 */
	void Reset(){
		_encoder->Reset();
	}

	/*
	 * �p�x�擾
	 */
	float GetRad() const{
		return _encoder->GetRad();
	}
	/*
	 * �p�x�擾&���Z�b�g
	 */
	float GetRadAndReset(){
		return _encoder->GetRadAndReset();
	}
	/*
	 * �����擾
	 */
	float GetDistance() const {
		return _encoder->GetRad() * _diameter_mm;
	}
	/*
	 * �����擾&���Z�b�g
	 */
	float GetDistanceAndReset() {
		return _encoder->GetRadAndReset() * _diameter_mm;
	}

	const Coordinate<float> GetPosition() const{
		return _position;
	}
};
}
