/*
 * ���[�^���[�G���R�[�_���N���X
 * �^�C���A�܂��̓M���̃T�C�Y���ƕ
 */
#pragma once

#include "../../Global.hpp"


namespace nut{
class Encoder{
protected:
	const uint32_t _resolution;

public:
	/*
	 * �R���X�g���N�^�A�f�X�g���N�^
	 */
	Encoder(uint32_t resolution): _resolution(resolution){}
	virtual ~Encoder(){}

	/*
	 * ������
	 */
	virtual void Init() = 0;


	/*
	 * �p�x���Z�b�g
	 */
	virtual void Reset() = 0;

	/*
	 * �p�x�擾
	 */
	virtual float GetRad() const = 0;
	/*
	 * �p�x�擾&���Z�b�g
	 * �C���N�������g�^�Ȃ炱����̕�����萳�m
	 */
	virtual float GetRadAndReset() = 0;
};
}
