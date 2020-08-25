/*
 * IMU���N���X
 * ��{3�������x+�O���W���C��
 * �P�ʌn�� [mg] �� [rad/s] �� [rad]
 */
#pragma once

#include "../../Global.hpp"

namespace nut{
class IMU{
protected:
	//x,y,z,�܂���role,pitch,yaw�̏�
	Eigen::Vector3f _sensor_acc = {0.0, 0.0, 0.0};
	Eigen::Vector3f _sensor_rot = {0.0, 0.0, 0.0};
	Eigen::Vector3f _global_acc = {0.0, 0.0, 0.0};
	Eigen::Vector3f _global_rot = {0.0, 0.0, 0.0};
	Eigen::Vector3f _global_angle = {0.0, 0.0, 0.0};

public:
	IMU(){}
	virtual ~IMU(){}


	/*
	 * ������
	 */
	virtual void Init() = 0;

	/*
	 * ���Z�b�g
	 */
	virtual void Reset() = 0;


	/*
	 * getter[m/s^2]
	 */
	const Eigen::Vector3f& GetSensorAcc() const{
		return _sensor_acc;
	}
	/*
	 * getter[rad/s]
	 */
	const Eigen::Vector3f& GetSensorRot() const{
		return _sensor_rot;
	}
	/*
	 * getter[m/s^2]
	 */
	const Eigen::Vector3f& GetGlobalAcc() const{
		return _global_acc;
	}
	/*
	 * getter[rad/s]
	 */
	const Eigen::Vector3f& GetGlobalRot() const{
		return _global_rot;
	}
	/*
	 * getter[rad]
	 */
	const Eigen::Vector3f& GetGlobalAngle() const{
		return _global_angle;
	}
};
}
