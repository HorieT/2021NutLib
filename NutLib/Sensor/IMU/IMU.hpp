/*
 * IMU基底クラス
 * 基本3軸加速度+三軸ジャイロ
 * 単位系は [mg] と [rad/s] と [rad]
 */
#pragma once

#include "../../Global.hpp"

namespace nut{
class IMU{
protected:
	//x,y,z,またはrole,pitch,yawの順
	Eigen::Vector3f _sensor_acc = {0.0, 0.0, 0.0};
	Eigen::Vector3f _sensor_rot = {0.0, 0.0, 0.0};
	Eigen::Vector3f _global_acc = {0.0, 0.0, 0.0};
	Eigen::Vector3f _global_rot = {0.0, 0.0, 0.0};
	Eigen::Vector3f _global_angle = {0.0, 0.0, 0.0};

public:
	IMU(){}
	virtual ~IMU(){}


	/*
	 * 初期化
	 */
	virtual void Init() = 0;

	/*
	 * リセット
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
