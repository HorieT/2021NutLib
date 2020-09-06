/**
 * @file IMU.hpp
 * @brief IMU基底
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "../../Global.hpp"

namespace nut{
/**
 * @brief IMU基底純粋仮想クラス
 * @details 3軸加速度+3軸ジャイロまで基底でサポート<br>
 * 今後クウォータニオン導入予定
 */
class IMU{
protected:
	Eigen::Vector3f _sensor_acc = {0.0, 0.0, 0.0};//!< x,y,zの加速度(センサの生値)
	Eigen::Vector3f _sensor_rot = {0.0, 0.0, 0.0};//!< role,pitch,yawの角速度(センサの生値)
	Eigen::Vector3f _global_acc = {0.0, 0.0, 0.0};//!< x,y,zの加速度(グローバル座標)
	Eigen::Vector3f _global_rot = {0.0, 0.0, 0.0};//!< role,pitch,yawの角速度(グローバル座標)
	Eigen::Vector3f _global_angle = {0.0, 0.0, 0.0};//!< role,pitch,yawの角度(グローバル座標)

public:
	/**
	 * @brief コンストラクタ
	 */
	IMU(){}
	/**
	 * @brief デストラクタ
	 */
	virtual ~IMU(){}


	/**
	 * @brief 初期化関数
	 */
	virtual void Init() = 0;

	/**
	 * @brief IMUリセット
	 */
	virtual void Reset() = 0;


	/**
	 * getter[m/s^2]
	 */
	const Eigen::Vector3f& GetSensorAcc() const{
		return _sensor_acc;
	}
	/**
	 * getter[rad/s]
	 */
	const Eigen::Vector3f& GetSensorRot() const{
		return _sensor_rot;
	}
	/**
	 * getter[m/s^2]
	 */
	const Eigen::Vector3f& GetGlobalAcc() const{
		return _global_acc;
	}
	/**
	 * getter[rad/s]
	 */
	const Eigen::Vector3f& GetGlobalRot() const{
		return _global_rot;
	}
	/**
	 * getter[rad]
	 */
	const Eigen::Vector3f& GetGlobalAngle() const{
		return _global_angle;
	}
};
}
