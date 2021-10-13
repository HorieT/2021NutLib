/**
 * @file SteerChassisBase.hpp
 * @brief ステア足回り基幹
 * @author Horie
 * @date 2021/5
 */
#pragma once

#include "../Global.hpp"
#include "Chassis.hpp"
#include "../Motor/DriveWheel.hpp"
#include <array>
#include <Eigen/Geometry>

namespace nut{

/**
 * @brief ステア足回り基底純粋仮想クラス
 */
class SteerChassisBase : public Chassis{
public:
	enum class MoveMode : uint8_t{
		nomal = 0,
		reset,
		steerOnry,
		steerBreaking
	};


protected:
	static constexpr float RAD_DIFF_LIM = M_PI / 2.0;
	MoveMode _mode = MoveMode::nomal;


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override = 0;



public:
	/*
	 * @brief コンストラクタ
	 */
	SteerChassisBase(MilliSecond<uint32_t> period, const std::shared_ptr<Odmetry>& odmetry) :
		Chassis(period, odmetry){

	}

	/*
	 * @brief デストラクタ
	 */
	virtual ~SteerChassisBase(){

	}



	/**
	 * @brief 速度入力
	 * @param[in] velocity 速度[m/s],[rad/s]
	 * @param[in] mode 動作モード
	 * @return 速度入力可能か
	 */
	virtual bool SetVelocity(Coordinate<float> velocity, MoveMode mode = MoveMode::nomal){
		_mode = mode;
		Chassis::SetVelocity(velocity);
		return true;
	}
	/**
	 * @brief 速度入力
	 * @param[in] velocity 速度[m/s],[rad/s]
	 * @param[in] origin 速度[m/s],[rad/s]
	 * @return 速度入力可能か
	 */
	virtual bool SetVelocity(Coordinate<float> velocity, Coordinate<float> origen, MoveMode mode = MoveMode::nomal){
		_mode = mode;
		Chassis::SetVelocity(velocity, origen);
		return true;
	}
	/**
	 * @brief 速度入力
	 * @param[in] velocity_mps 速度[m/s]
	 * @param[in] rot_radps [rad/s]
	 * @param[in] mode 動作モード
	 * @return 速度入力可能か
	 */
	virtual bool SetVelocity(Eigen::Vector2f velocity_mps, float rot_radps, MoveMode mode = MoveMode::nomal){
		_mode = mode;
		Chassis::SetVelocity(velocity_mps, rot_radps);
		return true;
	}
	/**
	 * @brief 速度入力
	 * @param[in] velocity_mps 速度[m/s]
	 * @param[in] rot_radps [rad/s]
	 * @param[in] mode 動作モード
	 * @return 速度入力可能か
	 */
	virtual bool SetVelocity(Eigen::Vector2f velocity_mps, float rot_radps, Coordinate<float> origen, MoveMode mode = MoveMode::nomal){
		_mode = mode;
		Chassis::SetVelocity(velocity_mps, rot_radps, origen);
		return true;
	}
};
}
