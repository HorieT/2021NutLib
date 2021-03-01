/**
 * @brief ステアドライバノード
 *
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include "../CANWrapper.hpp"
#include "../CANBusProtocol.hpp"
#include <memory>
#include <limits>


namespace nut{
/*
 * @brief ステアドライバクラス
 * @details ステア用の一体型ドライバへのノード接続用
 */
class SteerDriver{
protected:
	//Period control
	TimeScheduler<void> _scheduler;

	//
	bool _is_init = false;


	/* target */
	float _target_norm = 0.0f;
	float _target_rad = 0.0f;


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() = 0;

public:
	/**
	 * @brief コンストラク
	 * @param[in] period 周期
	 */
	SteerDriver(MilliSecond<uint32_t> period)
		: _scheduler([this]{ScheduleTask();}, period){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~SteerDriver(){
	}


	/**
	 * @brief 初期化関数
	 */
	virtual void Init() = 0;

	/**
	 * @brief 非初期化関数
	 */
	virtual void Deinit() = 0;


	/**
	 * @brief 制御スタート
	 */
	virtual bool Start(bool current_control = false) = 0;

	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() = 0;


	/*
	 *  @brief 制御入力
	 *  @param[in] norm 平面ベクトルの絶対値[m/s]
	 *  @param[in] norm 平面ベクトルの角度[rad]
	 */
	virtual bool SetMove(float norm, float rad){
		_target_norm = norm;
		_target_rad = rad;

		return true;
	}
	/*
	 *  @brief 制御入力(角度保持)
	 *  @param[in] norm 平面ベクトルの絶対値[m/s]
	 */
	virtual bool SetMove(float norm){
		_target_norm = norm;
		_target_rad = std::numeric_limits<float>::infinity();

		return true;
	}
};
}
