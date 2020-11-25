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


namespace nut{
/*
 * @brief ステアドライバクラス
 * @details ステア用の一体型ドライバへのノード接続用
 */
class SteerDriver{
protected:
	//Period control
	TimeScheduler<void> _scheduler;

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
	 * @param[in] can canのヘルパインスタンス
	 * @param[in] id 5bitのモータid
	 */
	SteerDriver(uint32_t period)
		: _scheduler([this]{ScheduleTask();}, period){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~SteerDriver(){
	}




	/**
	 * @brief 制御スタート
	 */
	virtual bool Start(bool current_control = false) = 0;

	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() = 0;


	virtual bool SetMove(float norm, float rad){
		_target_norm = norm;
		_target_rad = rad;

		return true;
	}
};
}
