/**
 * @file Motor.hpp
 * @brief モータ制御基底
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include "../PID.hpp"
#include <array>

namespace nut{
/**
 * @brief モータ制御基底純粋仮想クラス
 */
class Motor{
protected:
	/**
	 * @brief モータ制御モード
	 */
	enum class MoveType : uint8_t{
		stop = 0U,
		duty,
		radps,
		radMulti,
		radSingle,
		radSinglePolarity,
		radpsCurrent,
		radMultiCurrent,
		radSingleCurrent,
		radSinglePolarityCurrent,
	};
	MoveType _move_type = MoveType::stop;

	//Period control
	TimeScheduler<void> _scheduler;

	//Controller
	PID<float> _radps_pid;
	PID<float> _rad_pid;
	PID<float> _current_pid;

	//Target value
	float _target_duty = 0.0f;//Percentage
	float _target_radps = 0.0f;
	float _target_rad = 0.0f;//Multi-turn abs
	float _target_current = 0.0f;
	bool _turn_polarity = true;


	//Present value
	float _now_duty = 0.0f;//Percentage
	float _now_radps = 0.0f;
	float _now_rad = 0.0f;//Multi-turn abs
	float _now_current = 0.0f;



	/**
	 * @brief 目標値リセット
	 */
	virtual inline void ResetTarget(){
		_target_duty = 0.0f;
		_target_radps = 0.0f;
		_target_rad = 0.0f;
		_target_current = 0.0f;
	}
	/**
	 * @brief パラメータ&目標値リセット
	 */
	virtual inline void ResetParam(){
		_move_type = MoveType::stop;
		ResetTarget();
		_radps_pid.Reset();
		_rad_pid.Reset();
		_current_pid.Reset();
		//_now_radps = 0.0f;
		//_now_rad = 0.0f;
	}


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() = 0;
public:
	/**
	 * @brief コンストラクタ
	 * @param[in] period 制御周期[ms]
	 */
	Motor(uint32_t period) : _scheduler([this]{ScheduleTask();}, period){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~Motor(){}


	/**
	 * @brief 初期化関数
	 */
	virtual void Init() = 0;


	/**
	 * @brief 制御スタート
	 */
	virtual bool Start() = 0;

	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() = 0;



	/**
	 * @brief Duty制御
	 * @param[in] duty 百分率
	 * @return Duty制御可能かどうか
	 */
	virtual bool SetDuty(float duty) {
		if(_move_type == MoveType::stop) return false;
		if(std::fabs(duty) > 100.0f)return false;
		_target_duty = duty;
		_move_type = MoveType::duty;
		return true;
	}
	/**
	 * @brief 速度制御
	 * @param[in] rpm RPM
	 * @return 速度制御可能かどうか
	 */
	virtual bool SetRadps(float radps) {
		if(_move_type == MoveType::stop) return false;
		_target_radps = radps;
		_move_type = MoveType::radps;
		return true;
	}
	/**
	 * @brief 多回転角度制御
	 * @param[in] rad 角度[rad]
	 * @return 角度制御可能かどうか
	 */
	virtual bool SetRadMulti(float rad){
		if(_move_type == MoveType::stop) return false;
		_target_rad = rad;
		_move_type = MoveType::radMulti;
		return true;
	}

	/**
	 * @brief 単回転角度制御
	 * @details 近い方向に回転します
	 * @param[in] rad 角度[rad]
	 * @details M_PI ~ -M_PIまで
	 * @return 角度制御可能かどうか
	 */
	virtual bool SetRadSingle(float rad){
		if(_move_type == MoveType::stop) return false;
		if(std::fabs(rad) > static_cast<float>(M_PI))return false;

		float rad_diff = std::fmod(rad - _now_rad, M_PI*2.0);
		if(std::abs(rad_diff) > M_PI)//over rad
			rad_diff = (rad_diff < 0 ? -M_PI : M_PI) - rad_diff;

		_target_rad = _now_rad + rad_diff;
		_move_type = MoveType::radSingle;
		return true;
	}


	/**
	 * @brief 単回転角度制御
	 * @details 指示極性方向に回転します.
	 * @param[in] rad 角度[rad]
	 * @param[in] polarity 極性
	 * @details trueなら正,falsなら負です
	 * @return 角度制御可能かどうか
	 */
	virtual bool SetRadSingle(float rad, bool polarity){
		if(_move_type == MoveType::stop) return false;
		if(std::fabs(rad) > static_cast<float>(M_PI))return false;

		/* not yet!!!!! */
/*
		float rad_diff = std::fmod(rad - _now_rad, M_PI*2.0);
		if(std::abs(rad_diff) > M_PI)//over rad
			rad_diff = (rad_diff < 0 ? -M_PI : M_PI) - rad_diff;

		_target_rad = _now_rad + rad_diff;
		_move_type = MoveType::radSinglePolarity;
		_turn_polarity = ???//
		return true;
		*/

		return false;
	}


	/**
	 * @brief 速度制御ゲインセット
	 * @param[in] kp Pゲイン
	 * @param[in] ki Iゲイン
	 * @param[in] kd Dゲイン
	 * @return 速度PID可能かどうか
	 */
	virtual bool SetRadpsPID(float kp, float ki, float kd, float op_limit = infinityf(), float i_limit = infinityf()) {
		if(_move_type != MoveType::stop)return false;
		_radps_pid.SetGaine(kp, ki, kd);
		_radps_pid.SetLimit(op_limit);
		_radps_pid.SetLimitI(i_limit);
		return true;
	}
	/**
	 * @brief 角度制御ゲインセット
	 * @param[in] kp Pゲイン
	 * @param[in] ki Iゲイン
	 * @param[in] kd Dゲイン
	 * @return 角度PID可能かどうか
	 */
	virtual bool SetRadPID(float kp, float ki, float kd, float op_limit = infinityf(), float i_limit = infinityf()) {
		if(_move_type != MoveType::stop)return false;
		_rad_pid.SetGaine(kp, ki, kd);
		_rad_pid.SetLimit(op_limit);
		_rad_pid.SetLimitI(i_limit);
		return true;
	}



	/**
	 * @brief 角度原点リセット
	 * @return 角度原点リセット可能かどうか
	 */
	virtual bool ResetRadOrigin(float rad) = 0;

	/**
	 * @brief Duty取得
	 * @return Duty比(百分率)
	 */
	virtual float GetDuty() const{
		return _now_duty;
	}
	/**
	 * @brief 速度取得
	 * @return RPM
	 */
	virtual float GetRadps() const{
		return _now_radps;
	}
	/**
	 * @brief 角度取得
	 * @return Rad
	 */
	virtual float GetRad()const{
		return _now_rad;
	}

	/**
	 * @brief 角度取得
	 * @return Rad
	 */
	virtual float GetTagRad()const{
		return _target_rad;
	}
	/*
	virtual const std::array<float, 3>& GetRPMPID() const{
		return _rpm_pid;
	}
	virtual const std::array<float, 3>& GetRadPID() const{
		return _rad_pid;
	}
	*/
};
}
