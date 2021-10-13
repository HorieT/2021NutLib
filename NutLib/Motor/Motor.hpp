/**
 * @file Motor.hpp
 * @brief モータ制御基底
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include "../Unit/UnitCore.hpp"
#include "../ControlSystem/PID/VecPID.hpp"
#include "../ControlSystem/PID/PosPID.hpp"
#include <array>
#include <memory>

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
		currnet = 0x80,//special　expansion
	};
	MoveType _move_type = MoveType::stop;

	//Period control
	TimeScheduler<void> _scheduler;

	//Controller
	std::unique_ptr<PIDBase<float>> _radps_pid{new PosPID<float>()};
	std::unique_ptr<PIDBase<float>> _rad_pid{new PosPID<float>()};
	std::unique_ptr<PIDBase<float>> _current_pid{new PosPID<float>()};

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

	bool _is_init = false;



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
	virtual inline void ResetController(){
		if(_move_type == MoveType::stop) return;
		_radps_pid->Reset();
		_rad_pid->Reset();
		_current_pid->Reset();
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
	Motor(MilliSecond<uint32_t> period) : _scheduler([this]{ScheduleTask();}, period){

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
	 * @brief 非初期化関数
	 */
	virtual void Deinit() = 0;


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
	 * @param[in] radps rad/s
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
	virtual bool SetRadMulti(Radian<float> rad){
		if(_move_type == MoveType::stop) return false;
		_target_rad = rad.f();
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
	virtual bool SetRadSingle(Radian<float> rad){
		if(_move_type == MoveType::stop) return false;
		if(abs(rad) > M_PI_f)return false;

		_target_rad = _now_rad + NormalizeRadian(rad - _now_rad).f();
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
	[[deprecated("This function is not yet in place.")]]
	virtual bool SetRadSingle(Radian<float> rad, bool polarity){
		if(_move_type == MoveType::stop) return false;
		if(abs(rad) > M_PI_f)return false;

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
	 * @brief 電流制御
	 * @param[in] currnet 電流[A]
	 * @return 電流制御可能かどうか
	 */
	virtual bool SetCurrent(Ampere<float> currnet){
		if(_move_type == MoveType::stop) return false;
		_target_current = currnet.f();
		_move_type = MoveType::currnet;
		return true;
	}


	/* setter */

	/**
	 * @brief 角度原点リセット
	 * @details 現在角を書き換えます
	 * @param[in] rad 書き換え角度
	 * @return 角度原点リセット可能かどうか
	 */
	virtual bool ResetRadOrigin(Radian<float> rad) {
		if(_move_type != MoveType::stop)return false;
		_now_rad = rad.f();
		return true;
	}


	/**
	 * @brief 角速度PID制御器セット
	 * @param[in] controller PID制御器
	 * @return セットの可否
	 */
	virtual bool SetRadpsPID(std::unique_ptr<PIDBase<float>>&& controller){
		if(_move_type != MoveType::stop || !controller)return false;
		_radps_pid = std::move(controller);
		return true;
	}
	/**
	 * @brief 角度PID制御器セット
	 * @param[in] controller PID制御器
	 * @return セットの可否
	 */
	virtual bool SetRadPID(std::unique_ptr<PIDBase<float>>&& controller){
		if(_move_type != MoveType::stop || !controller)return false;
		_rad_pid = std::move(controller);
		return true;
	}
	/**
	 * @brief 電流PID制御器セット
	 * @param[in] controller PID制御器
	 * @return セットの可否
	 */
	virtual bool SetCurrentPID(std::unique_ptr<PIDBase<float>>&& controller){
		if(_move_type != MoveType::stop || !controller)return false;
		_current_pid = std::move(controller);
		return true;
	}


	/* getter */

	/**
	 * @brief 角速度PID制御器取得
	 * @return 制御器生ポインタ
	 * @details Stop()していない場合はヌルポ
	 */
	virtual PIDBase<float>* GetRadpsPID(){
		if(_move_type != MoveType::stop)return nullptr;
		return _radps_pid.get();
	}
	/**
	 * @brief 角度PID制御器取得
	 * @return 制御器生ポインタ
	 * @details Stop()していない場合はヌルポ
	 */
	virtual PIDBase<float>* GetRadPID(){
		if(_move_type != MoveType::stop)return nullptr;
		return _rad_pid.get();
	}
	/**
	 * @brief 電流PID制御器取得
	 * @return 制御器生ポインタ
	 * @details Stop()していない場合はヌルポ
	 */
	virtual PIDBase<float>* GetCurrentPID(){
		if(_move_type != MoveType::stop)return nullptr;
		return _current_pid.get();
	}






	/**
	 * @brief Duty取得
	 * @return Duty比(百分率)
	 */
	virtual float GetDuty() const{
		return _now_duty;
	}
	/**
	 * @brief 速度取得
	 * @return rad/s
	 */
	virtual float GetRadps() const{
		return _now_radps;
	}
	/**
	 * @brief 角度取得
	 * @return Rad
	 */
	virtual Radian<float> GetRad()const{
		return _now_rad;
	}
	/**
	 * @brief 現在電流値取得
	 * @return [A]
	 */
	virtual Ampere<float> GetCurrent()const{
		return _now_current;
	}
	/**
	 * @brief 目標速度取得
	 * @return rad/s
	 */
	virtual float GetTagRadps() const{
		return _target_radps;
	}
	/**
	 * @brief 目標角度取得
	 * @return Rad
	 */
	virtual Radian<float> GetTagRad()const{
		return _target_rad;
	}
	/**
	 * @brief 目標電流値取得
	 * @return [A]
	 */
	virtual Ampere<float> GetTagCurrent()const{
		return _target_current;
	}
	bool IsStart(){
		return _move_type != MoveType::stop;
	}
};
}
