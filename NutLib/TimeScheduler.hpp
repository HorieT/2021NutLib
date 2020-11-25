/**
 * @file TimeScheduler.hpp
 * @brief 時間制御
 * @details systickベース
 * @author Horie
 * @date 2020/10
 * @attention <font color='red'>Callback関数の実行時間は1ms以下にしてください<\font>
 */
#pragma once

#include "Global.hpp"
#include <functional>
#include <vector>
#include <memory>

/* nutlib での割込み定義 */
#ifdef USE_NUTLIB_CALLBACKS
#include "HALCallbacks/HALCallbacks.hpp"
#endif


namespace nut{
/**
 * @brief  時間制御基底純粋仮想クラス
 */
class TimeSchedulerBase{
private:
	static inline volatile uint32_t _time = 0;
	static inline std::vector<TimeSchedulerBase*> _scheduler;

#ifdef HAL_TIM_MODULE_ENABLED
	static inline TIM_HandleTypeDef* _auxiliary_timer = nullptr;
#endif

	uint32_t _start_time = 0;
	uint32_t _period;
	bool _setting = false;


	static inline void IncTime(){_time++;}

protected:
	const bool _one_time;//!< 一時オブジェクト用

	/**
	 * @brief スケジューラのセット
	 */
	void SetSchedule(){
		if(_setting)return;
		_setting = true;
		_scheduler.push_back(this);
		_start_time = _time;
	}
	/**
	 * @brief 周期呼び出しされる関数
	 */
	virtual void Callback() = 0;


public:
	/**
	 * @brief コンストラクタ
	 * @param[in] ms スケジューラ周期
	 */
	TimeSchedulerBase(uint32_t ms) : _period(ms), _one_time(false){}

	/**
	 * @brief デストラクタ
	 */
	virtual ~TimeSchedulerBase(){Erase();}

	/**
	 * @brief スケジューラ削除
	 */
	virtual void Erase() final{
		if(!_setting)return;
		for(auto it = _scheduler.begin(), e = _scheduler.end();it != e;++it){
			if(*it == this){
				_setting = false;
				_scheduler.erase(it);
				break;
			}
		}
	}
	/**
	 * @brief スケジューラ周期リセット
	 */
	virtual inline void Reset() final{_start_time = _time;}

	/**
	 * @brief スケジューラ周期取得
	 * @return スケジューラ周期[ms]
	 */
	virtual inline uint32_t GetPeriod() const final{return _period;}


	/**
	 * @brief スケジューラ時刻チェック<br>
	 * 	USE_NUTLIB_CALLBACKSが無効の場合、HAL_SYSTICK_Callback()内で呼び出してください
	 * @details cubeでSYSTICK割り込みを1ms周期設定(デフォルト)で許可してください
	 * @attention 最近のHALはHAL_SYSTICK_Callback()が呼び出しされなくなったのでScr下のit.cにあるSysTick_Handler()内でHAL_SYSTICK_IRQHandler()の呼び出しをすること<br>
	 * 		詳しくはサンプル閲覧推奨
	 */
	static void TimeCheck(){
#ifdef HAL_TIM_MODULE_ENABLED
		if(_auxiliary_timer == NULL){
			IncTime();
		}
		else{
			_time += _auxiliary_timer->Instance->CNT;
			_auxiliary_timer->Instance->CNT = 0;
		}
#else
		IncTime();
#endif

		for(auto t : _scheduler){
			if((_time - t->_start_time) >= t->_period){
				t->Callback();
				if(t->_one_time)t->Erase();
				else t->_start_time = _time;
			}
		}
	}


#ifdef HAL_TIM_MODULE_ENABLED
	/**
	 * @brief スケジューラ補助タイマ追加
	 * @details スケジューラが渋滞した際の補助に使われるタイマの設定
	 * @param[in] timer 補助タイマハンドル
	 */
	static void SetAuxiliaryTimer(TIM_HandleTypeDef* timer){
		_auxiliary_timer = timer;
		_auxiliary_timer->Instance->CNT = 0;
		HAL_TIM_Base_Start(_auxiliary_timer);
	}
	/**
	 * @brief スケジューラ補助タイマ削除
	 * @details スケジューラが渋滞した際の補助に使われるタイマの削除
	 */
	static void EraseAuxiliaryTimer(){
		HAL_TIM_Base_Stop(_auxiliary_timer);
		_auxiliary_timer = NULL;
	}
#endif

	/**
	 * @brief　delay関数
	 * @details 他のスケジューラを阻害しない
	 * @attention スケジューラのコールバック内で呼び出ししないでください<br>
	 * 		プログラムがロックされます
	 */
	[[deprecated("It may adversely affect the control system of 'sysClock'.")]]
	static void Delay(uint32_t ms){
		volatile uint32_t end = _time + ms;
		while(TimeSchedulerBase::GetTime() <= end);//最適化の阻害
	}

	/**
	 * @brief　現在時刻取得
	 * @return 現在時刻[ms]
	 */
	volatile static uint32_t GetTime(){return _time;}
};


/**
 * @brief  タイムスケジューラクラス
 * @tparam Args コールバック関数の引数型
 */
template<class Args>
class TimeScheduler final: public TimeSchedulerBase{
public:
	/**
	 * @brief コンストラクタ
	 * @param[in] func コールバック関数
	 * @param[in] ms コールバック周期
	 */
	TimeScheduler(std::function<void(Args)>&& func, uint32_t ms) : TimeSchedulerBase(ms), _callbuck_funk(func){}
	/**
	 * @brief デストラクタ
	 */
	virtual ~TimeScheduler() override{Erase();}

	/**
	 * @brief スケジューラのセット
	 * @param arg コールバック関数の引数
	 */
	void Set(Args arg) &{
		if constexpr(std::is_lvalue_reference_v<Args>){
			argment = &arg;
		}
		else{
			argment = new Args(arg);
		}
		TimeSchedulerBase::SetSchedule();
	}
	/*void Set(Args arg) &&{
		argment = arg;
		const_cast<bool&>(_one_time) = true;
		TimeSchedulerBase::SetSchedule();
	}*/

private:
	std::function<void(Args)> _callbuck_funk;//!< コールバック関数
	typename std::remove_reference<Args>::type *argment = nullptr;//!< コールバック関数の引数

	void Callback() override{_callbuck_funk(*argment);}//!< コールバック関数呼び出し
};

/**
 * @brief  タイムスケジューラクラスのvoid特殊化
 */
template<>
class TimeScheduler<void> final: public TimeSchedulerBase{
public:
	/**
	 * @brief コンストラクタ
	 * @param[in] func コールバック関数
	 * @param[in] ms コールバック周期
	 */
	TimeScheduler(std::function<void(void)>&& func, uint32_t ms) : TimeSchedulerBase(ms), _callbuck_funk(func){}
	/**
	 * @brief デストラクタ
	 */
	virtual ~TimeScheduler() override{Erase();}

	/**
	 * @brief スケジューラのセット
	 */
	void Set() &{TimeSchedulerBase::SetSchedule();}
	/*void Set() &&{
		const_cast<bool&>(_one_time) = true;
		TimeSchedulerBase::SetSchedule();
	}*/
private:
	std::function<void(void)> _callbuck_funk;//!< コールバック関数

	void Callback() override{_callbuck_funk();}//!< コールバック関数呼び出し
};


/*
 * 関数遅延呼び出し
 */
/*
template<typename T>
inline void DelayCall(std::function<void(T)>&& callback, T arg, uint32_t ms){
	typename TimeScheduler<T>::TimeScheduler(std::move(callback), ms).Set(arg);
}
inline void DelayCall(std::function<void(void)>&& callback, uint32_t ms){
	typename TimeScheduler<void>::TimeScheduler(std::move(callback), ms).Set();
}*/


}



/* NUT callback */
#ifdef USE_NUTLIB_CALLBACKS
namespace nut::callback{
/*@brief  SysTickのコールバッククラス */
static inline HALCallback<void> Systick;
}
#endif
