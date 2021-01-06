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
#include "Unit/UnitCore.hpp"
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
	static inline volatile uint32_t _time = 0;//volatileで最適化除外を行うため組み込み型
	static inline std::vector<TimeSchedulerBase*> _scheduler;
	static inline std::vector<TimeSchedulerBase*> _delay_func;

#ifdef HAL_TIM_MODULE_ENABLED
	static inline TIM_HandleTypeDef* _auxiliary_timer = nullptr;
#endif

	MilliSecond<uint32_t> _start_time = 0;
	MilliSecond<uint32_t> _period;
	bool _setting = false;


	static inline void IncTime(){_time += 1;}

protected:

	/**
	 * @brief スケジューラのセット
	 */
	void SetSchedule(){
		if(_setting)return;
		_setting = true;
		_scheduler.push_back(this);
		_start_time = _time;
	}

	static void SetDelayFunc(TimeSchedulerBase*&& this_h){
		//if(_setting)return;
		this_h->_setting = true;
		_scheduler.push_back(std::move(this_h));
		this_h->_start_time = _time;
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
	TimeSchedulerBase(MilliSecond<uint32_t> ms) : _period(ms){}

	/**
	 * @brief デストラクタ
	 */
	virtual ~TimeSchedulerBase(){Erase();}

	/**
	 * @brief スケジューラ削除
	 */
	void Erase() {
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
	virtual inline MilliSecond<uint32_t> GetPeriod() const final{return _period;}


	/**
	 * @brief スケジューラ時刻チェック<br>
	 * 	USE_NUTLIB_CALLBACKSが無効の場合、HAL_SYSTICK_Callback()内で呼び出してください
	 * @details cubeでSYSTICK割り込みを1ms周期設定(デフォルト)で許可してください
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
				 t->_start_time = _time;
			}
		}
		for(auto it = _delay_func.begin();it != _delay_func.end();++it){
			if((_time - (*it)->_start_time) >= (*it)->_period){
				(*it)->Callback();
				it = std::prev(_delay_func.erase(it), 1);
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
	static void Delay(MilliSecond<uint32_t> ms){
		volatile const uint32_t end = _time + static_cast<uint32_t>(ms);
		while(_time <= end);//最適化の阻害
	}

	/**
	 * @brief　現在時刻取得
	 * @return 現在時刻[ms]
	 */
	static MilliSecond<uint32_t> GetTime(){return _time;}
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
	TimeScheduler(std::function<void(Args)>&& func, MilliSecond<uint32_t> ms) : TimeSchedulerBase(ms), _callbuck_funk(func){}
	/**
	 * @brief デストラクタ
	 */
	virtual ~TimeScheduler() {}

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
	TimeScheduler(std::function<void(void)>&& func, MilliSecond<uint32_t> ms) : TimeSchedulerBase(ms), _callbuck_funk(func){}
	/**
	 * @brief デストラクタ
	 */
	virtual ~TimeScheduler() {}

	/**
	 * @brief スケジューラのセット
	 */
	void Set() &{TimeSchedulerBase::SetSchedule();}
	static void DelayCall(std::function<void(void)>&& func, MilliSecond<uint32_t> ms){
		TimeSchedulerBase::SetDelayFunc(new TimeScheduler<void>(std::move(func), ms));
	}
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
inline HALCallback<void> Systick;
}
#endif
