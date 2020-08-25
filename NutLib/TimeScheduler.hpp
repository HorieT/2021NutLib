/*
 * タスクの時間制御クラス
 * 周期的に呼び出す必要のあるものに使う
 */
#pragma once

#include <NutLib/Global.hpp>
#include <functional>
#include <vector>
#include <memory>

namespace nut{
/*
 * ベースクラス
 */
class TimeSchedulerBase : public std::enable_shared_from_this<TimeSchedulerBase>{
private:
	static inline volatile uint32_t _time = 0;
	static inline std::vector<std::shared_ptr<TimeSchedulerBase>> _scheduler;
	//TimeCheck()がつっかえたとき用の補助タイマ
	static inline TIM_HandleTypeDef* _auxiliary_timer = NULL;

	uint32_t _start_time = 0;
	uint32_t _period;
	bool _setting = false;


	static inline void IncTime(){_time++;}

protected:
	//one timeオブジェクト用
	const bool _one_time;

	//タイムスケジューラセット
	void SetSchedule(){
		if(_setting)return;
		_setting = true;
		_scheduler.push_back(shared_from_this());
		_start_time = _time;
	}
	virtual void Callback() = 0;


public:
	/*
	 * コンストラクタ・デストラク
	 */
	TimeSchedulerBase(uint32_t ms) : _period(ms), _one_time(false){}
	virtual ~TimeSchedulerBase(){Erase();}

	/*
	 * タイムスケジューラ消去
	 */
	virtual void Erase() final{
		if(!_setting)return;
		for(auto it = _scheduler.begin(), e = _scheduler.end();it != e;++it){
			if(*it == shared_from_this()){
				_setting = false;
				_scheduler.erase(it);
				break;
			}
		}
	}
	/*
	 * タイムスケジューラリセット
	 */
	virtual inline void Reset() final{_start_time = _time;}

	/*
	 * ゲッター
	 */
	virtual inline uint32_t GetPeriod() final{return _period;}


	/*
	 * 時間計測　1ms周期割り込み関数内で呼び出し
	 */
	static void TimeCheck(){
		if(_auxiliary_timer == NULL){
			IncTime();
		}
		else{
			_time += _auxiliary_timer->Instance->CNT;
			_auxiliary_timer->Instance->CNT = 0;
		}

		for(auto t : _scheduler){
			if((_time - t->_start_time) >= t->_period){
				t->Callback();
				if(t->_one_time)t->Erase();
				else t->_start_time = _time;
			}
		}
	}


	/*
	 * 補助タイマセット
	 * 補助タイマは1CNT/msで十分な大きさのARRがセットされてること
	 */
	static void SetAuxiliaryTimer(TIM_HandleTypeDef* timer){
		_auxiliary_timer = timer;
		_auxiliary_timer->Instance->CNT = 0;
		HAL_TIM_Base_Start(_auxiliary_timer);
	}
	static void EraseAuxiliaryTimer(){
		HAL_TIM_Base_Stop(_auxiliary_timer);
		_auxiliary_timer = NULL;
	}
	/*
	 *delay関数(timeScheduler内では使わないように)
	 * 関数遅延呼び出し→クラス外に記述
	 */
	[[deprecated("It may adversely affect the control system of 'sysClock'.")]]
	static void Delay(uint32_t ms){
		volatile uint32_t end = _time + ms;
		while(TimeSchedulerBase::GetTime() <= end);//関数呼び出しによる最適化の除外
	}

	/*
	 * 時刻取得
	 */
	volatile static uint32_t GetTime(){return _time;}
};


/*
 * タイムスケジューラ
 */
template<class Args>
class TimeScheduler final: public TimeSchedulerBase{
public:
	TimeScheduler(std::function<void(Args)>&& func, uint32_t ms) : TimeSchedulerBase(ms), _callbuck_funk(func){}
	virtual ~TimeScheduler() override{Erase();}

	void Set(Args arg) &{
		argment = arg;
		TimeSchedulerBase::SetSchedule();
	}
	void Set(Args arg) &&{
		argment = arg;
		const_cast<bool&>(_one_time) = true;
		TimeSchedulerBase::SetSchedule();
	}

private:
	std::function<void(Args)> _callbuck_funk;
	Args argment = static_cast<Args>(0);

	void Callback() override{_callbuck_funk(argment);}
};
/*
 * void型の完全特殊化タイムスケジューラ
 */
template<>
class TimeScheduler<void> final: public TimeSchedulerBase{
public:
	TimeScheduler(std::function<void(void)>&& func, uint32_t ms) : TimeSchedulerBase(ms), _callbuck_funk(func){}
	virtual ~TimeScheduler() override{Erase();}

	void Set() &{TimeSchedulerBase::SetSchedule();}
	void Set() &&{
		const_cast<bool&>(_one_time) = true;
		TimeSchedulerBase::SetSchedule();
	}
private:
	std::function<void(void)> _callbuck_funk;

	void Callback() override{_callbuck_funk();}
};


/*
 * 関数遅延呼び出し
 */
template<typename T>
inline void DelayCall(std::function<void(T)>&& callback, T arg, uint32_t ms){
	typename TimeScheduler<T>::TimeScheduler(std::move(callback), ms).Set(arg);
}
inline void DelayCall(std::function<void(void)>&& callback, uint32_t ms){
	typename TimeScheduler<void>::TimeScheduler(std::move(callback), ms).Set();
}
}
