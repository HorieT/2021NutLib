/**
 * ROSとの通信回りの処理の基底
 * ユーザーが継承して使って
 */
#pragma once
#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include "../Coordinate.hpp"
#include "../CANWrapper.hpp"
#include "../CANBusProtocol.hpp"
#include "../HALCallbacks/HALCallbacks.hpp"

#include <functional>

namespace nut::NHK21{
class ComROS{
private:
	static constexpr MilliSecond<uint32_t> SEND_PERIOD = 10;
	static constexpr MilliSecond<uint32_t> TIMEOUT = 150;
	TimeScheduler<void> _send_schdule{[this](){UserSendMsg();}, SEND_PERIOD};
	TimeScheduler<void> _timeout{[this](){Timeout();}, TIMEOUT};
	bool _is_init = false;
	bool _is_enabled = false;
	bool _is_active = false;
	CANWrapper::RxExCallbackIt _rx_callback_it;



	bool Receive(CANWrapper::RxDataType rx_data){
		if((rx_data.data[0] == _ros_id) && _is_enabled){
			_timeout.Reset();
			_is_active = true;
			return UserReceive(rx_data);
		}
		return false;
	}

	void Timeout(){
		_is_active = false;
		//_is_enabled = false;
		UserTimeout();
	}

protected:
	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _my_id = 0;
	const uint8_t _ros_id = 0;


	virtual bool UserReceive(CANWrapper::RxDataType rx_data) = 0;
	virtual void UserTimeout() = 0;
	virtual void UserSendMsg() = 0;
	virtual void UserInit() = 0;


public:
	ComROS(std::shared_ptr<CANWrapper> can) :
	_can(can){
	}
	virtual ~ComROS(){
		Deinit();
	}


	void Init(uint8_t my_id, uint8_t ros_num){
		if(_is_init)return;
		const_cast<uint8_t&>(_my_id) = my_id;
		const_cast<uint8_t&>(_ros_id) = can_protocol::MakeDeviceID(can_protocol::Device::PC, ros_num);
		_rx_callback_it = _can->FIFO0ReceiveCallback().AddExclusiveCallback(2, [this](CANWrapper::RxDataType rx){return Receive(rx);});
		UserInit();
		_is_init = true;
	}
	void Deinit(){
		if(!_is_init)return;
		SetDisable();
		_can->FIFO0ReceiveCallback().EraseExclusiveCallback(_rx_callback_it);
		_is_active = false;
		_is_init = false;
	}

	bool SetEnable(){
		if(!_is_init)return false;
		if(_is_enabled)return true;
		//if(!_is_active)return false;
		_is_enabled = true;
		_send_schdule.Set();
		_timeout.Set();
		return true;
	}
	void SetDisable(){
		if(!_is_init || !_is_enabled)return;
		_timeout.Erase();
		_send_schdule.Erase();
		_is_enabled = false;
	}
	bool IsEnable()const{return _is_enabled;}
	bool IsActeve()const{return _is_active;}
	uint8_t GetID()const{return _ros_id;}
};
}
