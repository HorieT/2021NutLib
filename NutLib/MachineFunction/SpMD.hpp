/**
 * MDの特殊ファームの基底
 */
#pragma once

#include "NutLib/Global.hpp"
#include "NutLib/CANWrapper.hpp"
#include "NutLib/CANBusProtocol.hpp"
#include "NutLib/Unit/UnitCore.hpp"

namespace nut::NHK21{
class SpMD {
protected:
	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _my_id;
	const uint8_t _id;

	bool _is_started = false;
	bool _is_move = false;
	CANWrapper::RxExCallbackIt _rx_callback_it;
	std::function<void(void)> _move_end_callback;


	bool ReceiveBase(CANWrapper::RxDataType rx_data){
		if(rx_data.data[0] == _id && _is_started){
			if(can_protocol::GetDataType(rx_data.header.StdId) == 0xF && rx_data.data[1] == 0xFF){
				_is_move = false;
				if(_move_end_callback)_move_end_callback();
				return true;
			}else{
				return Receive(rx_data);
			}
		}
		return false;
	}

	virtual bool Receive(CANWrapper::RxDataType rx_data) = 0;


public:
	SpMD(const std::shared_ptr<CANWrapper>& can, uint8_t my_id, uint8_t dv_num, uint8_t priolity):
	_can(can), _my_id(my_id), _id(can_protocol::MakeDeviceID(can_protocol::Device::motorDriver, dv_num)){
		_rx_callback_it = _can->FIFO0ReceiveCallback().AddExclusiveCallback(priolity, [this](CANWrapper::RxDataType rx){return ReceiveBase(rx);});
	}
	virtual ~SpMD(){
		_can->FIFO0ReceiveCallback().EraseExclusiveCallback(_rx_callback_it);
	}

	virtual void Start(){
		if(_is_started)return;
		std::array<uint8_t, 2> data{_my_id, static_cast<uint8_t>(can_protocol::motor::SpecialOperation::dedicatedFirmware)};
		_can->Transmit<2>(can_protocol::MakeCANID(_id, can_protocol::motor::DataType::specialOperation), data);
		_is_started = true;
	}
	virtual void Stop(){
		if(!_is_started)return;
		std::array<uint8_t, 2> data{_my_id, static_cast<uint8_t>(can_protocol::motor::SpecialOperation::stop)};
		_can->Transmit<2>(can_protocol::MakeCANID(_id, can_protocol::motor::DataType::specialOperation), data);
		_is_started = false;
		_is_move = false;
	}

	void SetMoveEndCallback(std::function<void(void)> callback){
		_move_end_callback = callback;
	}

	bool IsMove()const{
		return _is_move;
	}
};
}
