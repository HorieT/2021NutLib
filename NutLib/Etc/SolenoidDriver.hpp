/**
 *
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include "../CANWrapper.hpp"
#include "../CANBusProtocol.hpp"

namespace nut{
class SolenoidDriver final{
private:
	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _my_id;
	const uint8_t _id;

	enum LastSendType : uint8_t{
		And,
		Or,
		Copy
	};
	LastSendType _last_type = LastSendType::And;
	uint16_t _last_send_data = 0;
	bool _continue_flag = false;
	TimeScheduler<void> _continue_check{[this]{_continue_flag = false;}, 50};

public:
	SolenoidDriver(const std::shared_ptr<CANWrapper>& can, uint8_t dv_num, uint8_t my_id) :
	_can(can), _my_id(my_id), _id(can_protocol::MakeDeviceID(can_protocol::Device::solenoidValve, dv_num)){
		_continue_check.Set();
	}
	~SolenoidDriver(){
		_continue_check.Erase();
	}

	bool SetAnd(uint16_t bit){
		if(_last_type == LastSendType::And && bit == _last_send_data && _continue_flag)return false;
		std::array<uint8_t, 3> data{_my_id};
		std::memcpy(&data[1], &bit, 2);

		_last_send_data = bit;
		_last_type = LastSendType::And;
		_continue_flag = true;
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveSetAnd), data);
		_continue_check.Reset();
		return true;
	}
	bool SetOr(uint16_t bit){
		if(_last_type == LastSendType::Or && bit == _last_send_data && _continue_flag)return false;
		std::array<uint8_t, 3> data{_my_id};
		std::memcpy(&data[1], &bit, 2);

		_last_send_data = bit;
		_last_type = LastSendType::Or;
		_continue_flag = true;
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveSetOr), data);
		_continue_check.Reset();
		return true;
	}
	bool SetCopy(uint16_t bit){
		if(_last_type == LastSendType::Copy && bit == _last_send_data && _continue_flag)return false;
		std::array<uint8_t, 3> data{_my_id};
		std::memcpy(&data[1], &bit, 2);

		_last_send_data = bit;
		_last_type = LastSendType::Copy;
		_continue_flag = true;
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveState), data);
		_continue_check.Reset();
		return true;
	}


	void On(uint16_t bit){
		SetOr(bit);
	}
	void On(uint16_t bit, MilliSecond<uint32_t> time){
		On(bit);
		TimeScheduler<void>::DelayCall([this, bit](){Off(bit);}, time);
	}
	void On(uint16_t bit, MilliSecond<uint32_t> time, std::function<void(void)> callback){
		On(bit);
		TimeScheduler<void>::DelayCall(
				[this, bit, callback](){
					Off(bit);
					callback();
				}, time);
	}
	void Off(uint16_t bit){
		SetAnd(~bit);
	}
	void Off(uint16_t bit, MilliSecond<uint32_t> time){
		Off(bit);
		TimeScheduler<void>::DelayCall([this, bit](){On(bit);}, time);
	}
	void Off(uint16_t bit, MilliSecond<uint32_t> time, std::function<void(void)> callback){
		Off(bit);
		TimeScheduler<void>::DelayCall(
				[this, bit, callback](){
					On(bit);
					callback();
				}, time);
	}

	void AllOn(){
		On(0x3FF);
	}
	void AllOff(){
		Off(0x3FF);
	}
};
}
