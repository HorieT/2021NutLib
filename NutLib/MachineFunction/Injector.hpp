/**
 * 射出機構
 */
#pragma once

#include "NutLib/Global.hpp"
#include "SpMD.hpp"

namespace nut::NHK21{
class Injector final : public SpMD{
private:
	static constexpr float MAX_VEL = 7.5;
	static constexpr float MIN_VEL = 0.1;

	bool _is_loaded = false;

	virtual bool Receive(CANWrapper::RxDataType rx_data) override{
		_is_move = false;
		return true;
	}


public:
	Injector(const std::shared_ptr<CANWrapper>& can, uint8_t my_id, uint8_t dv_num):
	SpMD(can, my_id, dv_num, 4){
	}
	~Injector(){
	}

	bool Load(){
		if(_is_move || !_is_started)return false;
		_is_move = true;
		std::array<uint8_t, 1> data{_my_id};
		_can->Transmit<1>(can_protocol::MakeCANID(_id, 0x1), data);
		return true;
	}
	bool Inject(float mps){
		if(_is_move || !_is_started || mps > MAX_VEL || mps < MIN_VEL)return false;
		_is_move = true;
		std::array<uint8_t, 5> data{_my_id};
		std::memcpy(&data[1], &mps, 4);
		_can->Transmit<5>(can_protocol::MakeCANID(_id, 0x2), data);
		return true;
	}
};
}
