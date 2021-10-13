/**
 * 装填機構
 */
#pragma once

#include "NutLib/Global.hpp"
#include "SpMD.hpp"

namespace nut::NHK21{
class Loader final : public SpMD{
private:
	virtual bool Receive(CANWrapper::RxDataType rx_data) override{
		return true;
	}

public:
	Loader(const std::shared_ptr<CANWrapper>& can, uint8_t my_id, uint8_t dv_num):
	SpMD(can, my_id, dv_num, 7){
	}
	~Loader(){
	}

	bool Load(uint8_t point){
		if(_is_move || !_is_started || point > 4)return false;
		std::array<uint8_t, 2> data{_my_id, point};
		_can->Transmit<2>(can_protocol::MakeCANID(_id, 0x1), data);
		return true;
	}
};
}
