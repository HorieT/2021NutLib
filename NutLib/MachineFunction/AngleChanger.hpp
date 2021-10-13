/**
 * 角度変更機構
 */
#pragma once

#include "NutLib/Global.hpp"
#include "SpMD.hpp"


namespace nut::NHK21{
class AngleChanger final : public SpMD{
private:
	static constexpr Radian<float> MAX_RAD = 47.0 * M_PI / 180.0;
	static constexpr Radian<float> MIN_RAD = 27.0 * M_PI / 180.0;

	virtual bool Receive(CANWrapper::RxDataType rx_data) override{
		return true;
	}
public:
	AngleChanger(const std::shared_ptr<CANWrapper>& can, uint8_t my_id, uint8_t dv_num):
	SpMD(can, my_id, dv_num, 6){
	}
	~AngleChanger(){
	}


	bool InitAngle(){
		if(_is_move || !_is_started)return false;
		std::array<uint8_t, 1> data{_my_id};
		_can->Transmit<1>(can_protocol::MakeCANID(_id, 0x1), data);
		return true;
	}
	bool SetAngle(Radian<float> rad){
		if(_is_move || !_is_started || rad > MAX_RAD || rad < MIN_RAD)return false;
		std::array<uint8_t, 5> data{_my_id};
		float tmp = rad.f();
		std::memcpy(&data[1], &tmp, 4);
		_can->Transmit<5>(can_protocol::MakeCANID(_id, 0x2), data);
		return true;
	}
};
}
