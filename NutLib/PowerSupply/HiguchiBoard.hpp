/*
 * 電源遮断基板樋口のクラス
 * Powerのを写してきたので不適格
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include "../CAN.hpp"
#include <memory>

namespace nut{
class HiguchiBoard{
private:
	typedef enum{
		GateSignal = 0x00,

		PS_ReadStatus = 0x10,

	}PS_DataType_t;

	typedef union{
		struct{
			PS_DataType_t dataType	: 5;
			unsigned boardChannel		: 2;
			unsigned boardType			: 3;//そもそもフィールドがおかしくないか？
			unsigned forMotor 			: 1;
		};
		uint32_t StdID;
	}PS_StdID_t;


	TimeScheduler<void> _scheduler;
	uint8_t _id;
	std::shared_ptr<CAN> _can;
	bool _swich_on = false;
	bool _swich_state = false;

	void SendState(){
		PS_StdID_t txStdID;
		txStdID.forMotor = 0;
		txStdID.boardType = 0x00;
		txStdID.boardChannel = _id;
		txStdID.dataType = GateSignal;

		std::array<uint8_t, 1> data{static_cast<uint8_t>(_swich_on)};
		_can->Transmit<1>(txStdID.StdID, data);
	}

public:
	HiguchiBoard(uint32_t period, uint8_t id, std::shared_ptr<CAN> can)
		: _scheduler([this]{SendState();}, period), _id(id & 0x03), _can(can){

	}
	~HiguchiBoard(){
		_scheduler.Erase();
	}

	void SwichOn(){
		_scheduler.Set();
		_swich_on = true;
	}
	void SwichOff(){
		_scheduler.Erase();
		_swich_on = false;
	}

	void ReadSwichState(){
		PS_StdID_t txStdID;
		txStdID.forMotor = 0;
		txStdID.boardType = 0x00;
		txStdID.boardChannel = _id;
		txStdID.dataType = PS_ReadStatus;

		_can->TransmitRemote(txStdID.StdID);
	}

	/*
	 * CAN受信関数
	 */
	bool ReadCanData(CAN_HandleTypeDef* hcan, const CAN_RxHeaderTypeDef& RxHeader, const std::array<uint8_t, 8> data){
		if(hcan == _can->GetHandle()){
			PS_StdID_t txStdID;
			txStdID.forMotor = 0;
			txStdID.boardType = 0x00;
			txStdID.boardChannel = _id;
			txStdID.dataType = PS_ReadStatus;
			if(RxHeader.RTR == CAN_RTR_DATA &&
					RxHeader.StdId  == txStdID.StdID){
				_swich_state = static_cast<bool>(data[0]);
				return true;
			}
		}

		return false;
	}
};
}
