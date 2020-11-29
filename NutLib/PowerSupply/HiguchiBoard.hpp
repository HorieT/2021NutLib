/**
 * @file HiguchiBoard.hpp
 * @brief 電源制御基板"樋口"
 * @details 過去ライブラリの移植
 * @author Horie
 * @date 2020/10
 * @attention そのまま移植しているのでコーディングルールを逸脱しています
 */
#pragma once

#include "../CANWrapper.hpp"
#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include <memory>

namespace nut{
/**
 * @brief 電源制御基板"樋口"クラス
 * @attention そのまま移植しているのでコーディングルールの逸脱、未定義動作が含まれます
 */
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
			unsigned boardType			: 3;
			unsigned forMotor 			: 1;
		};
		uint32_t StdID;
	}PS_StdID_t;


	TimeScheduler<void> _scheduler;
	uint8_t _id;
	std::shared_ptr<CANWrapper> _can;
	bool _swich_on = false;//!< 書き込み
	bool _swich_state = false;//!< 読み取り

	/**
	 * @brief スイッチ状態周期送信関数
	 */
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
	/**
	 * @brief コンストラクタ
	 * @param[in] period 周期[ms]
	 * @param[in] id 2bit基板id
	 * @param[in] can canのヘルパインスタンス
	 */
	[[deprecated("Stability is not guaranteed as it is a port of past libraries.")]]
	HiguchiBoard(uint32_t period, uint8_t id,const std::shared_ptr<CANWrapper>& can)
		: _scheduler([this]{SendState();}, period), _id(id & 0x03), _can(can){

	}
	/**
	 * @brief デストラクタ
	 */
	~HiguchiBoard(){
		_scheduler.Erase();
	}


	/**
	 * @brief 電源スイッチオン
	 */
	void SwichOn(){
		_scheduler.Set();
		_swich_on = true;
	}


	/**
	 * @brief 電源スイッチオフ
	 */
	void SwichOff(){
		_scheduler.Erase();
		_swich_on = false;
	}

	/**
	 * @brief スイッチ読み取り要求
	 */
	void ReadSwichState(){
		PS_StdID_t txStdID;
		txStdID.forMotor = 0;
		txStdID.boardType = 0x00;
		txStdID.boardChannel = _id;
		txStdID.dataType = PS_ReadStatus;

		_can->TransmitRemote(txStdID.StdID);
	}

	/**
	 * @brief CAN受信関数
	 * @details HAL_CAN_RxFifo0MsgPendingCallback()またはHAL_CAN_RxFifo1MsgPendingCallback()内で呼び出してください
	 * @param[in] hcan canハンドル
	 * @param[in] RxHeader 受信ヘッダ
	 * @param[in] data 受信データ
	 * @return 受信パケットがこの電源基板に該当するかどうか
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
