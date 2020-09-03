/*
 * canのヘルパクラス
 */
#pragma once

#include <NutLib/Global.hpp>
#include <array>
#include <vector>


namespace nut{
class CAN{
private:
	CAN_HandleTypeDef* const _hcan;

public:
	CAN(CAN_HandleTypeDef* hcan) : _hcan(hcan){}
	~CAN(){}

	/*
	 * データ送信メソッド
	 */
	template<uint8_t N>
	void Transmit(uint32_t id, std::array<uint8_t, N> data){
		static_assert(N <= 8, "CAN data size over!");

		uint32_t            tx_mailbox = 0;
		CAN_TxHeaderTypeDef tx_header;

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.IDE = CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;

		tx_header.DLC = N;

		while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);
		HAL_CAN_AddTxMessage(_hcan, &tx_header, data.data(), &tx_mailbox);
	}
	void Transmit(uint32_t id, std::vector<uint8_t> data){
		uint32_t            tx_mailbox = 0;
		CAN_TxHeaderTypeDef tx_header;

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = CAN_RTR_DATA;//CAN_RTR_DATA;
		tx_header.IDE = CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;

		tx_header.DLC = (data.size() < 8) ? data.size() : 8;

		while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);
		HAL_CAN_AddTxMessage(_hcan, &tx_header, data.data(), &tx_mailbox);
	}
	void Transmit(uint32_t id){
		uint32_t            tx_mailbox = 0;
		CAN_TxHeaderTypeDef tx_header;
		std::array<uint8_t, 8> data;//dummy

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = CAN_RTR_DATA;//CAN_RTR_DATA;
		tx_header.IDE = CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;

		tx_header.DLC = 0;

		while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);
		HAL_CAN_AddTxMessage(_hcan, &tx_header, data.data(), &tx_mailbox);
	}
	/*
	 * リモートフレーム送信
	 */
	void TransmitRemote(uint32_t id){
		uint32_t            tx_mailbox = 0;
		CAN_TxHeaderTypeDef tx_header;
		std::array<uint8_t, 8> data;//dummy

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = CAN_RTR_REMOTE;
		tx_header.IDE = CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;

		tx_header.DLC = 0;

		while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);
		HAL_CAN_AddTxMessage(_hcan, &tx_header, data.data(), &tx_mailbox);
	}

	CAN_HandleTypeDef* GetHandle()const{
		return _hcan;
	}
};
}
