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
	std::array<uint8_t, 8> _rx_data = {0};

public:
	CAN(CAN_HandleTypeDef* hcan) : _hcan(hcan){}
	~CAN(){}

	/*
	 * データ送信メソッド
	 */
	void Transmit(uint32_t id, std::vector<uint8_t> data){
		uint32_t            tx_mailbox = 0;
		CAN_TxHeaderTypeDef tx_header;

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = 0x0U;//CAN_RTR_DATA;
		tx_header.IDE = 0x0U;//CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;

		tx_header.DLC = data.size();

		while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);
		HAL_CAN_AddTxMessage(_hcan, &tx_header, data.data(), &tx_mailbox);
	}
	template<uint8_t N>
	void Transmit(uint32_t id, std::array<uint8_t, N> data){
		uint32_t            tx_mailbox = 0;
		CAN_TxHeaderTypeDef tx_header;

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = 0x0U;//CAN_RTR_DATA;
		tx_header.IDE = 0x0U;//CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;

		tx_header.DLC = N;

		while(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0);
		HAL_CAN_AddTxMessage(_hcan, &tx_header, data.data(), &tx_mailbox);
	}
};
}
