/**
 * @file CAN.hpp
 * @brief CANのヘルパ
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Global.hpp"
#include <array>
#include <vector>


namespace nut{
/**
 * @brief CANのヘルパクラス<br>
 * 現時点で最新のHALでのCANにしか対応していない
 */
class CAN{
private:
	CAN_HandleTypeDef* const _hcan;

public:
	/*
	 * @param[in] hcan CANハンドル
	 */
	CAN(CAN_HandleTypeDef* hcan) : _hcan(hcan){}
	~CAN(){}

	/**
	 * @brief 通常のデータフレームで送信します
	 * @tparam N 8以下のデータサイズ
	 * @param[in] id CAN識別子
	 * @param[in] data 送信データ
	 * @attention N>8でアサートを吐きます
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
	/**
	 * @brief 通常のデータフレームで送信します
	 * @param[in] id CAN識別子
	 * @param[in] data 送信データ
	 * @attention data.size()>8であれば[8]以降のデータは無視されます
	 */
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
	/**
	 * @brief 通常のデータフレームで空送信します
	 * @param[in] id CAN識別子
	 */
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

	/**
	 * @brief 通常のリモートフレームを送信します
	 * @param[in] id CAN識別子
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

	/**
	 * @brief 保有ハンドルのゲッター
	 * @return 保有しているCANハンドルの生ポインタ
	 */
	CAN_HandleTypeDef* GetHandle()const{
		return _hcan;
	}
};
}
