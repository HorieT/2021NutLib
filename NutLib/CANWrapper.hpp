/**
 * @file CANWrapper.hpp
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
class CANWrapper{
private:
	CAN_HandleTypeDef* const _hcan;

public:
	/*
	 * @param[in] hcan CANハンドル
	 */
	CANWrapper(CAN_HandleTypeDef* hcan) : _hcan(hcan){}
	~CANWrapper(){}

	/**
	 * @brief フィルタを32bitマスクモードで設定します
	 * @details 標準フレームフォーマットのみサポート
	 * @attention SlaveStartFilterBankは脳死で14になっているのでスレーブCAN使用時は注意
	 * @param[in] bank フィルタバンク
	 * @param[in] id 11bit ID
	 * @param[in] id_mask 11bit IDマスク
	 * @param[in] data_only データフレームのみ
	 * @param[in] remote_only リモートフレームのみ
	 * @param[in] fifo FIFOx
	 */
	bool SetFilterMask(
			uint32_t bank,
			uint16_t id,
			uint16_t id_mask,
			bool data_only,
			bool remote_only,
			uint32_t fifo){

		if(data_only && remote_only)return false;
		CAN_FilterTypeDef  filter;

		filter.FilterBank = bank;
		filter.FilterMode = CAN_FILTERMODE_IDMASK;
		filter.FilterScale = CAN_FILTERSCALE_32BIT;
		filter.FilterIdHigh = id << 5;
		filter.FilterIdLow = 0b0100 | remote_only << 1;
		filter.FilterMaskIdHigh = id_mask << 5;
		filter.FilterMaskIdLow = 0b0100 | ((data_only || remote_only) << 1);
		filter.FilterFIFOAssignment = fifo;
		filter.FilterActivation = ENABLE;
		filter.SlaveStartFilterBank = 14;


		if (HAL_CAN_ConfigFilter(_hcan, &filter) != HAL_OK)return false;
		return true;
	}

	/**
	 * @brief CAN開始
	 * @return ステート
	 */
	bool Start(){
		if (HAL_CAN_Start(_hcan) != HAL_OK)return false;
		return true;
	}
	/**
	 * @brief 割り込みありCAN開始
	 * @param[in] active_its 割り込み設定フラグ
	 * @return ステート
	 */
	bool Start(uint32_t active_its){
		if (HAL_CAN_Start(_hcan) != HAL_OK)return false;
		if (HAL_CAN_ActivateNotification(_hcan, active_its) != HAL_OK)return false;
		return true;
	}
	/**
	 * @brief デバッグモードCAN開始
	 * @return ステート
	 */
	bool StartOnDebug(){
		_hcan->Instance->MCR &= 0xFFFEFFFF;//debug
		if (HAL_CAN_Start(_hcan) != HAL_OK)return false;
		return true;
	}
	/**
	 * @brief 割り込みありデバッグモードCAN開始
	 * @param[in] active_its 割り込み設定フラグ
	 * @return ステート
	 */
	bool StartOnDebug(uint32_t active_its){
		_hcan->Instance->MCR &= 0xFFFEFFFF;//debug
		if (HAL_CAN_Start(_hcan) != HAL_OK)return false;
		if (HAL_CAN_ActivateNotification(_hcan, active_its) != HAL_OK)return false;
		return true;
	}

	/**
	 * @brief CAN停止
	 * @return ステート
	 */
	bool Stop(){
		if (HAL_CAN_Stop(_hcan) != HAL_OK)return false;
		return true;
	}


	/**
	 * @brief 標準データフレームで送信します
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
	 * @brief 標準データフレームで送信します
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
	 * @brief 標準データフレームで空送信します
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
	 * @brief 標準リモートフレームを送信します
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
