/**
 * @file CANWrapper.hpp
 * @brief CANのヘルパ
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Global.hpp"
#include "HALCallbacks/CAN.hpp"
#include <array>
#include <vector>
#include <queue>



namespace nut{
/**
 * @brief CANのヘルパクラス<br>
 * 現時点で最新のHALでのCANにしか対応していない
 * @details CANの要求仕様
 * ・Tx,Rx0,Rx1,Error割込みの有効化
 */
class CANWrapper{
public:
	struct TxDataType{
		CAN_TxHeaderTypeDef header;
		std::array<uint8_t, 8> data;
		uint32_t            mailbox;
	};
	struct RxDataType{
		CAN_RxHeaderTypeDef header;
		std::array<uint8_t, 8> data;
	};

private:
	using TxCallbackIt = decltype(callback::CAN_TxMailboxComplete)::CallbackIterator;
	using Rx0CallbackIt = decltype(callback::CAN_RxFifo0MsgPending)::ExCallbackIterator;
	using Rx1CallbackIt = decltype(callback::CAN_RxFifo1MsgPending)::ExCallbackIterator;

	static constexpr uint32_t CAN_DEFAULT_CALLBACK_FLAGS =
			CAN_IT_RX_FIFO0_MSG_PENDING |
			CAN_IT_RX_FIFO1_MSG_PENDING |
			CAN_IT_RX_FIFO0_FULL |
			CAN_IT_RX_FIFO1_FULL |
			CAN_IT_TX_MAILBOX_EMPTY |
			CAN_IT_ERROR;

	CAN_HandleTypeDef* const _hcan;
	std::priority_queue<
		TxDataType,
		std::vector<TxDataType>,
		std::greater<TxDataType>
		> _send_data_queue;

	TxCallbackIt _tx_callback_it;
	Rx0CallbackIt _rx0_callback_it;
	Rx1CallbackIt _rx1_callback_it;
	std::array<HALCallback<RxDataType>, 2> _receive_callback;


	/**
	 * @brief メールボックス追加関数
	 * @details 送信メールボックスにデータを入れます
	 * @param[in] tx_data 送信データ
	 */
	void AddMailbox(TxDataType tx_data){
		if(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) != 0){
			HAL_CAN_AddTxMessage(
					_hcan,
					&tx_data.header,
					tx_data.data.data(),
					&tx_data.mailbox);
		}
		else{
			_send_data_queue.push(tx_data);
		}
	}

	/**
	 * @brief メールボックス排出関数
	 * @details 送信メールボックスに_send_data_queueのデータを入れます
	 * @param[in] hcan 割込みCANハンドラ
	 */
	void MailboxComplete(CAN_HandleTypeDef* hcan){
		if(hcan != _hcan || _send_data_queue.empty())return;
		const auto& tx_data = _send_data_queue.top();
		HAL_CAN_AddTxMessage(
				_hcan,
				const_cast<CAN_TxHeaderTypeDef*>(&tx_data.header),
				const_cast<uint8_t*>(tx_data.data.data()),
				const_cast<uint32_t*>(&tx_data.mailbox));
		_send_data_queue.pop();
	}

	/**
	 * @brief 受信関数
	 * @param[in] hcan canハンドル
	 * @param[in] fifo fifoナンバー
	 * @return 受信処理成功の可否
	 */
	bool Receive(CAN_HandleTypeDef* hcan, uint8_t fifo){
		if(hcan == _hcan){
			RxDataType rx_data;
			if (HAL_CAN_GetRxMessage(
					hcan,
					fifo == 0 ? CAN_RX_FIFO0 : CAN_RX_FIFO1,
					&rx_data.header,
					rx_data.data.data())
					== HAL_OK){
				_receive_callback[fifo].ReadCallbacks(rx_data);
				return true;
			}
		}
		return false;
	}

public:
	/*
	 * @param[in] hcan CANハンドル
	 */
	CANWrapper(CAN_HandleTypeDef* hcan) : _hcan(hcan){
		_tx_callback_it = callback::CAN_TxMailboxComplete.AddCallback(1, [this](CAN_HandleTypeDef* hcan){MailboxComplete(hcan);});
		_rx0_callback_it = callback::CAN_RxFifo0MsgPending.AddExclusiveCallback(1, [this](CAN_HandleTypeDef* hcan){return Receive(hcan, 0);});
		_rx1_callback_it = callback::CAN_RxFifo1MsgPending.AddExclusiveCallback(1, [this](CAN_HandleTypeDef* hcan){return Receive(hcan, 1);});
	}
	virtual ~CANWrapper(){
		callback::CAN_TxMailboxComplete.EraseCallback(_tx_callback_it);
		callback::CAN_RxFifo0MsgPending.EraseExclusiveCallback(_rx0_callback_it);
		callback::CAN_RxFifo1MsgPending.EraseExclusiveCallback(_rx1_callback_it);
	}


	/*copy禁止*/
	CANWrapper(const CANWrapper&) = delete;
	CANWrapper& operator=(const CANWrapper&) = delete;
	//CANWrapper(CANWrapper&&) = delete;
	//CANWrapper& operator=(CANWrapper&&) = delete;

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
		filter.FilterIdLow = remote_only << 1;
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
		return HAL_CAN_ActivateNotification(_hcan, CAN_DEFAULT_CALLBACK_FLAGS) == HAL_OK;
	}
	/**
	 * @brief 割り込みありCAN開始
	 * @param[in] active_its 割り込み設定フラグ
	 * @return ステート
	 */
	bool Start(uint32_t active_its){
		if (HAL_CAN_Start(_hcan) != HAL_OK)return false;
		return HAL_CAN_ActivateNotification(_hcan, active_its | CAN_DEFAULT_CALLBACK_FLAGS) == HAL_OK;
	}
	/**
	 * @brief デバッグモードCAN開始
	 * @return ステート
	 */
	bool StartOnDebug(){
		_hcan->Instance->MCR &= 0xFFFEFFFF;//debug
		return Start();
	}
	/**
	 * @brief 割り込みありデバッグモードCAN開始
	 * @param[in] active_its 割り込み設定フラグ
	 * @return ステート
	 */
	bool StartOnDebug(uint32_t active_its){
		_hcan->Instance->MCR &= 0xFFFEFFFF;//debug
		return Start(active_its);
	}

	/**
	 * @brief CAN停止
	 * @return ステート
	 */
	bool Stop(){
		return HAL_CAN_Stop(_hcan) == HAL_OK;
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

		CAN_TxHeaderTypeDef tx_header;

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.IDE = CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;
		tx_header.DLC = N;

		std::array<uint8_t, 8> copy{0};
		std::memcpy(copy.data(), data.data(), N);

		AddMailbox({tx_header, copy, 0});
	}
	/**
	 * @brief 標準データフレームで送信します
	 * @param[in] id CAN識別子
	 * @param[in] data 送信データ
	 * @attention data.size()>8であれば[8]以降のデータは無視されます
	 */
	void Transmit(uint32_t id, std::vector<uint8_t> data){
		CAN_TxHeaderTypeDef tx_header;

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = CAN_RTR_DATA;//CAN_RTR_DATA;
		tx_header.IDE = CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;
		tx_header.DLC = (data.size() < 8) ? data.size() : 8;

		std::array<uint8_t, 8> copy{0};
		std::memcpy(copy.data(), data.data(), tx_header.DLC);

		AddMailbox({tx_header, copy, 0});
	}
	/**
	 * @brief 標準データフレームで空送信します
	 * @param[in] id CAN識別子
	 */
	void Transmit(uint32_t id){
		CAN_TxHeaderTypeDef tx_header;

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = CAN_RTR_DATA;
		tx_header.IDE = CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;
		tx_header.DLC = 0;

		AddMailbox({tx_header, {}, 0});
	}

	/**
	 * @brief 標準リモートフレームを送信します
	 * @param[in] id CAN識別子
	 */
	void TransmitRemote(uint32_t id){
		CAN_TxHeaderTypeDef tx_header;

		tx_header.StdId = id;
		tx_header.ExtId = 0x00;
		tx_header.RTR = CAN_RTR_REMOTE;
		tx_header.IDE = CAN_ID_STD;
		tx_header.TransmitGlobalTime = DISABLE;
		tx_header.DLC = 0;

		AddMailbox({tx_header, {}, 0});
	}
	/**
	 * @brief 受信コールバック関数ハンドラ
	 */
	inline HALCallback<RxDataType>& FIFO0ReceiveCallback(){
		return _receive_callback[0];
	}
	/**
	 * @brief 受信コールバック関数ハンドラ
	 */
	inline HALCallback<RxDataType>& FIFO1ReceiveCallback(){
		return _receive_callback[1];
	}

	/**
	 * @brief 保有ハンドルのゲッター
	 * @return 保有しているCANハンドルの生ポインタ
	 */
	CAN_HandleTypeDef* GetHandle()const{
		return _hcan;
	}
};


constexpr bool operator<(const CANWrapper::TxDataType& a, const CANWrapper::TxDataType& b)noexcept{return a.header.StdId < b.header.StdId;}
constexpr bool operator>(const CANWrapper::TxDataType& a, const CANWrapper::TxDataType& b)noexcept{return a.header.StdId > b.header.StdId;}
constexpr bool operator<=(const CANWrapper::TxDataType& a, const CANWrapper::TxDataType& b)noexcept{return !(a.header.StdId > b.header.StdId);}
constexpr bool operator>=(const CANWrapper::TxDataType& a, const CANWrapper::TxDataType& b)noexcept{return !(a.header.StdId < b.header.StdId);}
}

