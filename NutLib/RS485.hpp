/**
 * @brief RS485インターフェース
 * @attention まだ！！！！！！！！！！！！！！！！！！！！
 */
#pragma once

#include "Global.hpp"
#include <vector>
#include <queue>
#include <map>
#include <functional>

namespace nut{

#if false
/**
 * @brief RS485インターフェースクラス
 * @attention 未完成！！！！！
 *
 */
class RS485{
public:
	using Callback = std::function<void(std::vector<uint8_t>)>;

private:

	/**
	 * @brief データ送信＆要求構造体
	 * @details callbackが空であれば要求データはありません
	 */
	struct DataRequest{
		std::vector<uint8_t> request_data;
		uint32_t receive_size;
		Callback callback;
		uint32_t receive_timeout;
	};


	UART_HandleTypeDef* const _huart;
	GPIO_TypeDef* const _port;
	const uint16_t _pin;
	TIM_HandleTypeDef* const _htim;
	std::queue<DataRequest> _tx_queue;//!<送信データキュー
	std::multimap<uint8_t, Callback> _callback_func;//!<受信コールバック
	Callback _request_callback;//!<受信コールバック
	std::vector<uint8_t> _read_data;
	bool _busy = false;


	bool Transmit(std::vector<uint8_t>& data){
		if(_busy)return true;
		_busy = true;
		HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
		HAL_UART_Transmit(_huart, data.data(), data.size(), 5);
		HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);

		return true;
	}

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] huart UARTハンドル
	 * @param[in] port RS485 enableピンのIOポート
	 * @param[in] pin RS485 enableピンのIOピン
	 * @param[in] htim 通信調停用1MHzタイマ
	 * @details カウント周期(リロード周期ではない)を1usに設定してください
	 */
	RS485(UART_HandleTypeDef* huart, GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef* htim)
		: _huart(huart), _port(port), _pin(pin), _htim(htim){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~RS485(){

	}


	/**
	 * @brief 受信コールバック関数のセット
	 */
	void SetCallback(Callback callback, uint8_t priority){
		_callback_func.insert(std::make_pair(priority, callback));
	}

	void TransmitData(std::vector<uint8_t> data){

	}

	void TransmitRequest(std::vector<uint8_t> request_data, size_t receive_size, Callback callback, uint16_t timeout_us){
		//_tx_queue.push({request_data, receive_size, callback, timeout_us});
		if(!_busy){
			_request_callback = callback;
			_read_data.resize(receive_size);
			Transmit(request_data);
			HAL_UART_Receive_DMA(_huart, _read_data.data(), receive_size);
			_htim->Instance->ARR = timeout_us;
			_htim->Instance->CNT = 0;
			HAL_TIM_Base_Start(_htim);

		}else{

		}
	}



	/**
	 *
	 */
	bool Receiver(UART_HandleTypeDef* huart){
		if(huart == _huart){

		}
		return false;
	}

	bool TimeoutCheck(TIM_HandleTypeDef* htim){
		if(htim != _htim)return false;


		HAL_UART_AbortReceive(_huart);
		return true;
	}
};
#endif
}
