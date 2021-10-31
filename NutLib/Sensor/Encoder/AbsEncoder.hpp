/**
 * @file AbsEncoder.hpp
 * @brief アブソリュート型エンコーダ基底
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Encoder.hpp"
#include "../../HALCallbacks/UART.hpp"
#include "../../RS485.hpp"
#include <bitset>
#include <array>

namespace nut{
/**
 * @brief アブソリュート型エンコーダクラス
 * @attention というのは偽りで現状AMT20系列専用クラス.
 * そのうちちゃんと派生させます(誰かやって)//
 * それと1つの485バスで複数稼働させることも考慮してないです
 *
 */
class AbsEncoder : public Encoder{
private:
	using RxCallbackIt = decltype(callback::UART_RxHalfComplete)::ExCallbackIterator;
	static constexpr uint32_t BIT12 = std::pow(2, 12);
	static constexpr uint32_t BIT14 = std::pow(2, 14);


	UART_HandleTypeDef* const _huart;
	GPIO_TypeDef* const _port;
	const uint16_t _pin;
	uint8_t _buff[2] = {0};
	int16_t _last_bit = 0;
	int16_t _bit = 0;
	TimeScheduler<void> _scheduler;
	RxCallbackIt _rx_it;

	/**
	 * @brief 受信関数
	 * @param[in] huart uartハンドル
	 * @return 受信処理成功の可否
	 */
	bool Receive(UART_HandleTypeDef *huart){
		if(huart == _huart){
			__disable_irq();
			std::bitset<8> h_byte(_buff[1]);
			std::bitset<8> l_byte(_buff[0]);
			_buff[0] = 0;
			_buff[1] = 0;
			__enable_irq();
			if(h_byte[7] == (h_byte[5] ^ h_byte[3] ^ h_byte[1] ^ l_byte[7] ^ l_byte[5] ^ l_byte[3] ^ l_byte[1]))return false;
			if(h_byte[6] == (h_byte[4] ^ h_byte[2] ^ h_byte[0] ^ l_byte[6] ^ l_byte[4] ^ l_byte[2] ^ l_byte[0]))return false;
			uint32_t tmp = (l_byte.to_ulong() | (h_byte.to_ulong() << 8)) & 0x3FFF;
			if(_resolution == BIT12)tmp = tmp >> 2;
			_bit = (tmp > _resolution/2 ? tmp -  static_cast<int16_t>(_resolution) : tmp);
			return true;
		}
		return false;
	}


public:
	/**
	 * @brief コンストラクタ
	 * @param[in] resolution 分解能
	 * @attention 12bit,14bitにしか対応してません
	 * @param[in] huart RS485用uartハンドル
	 * @param[in] port RS485用enableGPIO
	 * @param[in] pin RS485用enableGPIO
	 */
	AbsEncoder(uint32_t resolution, UART_HandleTypeDef* huart, GPIO_TypeDef* port, uint16_t pin)
		: Encoder(resolution), _huart(huart), _port(port), _pin(pin), _scheduler([this]{Reqest();}, 1){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~AbsEncoder(){

	}



	/**
	 * @brief 初期化
	 */
	virtual void Init() override{
		if(_is_init)return;
		_is_init = true;
		_rx_it = callback::UART_RxComplete.AddExclusiveCallback(1, [this](UART_HandleTypeDef* huart){return Receive(huart);});
		_scheduler.Set();
	}

	/**
	 * @brief 初期化
	 */
	virtual void Deinit() override{
		if(!_is_init)return;
		_is_init = false;
		callback::UART_RxComplete.EraseExclusiveCallback(_rx_it);
		_scheduler.Erase();
	}

	/**
	 * @brief カウントリセット
	 */
	virtual void Reset() override{
		std::array<uint8_t, 2> data{0x56, 0x5E};
		HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
		HAL_UART_Transmit(_huart, data.data(), 2, 2);
		HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
	}

	/**
	 * @brief 角度取得
	 * @return 角度
	 */
	virtual Radian<float> GetRad() override{
		return (_bit - static_cast<int32_t>(_last_bit)) * M_2PI_f / static_cast<float>(_resolution);
	}

	/**
	 * @brief 角度取得&カウントリセット
	 * @details 周期角度取得精度を上げるためのもの
	 * @return 角度
	 */
	virtual Radian<float> GetRadAndReset() override{
		float value = (_bit - static_cast<int32_t>(_last_bit)) * M_2PI_f / static_cast<float>(_resolution);
		_last_bit = _bit;
		return (value > M_PI) ? value - M_PI : ((value < -M_PI) ? value + M_PI : value);
	}


	/**
	 * @brief 角度読み出しリクエスト
	 */
	void Reqest(){
		uint8_t addr = 0x54;
		HAL_UART_AbortReceive(_huart);
		_buff[0] = 0;
		_buff[1] = 0;
		HAL_UART_Receive_DMA(_huart, reinterpret_cast<uint8_t*>(&_buff), 2);
		HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET);
		HAL_UART_Transmit(_huart, &addr, 1, 1);
		HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET);
	}
};
}
