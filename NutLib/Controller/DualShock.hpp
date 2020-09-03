/*
 * デュアルショック、SBDBTのクラス
 *
 *まだ細部が確定していない
 */
#pragma once

#if false

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include <memory>
#include <numeric>

namespace nut{
class DualShock{
public:
	enum class Button : uint16_t{
		up 		= 0x0001,
		down	= 0x0002,
		right	= 0x0004,
		left	= 0x0008,
		triangle= 0x0010,
		cross	= 0x0020,
		circle	= 0x0040,
		start	= 0x0080,
		square	= 0x0100,
		L1		= 0x0200,
		L2		= 0x0400,
		R1		= 0x0800,
		R2		= 0x1000,
		L3		= 0x2000,
		R3		= 0x4000,
		select	= 0x8000
	};
	enum class AnalogPad : uint8_t{
		LX = 0x01,
		LY = 0x02,
		RX = 0x04,
		RY = 0x08,
	};

	class ButtonData{

	};


	/*user座標値*/
	static constexpr int8_t USE_ANAROG_MAX =		63;
	static constexpr int8_t USE_ANAROG_MIN =		-63;
	static constexpr int8_t USE_ANAROG_CENTER =		0;

	struct ButtonFlag{
		Button button;
		AnalogPad pad;
	};
	using ButtonData = std::array<uint8_t, 6>;

private:
	static constexpr uint8_t SBDBT_DATA_SIZE = 8;
	static constexpr uint8_t SBDBT_BUFF_SIZE = 16;
	static constexpr uint8_t SBDBT_BUFF_SIZE_D = SBDBT_BUFF_SIZE - 1;

	/*SBDBT受信値*/
	static constexpr int8_t ANAROG_MAX =		127;
	static constexpr int8_t ANAROG_MIN =		1;
	static constexpr int8_t ANAROG_CENTER =		64;

	/**/
	UART_HandleTypeDef* const _uart;
	TimeScheduler<void> _schduler;

	std::array<uint8_t, SBDBT_DATA_SIZE> _buttonn_data{0};
	std::array<uint8_t, SBDBT_DATA_SIZE> _last_buttonn_data{0};
	std::array<uint8_t, SBDBT_BUFF_SIZE> _buff{0};

	/*コールバック関数*/
	std::function<void(ButtonData)> _button_callback = nullptr;
	std::function<void()> _timeout_callback;



	//連続受信フラグ
	bool _continue_flag = false;

	/*
	 * タイムアウト関数
	 */
	void timeout(void){
		_continue_flag = false;
		{
			uint8_t num = 0;
			for(auto& data : _buttonn_data){
				data = (num == 0)? 0x80 : ((num < 3 || num == 7) ? 0 : ANAROG_CENTER);
				++num;
			}
		}

		_last_buttonn_data = _buttonn_data;
		if(_timeout_callback != nullptr)_timeout_callback();
	}


public:
	DualShock(UART_HandleTypeDef* huart, std::function<void()>&& callback_func, uint32_t time)
		: _uart(huart), _schduler([this]{timeout();}, time), _timeout_callback(callback_func){}
	virtual	~DualShock(){
		_schduler.Erase();
	}


	/*
	 * 初期化
	 */
	inline void Init(){
		HAL_UART_Receive_DMA(_uart, _buff.data(), SBDBT_BUFF_SIZE);
		_schduler.Set();
	}


	/*
	 * 受信関数
	 *  HAL_UART_RxHalfCpltCallback()内で呼び出すこと
	 */
	bool Receive(UART_HandleTypeDef *huart){
		bool state = false;

		if(huart == _uart){
			std::array<uint8_t, SBDBT_BUFF_SIZE> hold_buff(_buff);
			uint32_t ndtr_ptr = _uart->hdmarx->Instance->NDTR;

			//リングバッファ全探索
			for(uint8_t j = 0;j < SBDBT_BUFF_SIZE;++j){
				//ヘッダ探索
				if((hold_buff[j] == 0x80) && (((j + ndtr_ptr) & SBDBT_BUFF_SIZE_D) < SBDBT_DATA_SIZE)){//受信中途データの場合はじく
					std::array<uint8_t, SBDBT_DATA_SIZE> data;
					uint8_t check_sum = 0;

					//バッファ移し
					{
						uint8_t i = 0;
						for(auto& d : data){
							d = hold_buff[(j + i) & SBDBT_BUFF_SIZE_D];
							++i;
						}
					}

					for(auto it = std::next(data.begin(), 1), end = std::next(data.end(), -1);it != end;++it)
						check_sum += *it;


					//チェックサム確認
					if((check_sum &  0x7F) == (data.at(7) & 0x7F)){
						_buttonn_data = data;

						if((_buttonn_data != _last_buttonn_data) && (_button_callback != nullptr)){//エッジ検出&NULLチェック
							ButtonData edge_button;
							edge_button.at(0) = _buttonn_data.at(1) ^ _last_buttonn_data.at(1);
							edge_button.at(1) = _buttonn_data.at(2) ^ _last_buttonn_data.at(2);
							edge_button.at(2) = (_buttonn_data[3] != _last_buttonn_data[3]) ? 0xFF : 0;
							edge_button.at(3) = (_buttonn_data[4] != _last_buttonn_data[4]) ? 0xFF : 0;
							edge_button.at(4) = (_buttonn_data[5] != _last_buttonn_data[5]) ? 0xFF : 0;
							edge_button.at(5) = (_buttonn_data[6] != _last_buttonn_data[6]) ? 0xFF : 0;

							_continue_flag = true;
							_button_callback(edge_button);
						}else
							_continue_flag = true;

						_last_buttonn_data = _buttonn_data;
						state = true;
						_schduler.Reset();
						break;
					}
				}
			}
		}
		return state;
	}


	/*
	 * コントローラのボタン割り込み
	 */
	inline void set_sbdbt_Callback(std::function<void(ButtonData)>&& callback_func){
		_button_callback = callback_func;
	}

	/*
	 * ゲッター
	 */
	ButtonData GetButtonData(){
		ButtonData button;
		button.at(0) = _buttonn_data.at(1);
		button.at(1) = _buttonn_data.at(2);
	}


	/*
	 * ボタン読み出し
	 */
	static bool GetButton(ButtonData data, Button button) const{
		uint16_t all_data;
		memcpy(&all_data, &data[0], 2);

		return all_data & (1 << static_cast<uint8_t>(button));
	}
	static int8_t GetAnalogPad(ButtonData data, AnalogPad analog_pad) const{
		return data[2 + static_cast<uint8_t>(analog_pad)];
	}
};

}

#endif
