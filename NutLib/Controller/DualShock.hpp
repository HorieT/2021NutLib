/*
 * 繝�繝･繧｢繝ｫ繧ｷ繝ｧ繝�繧ｯ縲ヾBDBT縺ｮ繧ｯ繝ｩ繧ｹ
 *
 *縺ｾ縺�邏ｰ驛ｨ縺檎｢ｺ螳壹＠縺ｦ縺�縺ｪ縺�
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include <memory>
#include <numeric>

namespace nut{
class DualShock{
public:
	using Button = uint32_t;
	/*
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
*/


	/*user蠎ｧ讓吝�､*/
	static constexpr int8_t USE_ANAROG_MAX =		63;
	static constexpr int8_t USE_ANAROG_MIN =		-63;
	static constexpr int8_t USE_ANAROG_CENTER =		0;

	/*荳倶ｽ�*/
	static constexpr Button PS3_UP =		0x0001;
	static constexpr Button PS3_DOWN =		0x0002;
	static constexpr Button PS3_RIGHT =		0x0004;
	static constexpr Button PS3_LEFT =		0x0008;
	static constexpr Button PS3_TRIANGLE =	0x0010;
	static constexpr Button PS3_CROSS =		0x0020;
	static constexpr Button PS3_CIRCLE =  	0x0040;
	static constexpr Button PS3_START =		0x0080;
	/*荳贋ｽ�*/
	static constexpr Button PS3_SQUARE = 	0x0100;
	static constexpr Button PS3_L1 =		0x0200;
	static constexpr Button PS3_L2 =		0x0400;
	static constexpr Button PS3_R1 = 		0x0800;
	static constexpr Button PS3_R2 = 		0x1000;
	static constexpr Button PS3_L3 =		0x2000;
	static constexpr Button PS3_R3 =		0x4000;
	static constexpr Button PS3_SELECT = 	0x8000;
	//萓ｿ螳應ｸ翫�ｮ螳夂ｾｩ
	static constexpr Button PS3_ANALOG_LX =		0x000F0000;
	static constexpr Button PS3_ANALOG_LY = 	0x00F00000;
	static constexpr Button PS3_ANALOG_RX =		0x0F000000;
	static constexpr Button PS3_ANALOG_RY = 	0xF0000000;

private:
	static constexpr uint8_t SBDBT_DATA_SIZE = 8;
	static constexpr uint8_t SBDBT_BUFF_SIZE = 16;
	static constexpr uint8_t SBDBT_BUFF_SIZE_D = SBDBT_BUFF_SIZE - 1;

	/*SBDBT蜿嶺ｿ｡蛟､*/
	static constexpr int8_t ANAROG_MAX =		127;
	static constexpr int8_t ANAROG_MIN =		1;
	static constexpr int8_t ANAROG_CENTER =		64;

	/**/
	UART_HandleTypeDef* const _uart;
	TimeScheduler<void> _schduler;

	std::array<uint8_t, SBDBT_DATA_SIZE> _buttonn_data{0};
	std::array<uint8_t, SBDBT_DATA_SIZE> _last_buttonn_data{0};
	std::array<uint8_t, SBDBT_BUFF_SIZE> _buff{0};

	/*繧ｳ繝ｼ繝ｫ繝舌ャ繧ｯ髢｢謨ｰ*/
	std::function<void(Button)> _button_callback = nullptr;
	std::function<void()> _timeout_callback;



	//騾｣邯壼女菫｡繝輔Λ繧ｰ
	bool _continue_flag = false;

	/*
	 * 繧ｿ繧､繝�繧｢繧ｦ繝磯未謨ｰ
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
	 * 蛻晄悄蛹�
	 */
	inline void Init(){
		HAL_UART_Receive_DMA(_uart, _buff.data(), SBDBT_BUFF_SIZE);
		_schduler.Set();
	}


	/*
	 * 蜿嶺ｿ｡髢｢謨ｰﾂ�
	 *  HAL_UART_RxHalfCpltCallback()蜀�縺ｧ蜻ｼ縺ｳ蜃ｺ縺吶％縺ｨ
	 */
	bool Receive(UART_HandleTypeDef *huart){
		bool state = false;

		if(huart == _uart){
			std::array<uint8_t, SBDBT_BUFF_SIZE> hold_buff(_buff);
			uint32_t ndtr_ptr = _uart->hdmarx->Instance->NDTR;

			//繝ｪ繝ｳ繧ｰ繝舌ャ繝輔ぃ蜈ｨ謗｢邏｢
			for(uint8_t j = 0;j < SBDBT_BUFF_SIZE;++j){
				//繝倥ャ繝�謗｢邏｢
				if((hold_buff[j] == 0x80) && (((j + ndtr_ptr) & SBDBT_BUFF_SIZE_D) < SBDBT_DATA_SIZE)){//蜿嶺ｿ｡荳ｭ騾斐ョ繝ｼ繧ｿ縺ｮ蝣ｴ蜷医�ｯ縺倥￥
					std::array<uint8_t, SBDBT_DATA_SIZE> data;
					uint8_t check_sum = 0;

					//繝舌ャ繝輔ぃ遘ｻ縺�
					{
						uint8_t i = 0;
						for(auto& d : data){
							d = hold_buff[(j + i) & SBDBT_BUFF_SIZE_D];
							++i;
						}
					}

					for(auto it = std::next(data.begin(), 1), end = std::next(data.end(), -1);it != end;++it)
						check_sum += *it;


					//繝√ぉ繝�繧ｯ繧ｵ繝�遒ｺ隱�
					if((check_sum &  0x7F) == (data.at(7) & 0x7F)){
						_buttonn_data = data;

						if((_buttonn_data != _last_buttonn_data) && (_button_callback != nullptr)){//繧ｨ繝�繧ｸ讀懷�ｺ&NULL繝√ぉ繝�繧ｯ
							Button edge_button =
							(uint32_t)_buttonn_data.at(2) |
							((uint32_t)_buttonn_data.at(1) << 8) |
							((_buttonn_data.at(3) != _last_buttonn_data.at(3)) ? (0x0F << 16) : 0) |
							((_buttonn_data.at(4) != _last_buttonn_data.at(4)) ? (0xF0 << 16) : 0) |
							((_buttonn_data.at(5) != _last_buttonn_data.at(5)) ? (0x0F << 24) : 0) |
							((_buttonn_data.at(6) != _last_buttonn_data.at(6)) ? (0xF0 << 24) : 0);

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
	 * 繧ｳ繝ｳ繝医Ο繝ｼ繝ｩ縺ｮ繝懊ち繝ｳ蜑ｲ繧願ｾｼ縺ｿ
	 */
	inline void set_sbdbt_Callback(std::function<void(Button)>&& callback_func){
		_button_callback = callback_func;
	}

	/*
	 * 繧ｲ繝�繧ｿ繝ｼ
	 */
	int8_t GetButtonData(Button button){
		int8_t data = 0;

		if(!(((button & 0xFFFF) != 0) && ((button & 0xFFFF0000) != 0)) && _continue_flag){
			if(button & 0xFFFF){
				data = ((((uint32_t)_buttonn_data.at(2) |((uint32_t)_buttonn_data.at(1) << 8)) & button) != 0);
			}else if(button & 0xFFFF0000){
				switch(button){
				case PS3_ANALOG_LX:
					data = _buttonn_data.at(3) - ANAROG_CENTER;
					break;
				case PS3_ANALOG_LY:
					data = ANAROG_MAX + 1 - (int8_t)_buttonn_data.at(4) - ANAROG_CENTER;
					break;
				case PS3_ANALOG_RX:
					data = _buttonn_data.at(5) - ANAROG_CENTER;
					break;
				case PS3_ANALOG_RY:
					data = _buttonn_data.at(6) - ANAROG_CENTER;
					break;
				default:
					break;
				}
			}
		}
		return data;
	}
};

}

