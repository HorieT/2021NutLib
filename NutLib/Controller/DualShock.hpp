/**
 * @file DualShock.hpp
 * @brief DualshockをSBDBT5Vを通して受信する
 * @author Horie
 * @date 2020/9
 * @attention インターフェースが変わるような大幅更新を行う予定なので注意
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include <memory>
#include <numeric>

namespace nut{
/**
 * @brief DualshockをSBDBT5Vを通して受信するクラス
 */
class DualShock{
public:
	using Button = uint32_t;//!< ボタンの型エイリアス
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


	/*user用のアナログパッド座標*/
	static constexpr int8_t USE_ANAROG_MAX =		63;//!< アナログパッド座標最大値
	static constexpr int8_t USE_ANAROG_MIN =		-63;//!< アナログパッド座標最小値
	static constexpr int8_t USE_ANAROG_CENTER =		0;//!< アナログパッド座標中央値

	/*ボタン下位*/
	static constexpr Button PS3_UP =		0x0001;//!< 十字キー上
	static constexpr Button PS3_DOWN =		0x0002;//!< 十字キー下
	static constexpr Button PS3_RIGHT =		0x0004;//!< 十字キー右
	static constexpr Button PS3_LEFT =		0x0008;//!< 十字キー左
	static constexpr Button PS3_TRIANGLE =	0x0010;//!< 三角ボタン
	static constexpr Button PS3_CROSS =		0x0020;//!< バツボタン
	static constexpr Button PS3_CIRCLE =  	0x0040;//!< 丸ボタン
	static constexpr Button PS3_START =		0x0080;//!< STARTボタン
	/*ボタン上位*/
	static constexpr Button PS3_SQUARE = 	0x0100;//!< 四角ボタン
	static constexpr Button PS3_L1 =		0x0200;//!< L1ボタン
	static constexpr Button PS3_L2 =		0x0400;//!< L2ボタン
	static constexpr Button PS3_R1 = 		0x0800;//!< R1ボタン
	static constexpr Button PS3_R2 = 		0x1000;//!< R2ボタン
	static constexpr Button PS3_L3 =		0x2000;//!< L3ボタン
	static constexpr Button PS3_R3 =		0x4000;//!< R3ボタン
	static constexpr Button PS3_SELECT = 	0x8000;//!< SELECTボタン
	/*アナログパッド*/
	static constexpr Button PS3_ANALOG_LX =		0x000F0000;//!< アナログパッド左x座標
	static constexpr Button PS3_ANALOG_LY = 	0x00F00000;//!< アナログパッド左y座標
	static constexpr Button PS3_ANALOG_RX =		0x0F000000;//!< アナログパッド右ｘ座標
	static constexpr Button PS3_ANALOG_RY = 	0xF0000000;//!< アナログパッド右y座標

private:
	static constexpr uint8_t SBDBT_DATA_SIZE = 8;//!< SBDBT5V電文長
	static constexpr uint8_t SBDBT_BUFF_SIZE = 16;//!< SBDBT5V電文バッファサイズ
	static constexpr uint8_t SBDBT_BUFF_SIZE_D = SBDBT_BUFF_SIZE - 1;//!< SBDBT5V電文バッファサイズ-1

	/*SBDBTのアナログパッド値*/
	static constexpr int16_t ANAROG_MAX =		127;
	static constexpr int16_t ANAROG_MIN =		1;
	static constexpr int16_t ANAROG_CENTER =		64;


	UART_HandleTypeDef* const _uart;
	TimeScheduler<void> _schduler;

	std::array<uint8_t, SBDBT_DATA_SIZE> _buttonn_data{0};
	std::array<uint8_t, SBDBT_DATA_SIZE> _last_buttonn_data{0};
	std::array<uint8_t, SBDBT_BUFF_SIZE> _buff{0};

	/*コールバック関数*/
	std::function<void(Button)> _button_callback = nullptr;
	std::function<void()> _timeout_callback;



	//連続受信フラグ
	bool _continue_flag = false;

	/**
	 * @brief タイムアウト関数
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
	/**
	 * @brief コンストラクタ
	 * @param[in] huart uartハンドル
	 * @details uartは事前に通信仕様通りの設定を行い、DMA設定でCircularにしてください
	 * @param[in] callback_func タイムアウトコールバック関数
	 * @param[in] time タイムアウト時間[ms]
	 */
	DualShock(UART_HandleTypeDef* huart, std::function<void()>&& callback_func, uint32_t time)
		: _uart(huart), _schduler([this]{timeout();}, time), _timeout_callback(callback_func){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual	~DualShock(){
		_schduler.Erase();
	}



	/**
	 * @brief 初期化関数
	 */
	inline void Init(){
		HAL_UART_Receive_DMA(_uart, _buff.data(), SBDBT_BUFF_SIZE);
		_schduler.Set();
	}


	/**
	 * @brief 受信関数
	 * @details HAL_UART_RxHalfCpltCallback()内で呼び出してください
	 * @param[in] huart uartハンドル
	 * @return 受信処理成功の可否
	 */
	bool Receive(UART_HandleTypeDef *huart){
		bool state = false;

		if(huart == _uart){
			std::array<uint8_t, SBDBT_BUFF_SIZE> hold_buff(_buff);
			uint32_t ndtr_ptr = _uart->hdmarx->Instance->NDTR;

			//巡回探査
			for(uint8_t j = 0;j < SBDBT_BUFF_SIZE;++j){
				//ヘッダ捜索
				if((hold_buff[j] == 0x80) && (((j + ndtr_ptr) & SBDBT_BUFF_SIZE_D) < SBDBT_DATA_SIZE)){
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


					//チェックサム
					if((check_sum &  0x7F) == (data.at(7) & 0x7F)){
						_buttonn_data = data;

						if((_buttonn_data != _last_buttonn_data) && (_button_callback != nullptr)){//エッジ検出
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


	/**
	 * @brief Buttonエッジでのコールバック関数セット
	 * @param[in] callback_func コールバック関数
	 */
	inline void SetButtonCallback(std::function<void(Button)>&& callback_func){
		_button_callback = callback_func;
	}

	/**
	 * @brief Buttonデータの取得
	 * @param[in] button 目的のボタン、またはアナログパッド
	 * @details ボタンはor演算で複数指定できます
	 * @return buttonがボタンであればbool値、アナログパッドであれば座標が返ります
	 * @details 複数ボタンが指定されていればそれらのand演算値が返ります
	 * @attention ボタンとアナログパッドを同時指定するとfalseが返ります
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
					data = ANAROG_MAX + 1 - static_cast<int16_t>(_buttonn_data.at(4)) - ANAROG_CENTER;
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
		return static_cast<int8_t>(data);
	}

	std::array<uint8_t, SBDBT_DATA_SIZE> GetData(){
		return _buttonn_data;
	}
};

}

