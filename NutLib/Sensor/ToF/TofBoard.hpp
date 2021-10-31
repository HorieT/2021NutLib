/**
 *
 * @author 	Hasegawa
 * 			Horie
 * @date 	2020/12/02
 * 			2021/02/08
 */
#pragma once

#include "../../Global.hpp"
#include "../../CANWrapper.hpp"
#include "../../CANBusProtocol.hpp"
#include "../../TimeScheduler.hpp"

namespace nut{
/**
 * @brief Tof通信変換基板クラス
 * @details VL53L0xのI2C-CAN変換基板と通信する
 */
class TofBoard final{
private:
	static constexpr uint8_t DEFAULT_CAN_PRIORITY = 16;
	static constexpr MilliSecond<uint32_t> REGULAR_TIMEOUT = 300;
	static constexpr MilliSecond<uint32_t> REMOTE_TIMEOUT = 100;
	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _my_id = 0;
	const uint8_t _id = 0;

	bool _is_init = false;
	bool _is_regular_request = false;
	MilliMeter<uint16_t> _distance = 65535;

	CANWrapper::RxExCallbackIt _recive_it;


	/**
	 * @brief 受信関数
	 *
	 */
	bool Recive(CANWrapper::RxDataType rx_data){
		if(rx_data.header.RTR != CAN_RTR_DATA)return false;//not data frame
		if(rx_data.data[0] != _id)return false;
		if(can_protocol::GetDeviceID(rx_data.header.StdId) != _my_id)return false;

		switch(static_cast<can_protocol::micom::DataTypeTof>(can_protocol::GetDataType(rx_data.header.StdId))){
		case nut::can_protocol::micom::DataTypeTof::length:
		{
			uint16_t tmp;
			std::memcpy(&tmp, &rx_data.data[1], 2);
			_distance = tmp;
			break;
		}
		default:
			break;
		}
		return true;
	}

	/**
	 * @brief 命令送信
	 */
	void SendOperation(can_protocol::tof::SpecialOperation operation){
		if(!_is_init)return;
		std::array<uint8_t, 2> data{_my_id, static_cast<uint8_t>(operation)};
		_can->Transmit<2>(can_protocol::MakeCANID(_id, can_protocol::tof::DataType::specialOperation), data);
		_can->Transmit<2>(can_protocol::MakeCANID(_id, can_protocol::tof::DataType::specialOperation), data);
	}
public:
	/**
	 * @brief コンストラクタ
	 * @param[in] can canラッパ
	 */
	TofBoard(const std::shared_ptr<CANWrapper>& can) : _can(can){

	}
	/**
	 * @brief デストラクタ
	 */
	~TofBoard(){
		Deinit();
	}


	/**
	 * @brief 初期化
	 */
	void Init(uint8_t my_id, uint8_t device_num, uint8_t can_priority = DEFAULT_CAN_PRIORITY){
		if(_is_init)return;
		_is_init = true;
		const_cast<uint8_t&>(_my_id) = my_id;
		const_cast<uint8_t&>(_id) = can_protocol::MakeDeviceID(can_protocol::Device::tof, device_num);
		_recive_it = _can->FIFO0ReceiveCallback().AddExclusiveCallback(can_priority, [this](CANWrapper::RxDataType rx_data){return Recive(rx_data);});
	}

	/**
	 * @brief 初期化解除
	 */
	void Deinit(){
		if(!_is_init)return;
		_can->FIFO0ReceiveCallback().EraseExclusiveCallback(_recive_it);
		_is_init = false;
	}

	/**
	 * @brief レギュラモード開始
	 */
	void RegularModeStart(){
		if(!_is_init || _is_regular_request)return;
		SendOperation(can_protocol::tof::SpecialOperation::regularModeStart);
		_is_regular_request = true;
	}
	/**
	 * @brief レギュラモード停止
	 */
	void RegularModeStop(){
		if(!_is_init || !_is_regular_request)return;
		SendOperation(can_protocol::tof::SpecialOperation::regularModeStop);
		_is_regular_request = false;
	}

	/**
	 * @brief 距離取得
	 * @return 距離
	 */
	MilliMeter<uint16_t> Distance()const{
		return _distance;
	}
};
}
