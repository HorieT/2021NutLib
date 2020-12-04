/**
 * @brief 2021モータードライバ
 *
 */
#pragma once

#include "../Global.hpp"
#include "../CANWrapper.hpp"
#include "../CANBusProtocol.hpp"
#include "Motor.hpp"
#include <memory>

namespace nut{
class MD2021 : public Motor{
public:
	enum class Mode{
		incEnc,
		incEncCurrent,
		absEnc,
		absEncCurrent,
	};
private:
	using CANRxIt = CANWrapper::RxExCallbackIt;

	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _motor_id;
	const uint8_t _user_id;
	std::array<uint8_t, 8> _rx_data;

	const uint8_t _can_fifo;
	CANRxIt _can_callback;
	Mode _mode = Mode::incEnc;
	bool _is_debug = false;


protected:
	/**
	 * @brief CAN送信関数
	 * @tparam N 送信データサイズ
	 * @param[in] type 送信データタイプ
	 * @param[in] data 送信データ
	 */
	template<size_t N>
	void SendData(can_protocol::motor::DataType type, std::array<uint8_t, N> data) {
		static_assert(N <= 8,"Data size is over.");

		_can->Transmit<N>(_motor_id << can_protocol::DEVICE_NUM_SHIFT | static_cast<uint16_t>(type) , data);
	}

	void SendControl(can_protocol::motor::ControlInput input, float data){
		std::array<uint8_t, 6> send_data;

		send_data[0] = _user_id;
		send_data[1] = static_cast<uint8_t>(input);
		std::memcpy(&send_data[2], &data, 4);
		SendData(can_protocol::motor::DataType::controlInput, send_data);
	}
	void SendControl(can_protocol::motor::ControlInput input, float data, bool pol){
			std::array<uint8_t, 7> send_data;

			send_data[0] = _user_id;
			send_data[1] = static_cast<uint8_t>(input);
			std::memcpy(&send_data[2], &data, 4);
			send_data[6] = pol;
			SendData(can_protocol::motor::DataType::controlInput, send_data);
		}

	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		if(_move_type == MoveType::stop)return;
		switch(_move_type){
		case MoveType::duty:
			SendControl(can_protocol::motor::ControlInput::duty, _target_duty);
			break;
		case MoveType::radps:
			SendControl(can_protocol::motor::ControlInput::radps, _target_radps);
			break;
		case MoveType::radMulti:
			SendControl(can_protocol::motor::ControlInput::radMulti, _target_rad);
			break;
		case MoveType::radSingle:
			SendControl(can_protocol::motor::ControlInput::radSingle, _target_rad);
			break;
		case MoveType::radSinglePolarity:
			SendControl(can_protocol::motor::ControlInput::radSinglePolarity, _target_rad, _turn_polarity);
			break;
		case MoveType::currnet:
			SendControl(can_protocol::motor::ControlInput::current, _target_current);
			break;
		case MoveType::stop:
		default:
			return;
		}

	}
	/**
	 * @brief CAN受信関数
	 * @param[in] rx_data 受信データ
	 * @return 受信パケットがこのモータに該当するかどうか
	 */
	bool ReadCanData(CANWrapper::RxDataType rx_data){
		if(rx_data.header.RTR == CAN_RTR_DATA && (_motor_id == rx_data.data[0])){
			_rx_data = rx_data.data;
			return true;
		}
		return false;
	}

	/*
	 * @brief 変換メンバ関数
	 */
	can_protocol::motor::SpecialOperation ConvertOpration(Mode mode, bool debug = false){
		switch(mode){
		case Mode::incEnc:
			return debug ? can_protocol::motor::SpecialOperation::singleStartIncDebug : can_protocol::motor::SpecialOperation::singleStartInc;
		case Mode::incEncCurrent:
			return debug ? can_protocol::motor::SpecialOperation::singleStartIncCurrentDebug : can_protocol::motor::SpecialOperation::singleStartIncCurrent;
		case Mode::absEnc:
			return debug ? can_protocol::motor::SpecialOperation::singleStartAbsDebug : can_protocol::motor::SpecialOperation::singleStartAbs;
		case Mode::absEncCurrent:
			return debug ? can_protocol::motor::SpecialOperation::singleStartAbsCurrentDebug : can_protocol::motor::SpecialOperation::singleStartAbsCurrent;
		default:
			return can_protocol::motor::SpecialOperation::singleStartInc;
		}
	}



public:
	/**
	 * @brief コンストラク
	 * @param[in] period 周期
	 * @param[in] can canのヘルパインスタンス
	 * @param[in] use_fifo CANの使用FIFO（0 or 1）
	 * @param[in] motor_id モータのID
	 * @param[in] user_id 自分のID
	 */
	MD2021(uint32_t period, std::shared_ptr<CANWrapper> can, uint8_t use_fifo, uint8_t motor_id, uint8_t user_id)
		: Motor(period), _can(can), _motor_id(motor_id), _user_id(user_id), _can_fifo(use_fifo){
	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~MD2021(){
		Deinit();
	}


	/**
	 * @brief 初期化関数
	 */
	virtual void Init()override{
		if(_is_init)return;
		_is_init = true;
		if(_can_fifo == 0)_can_callback = _can->FIFO0ReceiveCallback().AddExclusiveCallback(5, [this](CANWrapper::RxDataType rx_data){return ReadCanData(rx_data);});
		else _can_callback = _can->FIFO1ReceiveCallback().AddExclusiveCallback(5, [this](CANWrapper::RxDataType rx_data){return ReadCanData(rx_data);});
	}
	/**
	 * @brief 非初期化関数
	 */
	virtual void Deinit()override{
		if(!_is_init)return;
		Stop();
		_is_init = false;
		if(_can_fifo == 0)_can->FIFO0ReceiveCallback().EraseExclusiveCallback(_can_callback);
		else _can->FIFO1ReceiveCallback().EraseExclusiveCallback(_can_callback);
	}


	/**
	 * @brief 制御スタート
	 */
	virtual bool Start() override{
		ResetTarget();
		_target_duty = 0.0f;
		SendData<2>(
				can_protocol::motor::DataType::specialOperation,
				std::array<uint8_t, 2>{_user_id, static_cast<uint8_t>(ConvertOpration(_mode, _is_debug))}
		);


		_move_type = MoveType::duty;
		_scheduler.Set();
		return true;
	}
	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() override{
		_move_type = MoveType::stop;
		_scheduler.Erase();
		SendData<2>(
				can_protocol::motor::DataType::specialOperation,
				std::array<uint8_t, 2>{_user_id, static_cast<uint8_t>(can_protocol::motor::SpecialOperation::stop)}
		);
		ResetController();
	}



	/**
	 * @brief 角度原点リセット
	 * @details 未実装です
	 * @return false
	 */
	virtual bool ResetRadOrigin(float rad) override{

		return false;
	}


	/**
	 * @brief 制御モード変更
	 */
	bool ChangeMode(Mode mode){
		if(_move_type != MoveType::stop)return false;
		_mode = mode;
		return true;
	}
};
}
