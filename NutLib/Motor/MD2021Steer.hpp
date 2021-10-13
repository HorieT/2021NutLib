/**
 * @brief 2021モータードライバ
 *
 */
#pragma once

#include "../Global.hpp"
#include "SteerDriver.hpp"
#include <memory>

namespace nut{
/**
 * @brief MD2021 Steer専用クラス
 * @details ステアモードで使用するための専用クラスです
 * 通常のMD2021と通衝突が起きないように注意してください.
 */
class MD2021Steer : public SteerDriver{
private:
	using CANRxIt = CANWrapper::RxExCallbackIt;

	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _steer_id;
	const uint8_t _user_id;
	std::array<uint8_t, 8> _rx_data;

	const uint8_t _can_fifo;
	CANRxIt _can_callback;

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

		_can->Transmit<N>(_steer_id << can_protocol::DEVICE_NUM_SHIFT | static_cast<uint16_t>(type) , data);
	}


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		std::array<uint8_t, 8> data;
		std::memcpy(&data[0], &_target_norm, 4);
		std::memcpy(&data[4], &_target_rad, 4);

		SendData<8>(can_protocol::motor::DataType::steerControlInput, data);
	}

	/**
	 * @brief CAN受信関数
	 * @param[in] rx_data 受信データ
	 * @return 受信パケットがこのモータに該当するかどうか
	 */
	bool ReadCanData(CANWrapper::RxDataType rx_data){
		if(rx_data.header.RTR == CAN_RTR_DATA && (_steer_id == rx_data.data[0])){
			_rx_data = rx_data.data;
			return true;
		}
		return false;
	}

public:
	/**
	 * @brief コンストラク
	 * @param[in] period 周期
	 * @param[in] can canのヘルパインスタンス
	 * @param[in] use_fifo CANの使用FIFO（0 or 1）
	 * @param[in] steer_id MDのID
	 * @param[in] user_id 自分のID
	 */
	MD2021Steer(MilliSecond<uint32_t> period, const std::shared_ptr<CANWrapper>& can, uint8_t use_fifo, uint8_t steer_id, uint8_t user_id)
		: SteerDriver(period), _can(can), _steer_id(steer_id), _user_id(user_id),  _can_fifo(use_fifo){
	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~MD2021Steer(){
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
	virtual bool Start(bool current_control = false) override{
		_target_norm = 0.0f;
		_target_rad = 0.0f;
		std::array<uint8_t, 2> data{
			_user_id,
			static_cast<uint8_t>(current_control ?
					can_protocol::motor::SpecialOperation::steerStartCurrent :
					can_protocol::motor::SpecialOperation::steerStart)
		};
		SendData<2>(can_protocol::motor::DataType::specialOperation, data);

		_scheduler.Set();
		return true;
	}
	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() override{
		_scheduler.Erase();
		SendData<2>(
				can_protocol::motor::DataType::specialOperation,
				std::array<uint8_t, 2>{_user_id, static_cast<uint8_t>(can_protocol::motor::SpecialOperation::stop)}
		);
	}



	/**
	 *
	 */
	template<uint8_t N>
	static std::array<std::shared_ptr<SteerDriver>, N> MakeSteerArray(uint8_t driver_num_start, MilliSecond<uint32_t> period, const std::shared_ptr<CANWrapper>& can, uint8_t fifo, uint8_t my_id){
		std::array<std::shared_ptr<SteerDriver>, N> steers;

		uint8_t i = driver_num_start;
		for(auto& steer : steers){
		  steer = std::static_pointer_cast<SteerDriver>(
				  std::make_shared<MD2021Steer>(
						  period,
						  can,
						  fifo,
						  can_protocol::MakeDeviceID(can_protocol::Device::motorDriver, i),
						  my_id));
		  i+=2;
		}
		return steers;
	}

	/**
	 * @brief 制御IDの変更
	 * @attention Init()の呼び出し前、またはDeinit()の呼び出し後以外は無意味
	 * @return ID変更したか
	 */
	bool ResetUserID(uint8_t id){
		if(_is_init)return false;
		const_cast<uint8_t&>(_user_id) = id;
		return true;
	}
};
}
