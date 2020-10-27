/**
 * @brief 2021モータードライバ
 *
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include "../CANWrapper.hpp"
#include "../CANBusProtocol.hpp"
#include <memory>

namespace nut{
class MD2021Steer{
private:
	//Period control
	TimeScheduler<void> _scheduler;

	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _steer_id;
	const uint8_t _user_id;
	std::array<uint8_t, 8> _rx_data;

	/* target */
	float _target_norm = 0.0f;
	float _target_rad = 0.0f;


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
	virtual void ScheduleTask() {
		std::array<uint8_t, 8> data;
		std::memcpy(&data[0], &_target_norm, 4);
		std::memcpy(&data[4], &_target_rad, 4);

		SendData<8>(can_protocol::motor::DataType::steerControlInput, data);
	}

public:
	/**
	 * @brief コンストラク
	 * @param[in] period 周期
	 * @param[in] can canのヘルパインスタンス
	 * @param[in] id 5bitのモータid
	 */
	MD2021Steer(uint32_t period, std::shared_ptr<CANWrapper> can, uint8_t steer_id, uint8_t user_id)
		: _scheduler([this]{ScheduleTask();}, period), _can(can), _steer_id(steer_id), _user_id(user_id){

	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~MD2021Steer(){
		Stop();
	}




	/**
	 * @brief 制御スタート
	 */
	virtual bool Start() {
		_target_norm = 0.0f;
		_target_rad = 0.0f;
		SendData<2>(
				can_protocol::motor::DataType::specialOperation,
				std::array<uint8_t, 2>{_user_id, static_cast<uint8_t>(can_protocol::motor::SpecialOperation::steerStart)}
		);

		_scheduler.Set();
		return true;
	}
	/**
	 * @brief 制御ストップ
	 */
	virtual void Stop() {
		_scheduler.Erase();
		SendData<2>(
				can_protocol::motor::DataType::specialOperation,
				std::array<uint8_t, 2>{_user_id, static_cast<uint8_t>(can_protocol::motor::SpecialOperation::stop)}
		);
	}


	virtual bool SetMove(float norm, float rad){
		_target_norm = norm;
		_target_rad = rad;

		return true;
	}

	/**
	 * @brief CAN受信関数
	 * @details HAL_CAN_RxFifo0MsgPendingCallback()またはHAL_CAN_RxFifo1MsgPendingCallback()内で呼び出してください
	 * @param[in] hcan canハンドル
	 * @param[in] RxHeader 受信ヘッダ
	 * @param[in] data 受信データ
	 * @return 受信パケットがこのモータに該当するかどうか
	 */
	bool ReadCanData(CAN_HandleTypeDef* hcan, const CAN_RxHeaderTypeDef& RxHeader, const std::array<uint8_t, 8> data){
		if(hcan == _can->GetHandle()){
			if(RxHeader.RTR == CAN_RTR_DATA && (_steer_id == data[0])){
				_rx_data = data;
				return true;
			}
		}
		return false;
	}
};
}
