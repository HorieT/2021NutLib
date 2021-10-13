/**
 * @file PowerSupply2021.hpp
 * @brief 2021学ロボ用電源制御基板
 * @details 基板詳細は該当ファームウェアのドキュメントまたはesaを参照してください
 * @author Horie
 * @date 2020/10
 */
#pragma once

#include "../Global.hpp"
#include "../CANWrapper.hpp"
#include "../CANBusProtocol.hpp"
#include <memory>


namespace nut{
/**
 * @brief 2021gakurobo用電源制御基板
 */
class PowerSupply2021{
private:
	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _device_id;
	const uint8_t _user_id;
	std::array<uint8_t, 8> _rx_data;

public:
	PowerSupply2021(const std::shared_ptr<CANWrapper>& can, uint8_t device_num, uint8_t user_id = can_protocol::MakeDeviceID(can_protocol::Device::microcomputer)) :
		_can(can), _device_id(device_num), _user_id(user_id){

	}

	virtual ~PowerSupply2021(){

	}



	/**
	 * @brief 通電開始
	 */
	void Start(){
		  std::array<uint8_t, 2> ps_data{_user_id, 0x01};
		  _can->Transmit<2>(
				  can_protocol::MakeCANID(
						  can_protocol::Device::powerSupplay,
						  _device_id,
						  static_cast<uint8_t>(can_protocol::power::DataType::fetState)
				  ),
				  ps_data
		  );
	}
	/**
	 * @brief 通電停止
	 */
	void Stop(){
		  std::array<uint8_t, 2> ps_data{_user_id, 0x00};
		  _can->Transmit<2>(
				  can_protocol::MakeCANID(
						  can_protocol::Device::powerSupplay,
						  _device_id,
						  static_cast<uint8_t>(can_protocol::power::DataType::fetState)
				  ),
				  ps_data
		  );
	}


	/**
	 * @brief 制御IDの変更
	 * @attention Init()の呼び出し前、またはDeinit()の呼び出し後以外は無意味
	 * @return ID変更したか
	 */
	void ResetUserID(uint8_t id){
		const_cast<uint8_t&>(_user_id) = id;
	}
};
}
