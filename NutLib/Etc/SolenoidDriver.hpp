/**
 * @brief 電磁弁どらいば
 * @author Horie
 * @date 2021/3
 */
#pragma once

#include "../Global.hpp"
#include "../TimeScheduler.hpp"
#include "../CANWrapper.hpp"
#include "../CANBusProtocol.hpp"

namespace nut{
/**
 * @brief 電磁弁(ソレノイド)ドライバ基板クラス
 */
class SolenoidDriver final{
private:
	const std::shared_ptr<CANWrapper> _can;
	const uint8_t _user_id;
	const uint8_t _id;

	static constexpr MilliSecond<uint32_t> CONTINUE_BLOCK_TIME = 50;
	enum LastSendType : uint8_t{
		And,
		Or,
		Copy
	};
	LastSendType _last_type = LastSendType::And;
	uint16_t _last_send_data = 0;
	bool _continue_flag = false;
	TimeScheduler<void> _continue_check{[this]{_continue_flag = false;}, CONTINUE_BLOCK_TIME};

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] can CANハンドル
	 * @param[in] dv_num デバイスナンバー
	 * @param[in] my_id 自己ID
	 */
	SolenoidDriver(const std::shared_ptr<CANWrapper>& can, uint8_t dv_num, uint8_t user_id = can_protocol::MakeDeviceID(can_protocol::Device::microcomputer)) :
	_can(can), _user_id(user_id), _id(can_protocol::MakeDeviceID(can_protocol::Device::solenoidValve, dv_num)){
		_continue_check.Set();
	}
	/**
	 * @brief デストラクタ
	 */
	~SolenoidDriver(){
		_continue_check.Erase();
	}

	/**
	 * @brief 出力AND
	 * @param[in] bit ビット列
	 * @return 出力セットできたか
	 */
	bool SetAnd(uint16_t bit){
		if(_last_type == LastSendType::And && bit == _last_send_data && _continue_flag)return false;
		std::array<uint8_t, 3> data{_user_id};
		std::memcpy(&data[1], &bit, 2);

		_last_send_data = bit;
		_last_type = LastSendType::And;
		_continue_flag = true;
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveSetAnd), data);
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveSetAnd), data);
		_continue_check.Reset();
		return true;
	}
	/**
	 * @brief 出力OR
	 * @param[in] bit ビット列
	 * @return 出力セットできたか
	 */
	bool SetOr(uint16_t bit){
		if(_last_type == LastSendType::Or && bit == _last_send_data && _continue_flag)return false;
		std::array<uint8_t, 3> data{_user_id};
		std::memcpy(&data[1], &bit, 2);

		_last_send_data = bit;
		_last_type = LastSendType::Or;
		_continue_flag = true;
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveSetOr), data);
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveSetOr), data);
		_continue_check.Reset();
		return true;
	}
	/**
	 * @brief コピー出力
	 * @param[in] bit ビット列
	 * @return 出力セットできたか
	 */
	bool SetCopy(uint16_t bit){
		if(_last_type == LastSendType::Copy && bit == _last_send_data && _continue_flag)return false;
		std::array<uint8_t, 3> data{_user_id};
		std::memcpy(&data[1], &bit, 2);

		_last_send_data = bit;
		_last_type = LastSendType::Copy;
		_continue_flag = true;
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveState), data);
		_can->Transmit<3>(can_protocol::MakeCANID(_id, can_protocol::sv::DataType::valveState), data);
		_continue_check.Reset();
		return true;
	}


	/**
	 * @brief 弁ON
	 * @param[in] bit ビット列
	 */
	void On(uint16_t bit){
		SetOr(bit);
	}
	/**
	 * @brief 一定時間弁ON
	 * @param[in] bit ビット列
	 * @param[in] time ON時間
	 */
	void On(uint16_t bit, MilliSecond<uint32_t> time){
		On(bit);
		TimeScheduler<void>::DelayCall([this, bit](){Off(bit);}, time);
	}
	/**
	 * @brief 一定時間弁ON
	 * @param[in] bit ビット列
	 * @param[in] time ON時間
	 * @param[in] callback OFF時呼び出しコールバック関数
	 */
	void On(uint16_t bit, MilliSecond<uint32_t> time, std::function<void(void)> callback){
		On(bit);
		TimeScheduler<void>::DelayCall(
				[this, bit, callback](){
					Off(bit);
					callback();
				}, time);
	}
	/**
	 * @brief 弁OFF
	 * @param[in] bit ビット列
	 */
	void Off(uint16_t bit){
		SetAnd(~bit);
	}
	/**
	 * @brief 一定時間弁OFF
	 * @param[in] bit ビット列
	 * @param[in] time OFF時間
	 */
	void Off(uint16_t bit, MilliSecond<uint32_t> time){
		Off(bit);
		TimeScheduler<void>::DelayCall([this, bit](){On(bit);}, time);
	}
	/**
	 * @brief 一定時間弁OFF
	 * @param[in] bit ビット列
	 * @param[in] time OFF時間
	 * @param[in] callback ON時呼び出しコールバック関数
	 */
	void Off(uint16_t bit, MilliSecond<uint32_t> time, std::function<void(void)> callback){
		Off(bit);
		TimeScheduler<void>::DelayCall(
				[this, bit, callback](){
					On(bit);
					callback();
				}, time);
	}

	/**
	 * @brief 全ビットON
	 */
	void AllOn(){
		On(0x3FF);
	}
	/**
	 * @brief 全ビットOFF
	 */
	void AllOff(){
		Off(0x3FF);
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
