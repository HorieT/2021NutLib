/**
 * 自作PCコントローラとの通信用クラス
 * ジョイコンのデータはここで持つ
 * GUIの入出力はユーザー定義
 */
#pragma once

#define IS_EDIT_CONTROLAPP//編集用マクロ こいつが無いとエディタが仕事しない

#if defined(IS_EDIT_CONTROLAPP) || __has_include("usbd_cdc_if.h")
#include "../Global.hpp"
#include "../DataEncode.hpp"
#include "../Unit/UnitCore.hpp"
#include "JoyconROS.hpp"
#include "usbd_cdc_if.h"

#include <functional>
#include <map>

namespace nut{
class ControlApp final{
private:
	static constexpr uint8_t JOYCON_HEAD = 0xF0;
	std::map<uint8_t, std::function<bool(std::vector<uint8_t>)>> _get_sentence_callback;
	bool _is_enable_joycon = false;
	JoyconROS::ButtonData _joycon;


	bool ReadJoycon(std::vector<uint8_t> data){
		if(data.size() == sizeof(_joycon) + 1){
			std::memcpy(&_joycon, data.data(), sizeof(JoyconROS::ButtonData));
			_is_enable_joycon = true;
		}
		return true;
	}
	/**
	 * @brief 受信文の振り分け
	 * といっても一般データとジョイコンのデータを分けてるだけ
	 */
	bool ReadSentence(std::vector<uint8_t>&& data_sentence){
		uint8_t head = data_sentence[0];
		std::vector<uint8_t> data(std::next(data_sentence.begin()), data_sentence.end());

		if(head == JOYCON_HEAD){
			return ReadJoycon(std::move(data));
		}else{
			if(_get_sentence_callback.count(head) != 0)return _get_sentence_callback[head](std::move(data));
			else return false;
		}
	}
public:
	ControlApp(){}
	~ControlApp(){}


	/*
	 * @brief USB CDCからの受信関数
	 * @param[in,out] data 受信データ列
	 * 終端が存在しない未処理データを残して返す
	 * @return データ文の処理数
	 */
	uint8_t ReadData(std::vector<uint8_t>& data){
		uint8_t sentence_count = 0;
		auto start_it = data.begin();
		for(auto it = start_it, end = data.end();it != end;++it){
			if(*it == 0){
				std::vector<uint8_t> sentence = std::move(COBS::Decode(start_it, std::next(it)));
				start_it = std::next(it);

				/*sentence処理*/
				if(sentence.size() == 0)continue;
				if(ReadSentence(std::move(sentence)))++sentence_count;
			}
		}
		data.erase(data.begin(), start_it);
		return sentence_count;
	}

	bool AddReadDataCallback(uint8_t head, std::function<bool(std::vector<uint8_t>)> callback){
		if(head == JOYCON_HEAD)return false;
		if(_get_sentence_callback.count(head) != 0) return false;

		_get_sentence_callback.emplace(head, callback);
		return true;
	}
	bool EraseReadDataCallback(uint8_t head){
		if(head == JOYCON_HEAD)return false;
		if(_get_sentence_callback.count(head) == 0) return false;

		_get_sentence_callback.erase(head);
		return true;
	}


	template<size_t Size>
	void SendData(uint8_t head, std::array<uint8_t, Size> data){
		std::vector<uint8_t> send_msg(Size + 1);
		send_msg[0] = head;
		std::memcpy(&send_msg[1], data.data(), Size);

		CDC_Transmit_FS(COBS::Encode(send_msg).data(), Size + 3);//USB送信はブロッカーで同期処理になってるはず
	}
	template<typename T>
	auto SendData(uint8_t head, T data) -> std::enable_if<std::is_trivially_copyable_v<T>, void>{
		//static_assert(std::is_trivially_copyable_v<T>, "Send data must be trivial copyable");
		static constexpr uint8_t size = sizeof(T);

		std::vector<uint8_t> send_msg(size + 1);
		send_msg[0] = head;
		std::memcpy(&send_msg[1], &data, size);

		CDC_Transmit_FS(COBS::Encode(send_msg).data(), size + 3);//USB送信はブロッカーで同期処理になってるはず
	}

	bool IsEnableJoycon()const{
		return _is_enable_joycon;
	}
	const JoyconROS::ButtonData& GetJoycon()const{
		return _joycon;
	}
};
}
#endif
