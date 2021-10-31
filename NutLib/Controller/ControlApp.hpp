/**
 * @brief 自作PCコントローラとの通信用クラス
 * ジョイコンのデータはここで持つ
 * GUIの入出力はユーザー定義
 */
#pragma once

//#define IS_EDIT_CONTROLAPP///編集用マクロ こいつが無いとエディタが仕事しない

#if defined(IS_EDIT_CONTROLAPP) || __has_include("usbd_cdc_if.h")
#include "../Global.hpp"
#include "../DataEncode.hpp"
#include "../Unit/UnitCore.hpp"
#include "JoyconROS.hpp"
#include "usbd_cdc_if.h"

#include <functional>
#include <map>

namespace nut{
/**
 * @brief 自作PCコントローラクラス
 */
class ControlApp final{
private:
	static constexpr uint8_t JOYCON_HEAD = 0x80;
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
	 * @param[in] data_sentence 受信データ文
	 * @return 振り分けできたかどうか
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


	/**
	 * @brief USB CDCからの受信関数
	 * @details CDC_Receive_FS()内で呼び出してください
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

	/**
	 * @brief データ受信コールバック追加
	 * @details 同一ヘッダに付き一種類のコールバック関数が持てます
	 * @param[in] head データヘッダ
	 * @param[in] callback コールバック関数
	 * @return 追加できたか
	 */
	bool AddReadDataCallback(uint8_t head, std::function<bool(std::vector<uint8_t>)> callback){
		if(head == JOYCON_HEAD)return false;
		if(_get_sentence_callback.count(head) != 0) return false;

		_get_sentence_callback.emplace(head, callback);
		return true;
	}
	/**
	 * @brief データ受信コールバック削除
	 * @param[in] head データヘッダ
	 * @return 削除できたか
	 */
	bool EraseReadDataCallback(uint8_t head){
		if(head == JOYCON_HEAD)return false;
		if(_get_sentence_callback.count(head) == 0) return false;

		_get_sentence_callback.erase(head);
		return true;
	}


	/**
	 * @brief データ送信
	 * @tparam T データ型
	 * @param[in] head ヘッダ
	 * @param[in] data データ
	 */
	template<typename T>
	auto SendData(uint8_t head, T data) -> std::enable_if_t<std::is_trivially_copyable_v<T>>{
		//static_assert(std::is_trivially_copyable_v<T>, "Send data must be trivial copyable");
		static constexpr uint8_t size = sizeof(T);

		std::vector<uint8_t> send_msg(size + 1);
		send_msg[0] = head;
		std::memcpy(&send_msg[1], &data, size);

		CDC_Transmit_FS(COBS::Encode(send_msg).data(), size + 3);//USB送信はブロッカーで同期処理になってるはず
	}

	/**
	 * @brief ジョイコンデータが有効に受信できているか
	 * @return 有効か
	 */
	bool IsEnableJoycon()const{
		return _is_enable_joycon;
	}
	/**
	 * @brief ジョイコンデータの取得
	 * @return ジョイコンデータ
	 */
	const JoyconROS::ButtonData& GetJoycon()const{
		return _joycon;
	}
};
}
#endif
