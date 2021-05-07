/**
 *　@brief　データのエンコードやデコードのアルゴリズム類
 */
#pragma once

#include "Global.hpp"
#include <vector>

namespace nut{
/**
 * @brief COBSエンコード名前空間
 */
namespace COBS{
/**
 * @brief COBSエンコード
 * @param[in] first エンコード対象データコンテナのイテレータ始点
 * @param[in] last エンコード対象データコンテナのイテレータ終点
 * @return エンコード後データ
 */
inline std::vector<uint8_t> Encode(std::vector<uint8_t>::const_iterator first, std::vector<uint8_t>::const_iterator last){
	std::vector<uint8_t> encoded_data;
	size_t size = std::distance(first, last);
	if(size < 256 && size > 0){
		encoded_data.reserve(size + 2);
		encoded_data.push_back(0x00);
		std::copy(first, last, std::back_inserter(encoded_data));

		uint8_t zero_index = 0;
		{
			uint8_t i = 0;
			for(const auto& d : encoded_data){
				if(d == 0x00){
					encoded_data[zero_index] = i - zero_index;
					zero_index = i;
				}
				++i;
			}
		}
		encoded_data[zero_index] = encoded_data.size() - zero_index;
		encoded_data.push_back(0x00);
	}
	return encoded_data;
}
/**
 * @brief COBSエンコード
 * @param[in] data エンコード対象データ
 * @return エンコード後データ
 */
inline std::vector<uint8_t> Encode(const std::vector<uint8_t>& data){
	return std::move(Encode(data.begin(), data.end()));
}
/**
 * @brief COBSデコード
 * @param[in] first デコード対象データコンテナのイテレータ始点
 * @param[in] last デコード対象データコンテナのイテレータ終点
 * @return デコード後データ
 */
inline std::vector<uint8_t> Decode(std::vector<uint8_t>::const_iterator first, std::vector<uint8_t>::const_iterator last){
	std::vector<uint8_t> decoded_data;
	size_t size = std::distance(first, last);
	if(size < 258 && size > 2){
		decoded_data.reserve(size);
		std::copy(first, last, std::back_inserter(decoded_data));

		uint8_t zero_index = 0;
        while (zero_index < decoded_data.size() - 1)
        {
            if (decoded_data[zero_index] == 0) return std::vector<uint8_t>();
            uint8_t tmp_index = zero_index;
            zero_index += decoded_data[zero_index];
            decoded_data[tmp_index] = 0;
        }
        decoded_data.erase(decoded_data.begin());
        decoded_data.pop_back();
	}
	return decoded_data;
}
/**
 * @brief COBSデコード
 * @param[in] data デコード対象データ
 * @return デコード後データ
 */
inline std::vector<uint8_t> Decode(const std::vector<uint8_t>& data){
	return std::move(Decode(data.begin(), data.end()));
}
}
}
