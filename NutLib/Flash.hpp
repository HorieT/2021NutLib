/**
 * @file Flash.hpp
 * @brief Flashのヘルパ
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Global.hpp"
#include <array>
#include <vector>
#include <string>

namespace nut{
/**
 * @brief Flash操作関数群
 * @attention フラッシュメモリ領域サイズ、及びフラッシュ操作時間に注意！<br>
 * マイコン電源電圧を2.7~3.6Vにしてください
 */
namespace flash{
	/**
	 * @brief  フラッシュ読み取り
	 * @param[in] address 読み取り先頭アドレス
	 * @param[out] data 読み取りデータ保存先
	 * @param[in] size 読み取りデータサイズ
	 */
	inline void Read(uint32_t address, void* data, uint32_t size){
		std::memcpy(data, reinterpret_cast<uint8_t*>(address), size);
	}
	/**
	 * @brief  フラッシュ読み取り
	 * @tparam N 読み取りデータサイズ
	 * @param[in] address 読み取り先頭アドレス
	 * @param[out] data 読み取りデータ保存先
	 */
	template<uint32_t N>
	inline void Read(uint32_t address, std::array<uint8_t, N> data){
		std::memcpy(data.data(), reinterpret_cast<uint8_t*>(address), N);
	}
	/**
	 * @brief  フラッシュ読み取り
	 * @tparam T 読み取りデータ型
	 * @param[in] address 読み取り先頭アドレス
	 * @param[out] data 読み取りデータ保存先
	 */
	template<typename T>
	inline void Read(uint32_t address, T* data){
		std::memcpy(data, reinterpret_cast<uint8_t*>(address), sizeof(T));
	}


	/**
	 * @brief 1セクタのフラッシュ消去
	 * @param[in] sector 消去セクタ
	 */
	void Erase(uint32_t sector){
		FLASH_EraseInitTypeDef erase;
		erase.TypeErase = FLASH_TYPEERASE_SECTORS;	// セクタ毎
		erase.Sector = sector;		       			// セクタ指定
		erase.NbSectors = 1;						// 1セクタ
		erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;	// set voltage range (2.7 to 3.6V)

		uint32_t pageError = 0;

		HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
	}


	/**
	 * @brief フラッシュ書き込み
	 * @param[in] sector 書き込みセクタ
	 * @param[in] address 書き込み先頭アドレス
	 * @param[in] data 書き込みデータ
	 * @param[in] size 書き込みデータサイズ
	 * @attention セクタとアドレスが合致しているか、データがセクタをまたがないか注意してください<br>
	 * 			同一セクタの情報は全て消えます
	 */
	void Write(uint32_t sector, uint32_t address, void* data, uint32_t size){

		HAL_FLASH_Unlock();		// unlock flash
		Erase(sector);			// erase sector

		for ( uint32_t i = 0; i < size; ++i){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, static_cast<uint8_t*>(data)[i]); // write byte
		}

		HAL_FLASH_Lock();		// lock flash
	}
	/**
	 * @brief フラッシュ書き込み
	 * @tparam N 書き込みデータサイズ
	 * @param[in] sector 書き込みセクタ
	 * @param[in] address 書き込み先頭アドレス
	 * @param[in] data 書き込みデータ
	 * @attention セクタとアドレスが合致しているか、データがセクタをまたがないか注意してください<br>
	 * 			同一セクタの情報は全て消えます
	 */
	template<uint32_t N>
	void Write(uint32_t sector, uint32_t address, std::array<uint8_t, N> data){
		Write(sector, address, data.data(), N);
	}
	/**
	 * @brief フラッシュ書き込み
	 * @tparam T 書き込みデータ型
	 * @param[in] sector 書き込みセクタ
	 * @param[in] address 書き込み先頭アドレス
	 * @param[in] data 書き込みデータ
	 * @attention セクタとアドレスが合致しているか、データがセクタをまたがないか注意してください<br>
	 * 			同一セクタの情報は全て消えます
	 */
	template<typename T>
	void Write(uint32_t sector, uint32_t address, T* data){
		Write(sector, address, data, sizeof(T));
	}
}
}
