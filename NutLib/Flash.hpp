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
 * マイコン電源電圧を2.7~3.6Vにしてください<br>
 * STM32マイコンのフラッシュ書き換え最低保証回数は1万回です。
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
	 * @return 消去成功か
	 */
	bool EraseSector(uint32_t sector){
#ifdef FLASH_TYPEERASE_SECTORS
		FLASH_EraseInitTypeDef erase;
		erase.TypeErase = FLASH_TYPEERASE_SECTORS;	// sector毎
		erase.Sector = sector;		       			// sector指定
		erase.NbSectors = 1;						// 1sector
		erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;	// set voltage range (2.7 to 3.6V)

		uint32_t pageError = 0;

		return HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;	// erase sector
#else
		return false;
#endif
	}

	/**
	 * @brief 1ページのフラッシュ消去
	 * @param[in] address 消去ページアドレス
	 * @return 消去成功か
	 */
	bool ErasePage(uint32_t address){
#ifdef FLASH_TYPEERASE_PAGES
		FLASH_EraseInitTypeDef erase;
		erase.TypeErase = FLASH_TYPEERASE_PAGES;
		erase.PageAddress =　address;
		erase.NbPages = 1;

		uint32_t pageError = 0;

		return HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;	// erase sector
#else
		return false;
#endif
	}

	/**
	 * @brief フラッシュセクタ書き込み
	 * @param[in] sector 書き込みセクタ
	 * @param[in] address 書き込み先頭アドレス
	 * @param[in] data 書き込みデータ
	 * @param[in] size 書き込みデータサイズ
	 * @return 書き込み成功か
	 * @attention セクタとアドレスが合致しているか、データがセクタをまたがないか注意してください<br>
	 * 			同一セクタの情報は全て消えます
	 */
	bool WriteSector(uint32_t sector, uint32_t address, void* data, uint32_t size){
#ifdef FLASH_TYPEERASE_SECTORS
		HAL_FLASH_Unlock();		// unlock flash
		EraseSector(sector);			// erase sector

		for ( uint32_t i = 0; i < size; ++i){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, static_cast<uint8_t*>(data)[i]); // write byte
		}

		HAL_FLASH_Lock();		// lock flash
		return true;
#else
		return false;
#endif
	}
	/**
	 * @brief フラッシュセクタ書き込み
	 * @tparam N 書き込みデータサイズ
	 * @param[in] sector 書き込みセクタ
	 * @param[in] address 書き込み先頭アドレス
	 * @param[in] data 書き込みデータ
	 * @return 書き込み成功か
	 * @attention セクタとアドレスが合致しているか、データがセクタをまたがないか注意してください<br>
	 * 			同一セクタの情報は全て消えます
	 */
	template<uint32_t N>
	bool WriteSector(uint32_t sector, uint32_t address, std::array<uint8_t, N> data){
		return WriteSector(sector, address, data.data(), N);
	}
	/**
	 * @brief フラッシュセクタ書き込み
	 * @tparam T 書き込みデータ型
	 * @param[in] sector 書き込みセクタ
	 * @param[in] address 書き込み先頭アドレス
	 * @param[in] data 書き込みデータ
	 * @return 書き込み成功か
	 * @attention セクタとアドレスが合致しているか、データがセクタをまたがないか注意してください<br>
	 * 			同一セクタの情報は全て消えます
	 */
	template<typename T>
	bool WriteSector(uint32_t sector, uint32_t address, T* data){
		return WriteSector(sector, address, data, sizeof(T));
	}


	/**
	 * @brief フラッシュページ書き込み
	 * @param[in] address 書き込みページアドレス
	 * @param[in] data 書き込みデータ
	 * @param[in] size 書き込みデータサイズ
	 * @return 書き込み成功か
	 * @attention データがページをまたがないか注意してください<br>
	 * 			同一ページの情報は全て消えます
	 */
	bool WritePage(uint32_t address, void* data, uint32_t size){
#ifdef FLASH_TYPEERASE_PAGES
		HAL_FLASH_Unlock();		// unlock flash
		ErasePage(address);			// erase sector

		for ( uint32_t i = 0; i < size / 2 + size % 2; ++i){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, static_cast<uint16_t*>(data)[i]); // write byte
		}

		HAL_FLASH_Lock();		// lock flash
		return true;
#else
		return false;
#endif
	}
	/**
	 * @brief フラッシュページ書き込み
	 * @tparam N 書き込みデータサイズ
	 * @param[in] address 書き込みページアドレス
	 * @param[in] data 書き込みデータ
	 * @return 書き込み成功か
	 * @attention データがページをまたがないか注意してください<br>
	 * 			同一ページの情報は全て消えます
	 */
	template<uint32_t N>
	bool WritePage(uint32_t address, std::array<uint8_t, N> data){
		return WritePage(address, data.data(), N);
	}
	/**
	 * @brief フラッシュページ書き込み
	 * @tparam T 書き込みデータ型
	 * @param[in] address 書き込みページアドレス
	 * @param[in] data 書き込みデータ
	 * @return 書き込み成功か
	 * @attention データがページをまたがないか注意してください<br>
	 * 			同一ページの情報は全て消えます
	 */
	template<typename T>
	bool WritePage(uint32_t address, T* data){
		return WritePage(address, data, sizeof(T));
	}
}
}
