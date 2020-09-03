/*
 * �t���b�V������
 * �݊����̂��ߌ����ȃZ�[�t�e�B���������Ă��Ȃ��̂Ŏ�舵���͏\������
 * ����Ȏg����������ƃ}�C�R�������ʏꍇ������(����)
 */
#pragma once

#include "Global.hpp"
#include <array>
#include <vector>
#include <string>

namespace nut{
namespace flash{
	/*
	 * �ǂݏo��
	 */
	inline void Read(uint32_t address, void* data, uint32_t size){
		std::memcpy(data, reinterpret_cast<uint8_t*>(address), size);
	}
	template<uint32_t N>
	inline void Read(uint32_t address, std::array<uint8_t, N> data){
		std::memcpy(data.data(), reinterpret_cast<uint8_t*>(address), N);
	}
	template<typename T>
	inline void Read(uint32_t address, T* data){
		std::memcpy(data, reinterpret_cast<uint8_t*>(address), sizeof(T));
	}


	/*
	 * ����
	 */
	void Erase(uint32_t sector)
	{
		FLASH_EraseInitTypeDef erase;
		erase.TypeErase = FLASH_TYPEERASE_SECTORS;	// �Z�N�^���Ƃɏ���
		erase.Sector = sector;		       			// �擪�Z�N�^�I��
		erase.NbSectors = 1;						// �I���Z�N�^��
		erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;	// set voltage range (2.7 to 3.6V)

		uint32_t pageError = 0;

		HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
	}


	/*
	 * ��������
	 */
	void Write(uint32_t sector, uint32_t address, void* data, uint32_t size){

		HAL_FLASH_Unlock();		// unlock flash
		Erase(sector);			// erease sector

		for ( uint32_t i = 0; i < size; ++i){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, static_cast<uint8_t*>(data)[i]); // write byte
		}

		HAL_FLASH_Lock();		// lock flash
	}
	template<uint32_t N>
	void Write(uint32_t sector, uint32_t address, std::array<uint8_t, N> data){
		Write(sector, address, data.data(), N);
	}
	template<typename T>
	void Write(uint32_t sector, uint32_t address, T* data){
		Write(sector, address, data, sizeof(T));
	}
}
}
