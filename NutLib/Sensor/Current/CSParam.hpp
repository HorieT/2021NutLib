/**
 *
 */
#pragma once

#include "../../Global.hpp"
#include "../../Unit/UnitCore.hpp"


namespace nut{
/**
 * @brief 電流センサパラメータクラス
 */
class CSParam{
private:
	const float _current_sensitivity;


public:
	/**
	 * @brief コンストラクタ
	 * @param[in] current_sensitivity 電流感度[V/A]
	 */
	constexpr CSParam(float current_sensitivity) : _current_sensitivity(current_sensitivity){

	}


	/**
	 * @brief 電流感度取得
	 */
	constexpr float current_sensitivity() const{return _current_sensitivity;}
	/**
	 * @brief bit単位取得
	 * @tparam Bit ADC分解能
	 * @param[in] volt ADC電源電圧
	 * @return ADC出力係数[A/bit]
	 */
	template<uint8_t Bit>
	constexpr float ampere_par_bit(Volt<float> volt) const{
		static_assert(Bit <= 16 && Bit >= 1, "bit size over.");
		uint16_t bit = 1u;
		for(uint8_t i = 1;i < Bit;++i)bit |= 1 << i;

		return volt.f() / static_cast<float>(bit) / _current_sensitivity;
	}
};
}

