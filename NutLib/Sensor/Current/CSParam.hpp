/**
 *
 */
#pragma once

#include "../../Global.hpp"
#include "../../Unit/UnitCore.hpp"


namespace nut{
/**
 *
 */
class CSParam{
private:
	const float _current_sensitivity;


public:
	constexpr CSParam(float current_sensitivity) : _current_sensitivity(current_sensitivity){

	}


	constexpr float current_sensitivity() const{return _current_sensitivity;}
	template<uint8_t Bit>
	constexpr float ampere_par_bit(Volt<float> volt) const{
		static_assert(Bit <= 16 && Bit >= 1, "bit size over.");
		uint16_t bit = 1u;
		for(uint8_t i = 1;i < Bit;++i)
			bit |= 1 << i;

		return volt.f() / static_cast<float>(bit) / _current_sensitivity;
	}
};
}

