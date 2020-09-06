/**
 * @file Unit.hpp
 * @brief 単位系
 * @author Horie
 * @date 2020/9
 * @attention まだ作成中
 */
#pragma once

#include "Global.hpp"

#ifdef false;
namespace nut{
enum class UnitType : uint16_t{
	second = 0U,
	meter,
	radian,
	degre
};



template<UnitType U, typename T>
class Unit{
private:
	T _value;

public:
	Unit(T value) : _value(value){}


	/*
	 * ���Z�q�̃I�[�o�[���[�h
	 */
	constexpr T& operator=(const Unit<U, T>& r_operand) {
		return _value;
	}
	constexpr operator T () const noexcept {
		return _value;
	}

	constexpr Unit<U, T>& operator+=(const Unit<U, T>& r_operand){
		_value += r_operand;
		return *this;
	}
	constexpr Unit<U, T>& operator-=(const Unit<U, T>& r_operand){
		_value -= r_operand;
		return *this;
	}
	constexpr Unit<U, T>& operator*=(const T& r_operand){
		_value *= r_operand;
		return *this;
	}
	constexpr Unit<U, T>& operator/=(const T& r_operand){
		_value /= r_operand;
		return *this;
	}
};

/*
 * ���Z�q�̃I�[�o�[���[�h
 */
template<UnitType U1, UnitType U2, typename T>
bool operator<(const Unit<U1, T>& l_operand, const Unit<U2, T>& r_operand) {
	static_assert(U1 == U2, "UnitType is different.");
	return l_operand < r_operand;
}
template<UnitType U1, UnitType U2, typename T>
bool operator>(const Unit<U1, T>& l_operand, const Unit<U2, T>& r_operand) {
	static_assert(U1 == U2, "UnitType is different.");
	return l_operand > r_operand;
}
template<UnitType U1, UnitType U2, typename T>
bool operator<=(const Unit<U1, T>& l_operand, const Unit<U2, T>& r_operand) {
	return !(l_operand > r_operand);
}
template<UnitType U1, UnitType U2, typename T>
bool operator>=(const Unit<U1, T>& l_operand, const Unit<U2, T>& r_operand) {
	return !(l_operand < r_operand);
}
template<UnitType U1, UnitType U2, typename T>
Unit<U1, T> operator+(const Unit<U1, T>& l_operand, const Unit<U2, T>& r_operand) {
	static_assert(U1 == U2, "UnitType is different.");
	return l_operand += r_operand;
}
template<UnitType U1, UnitType U2, typename T>
Unit<U1, T> operator-(const Unit<U1, T>& l_operand, const Unit<U2, T>& r_operand) {
	static_assert(U1 == U2, "UnitType is different.");
	return l_operand -= r_operand;
}
}
#endif
