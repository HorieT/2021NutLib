/**
 * @file UnitCore.hpp
 * @brief 全単位系をインクルードする
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Unit.hpp"
#include <cmath>

namespace nut{
template<typename T>
using Second = unit::Unit<T, nut::unit::Type::second>;
template<typename T>
using MilliSecond = unit::Unit<T, nut::unit::Type::second, unit::milli>;
template<typename T>
using MicroSecond = unit::Unit<T, nut::unit::Type::second, unit::micro>;
template<typename T>
using Minute = unit::Unit<T, nut::unit::Type::second, unit::Prefix<60, 1>>;

template<typename T>
using Meter = unit::Unit<T, nut::unit::Type::meter>;
template<typename T>
using MilliMeter = unit::Unit<T, nut::unit::Type::meter, unit::milli>;
template<typename T>
using Inch = unit::Unit<T, nut::unit::Type::meter, unit::Prefix<1000, 393701>>;


template<typename T>
using Gram = unit::Unit<T, nut::unit::Type::gram>;
template<typename T>
using MilliGram = unit::Unit<T, nut::unit::Type::gram, unit::milli>;
template<typename T>
using KiloGram = unit::Unit<T, nut::unit::Type::gram, unit::kilo>;


template<typename T>
using Radian = unit::Unit<T, nut::unit::Type::radian>;
template<typename T>
using Degree = unit::Unit<T, nut::unit::Type::radian,
		unit::Prefix<static_cast<uint32_t>((std::numeric_limits<uint32_t>::max() / 180.0) * M_PI), std::numeric_limits<uint32_t>::max()>>;//なるべく精度を出すために最大化

template<typename T>
using Ampere = unit::Unit<T, nut::unit::Type::ampere>;
template<typename T>
using MilliAmpere = unit::Unit<T, nut::unit::Type::ampere, unit::milli>;

template<typename T>
using Volt = unit::Unit<T, nut::unit::Type::volt>;
template<typename T>
using MilliVolt = unit::Unit<T, nut::unit::Type::volt, unit::milli>;

template<typename T>
using Ohm = unit::Unit<T, nut::unit::Type::ohm>;
template<typename T>
using MilliOhm = unit::Unit<T, nut::unit::Type::ohm, unit::milli>;
template<typename T>
using KiloOhm = unit::Unit<T, nut::unit::Type::ohm, unit::kilo>;
template<typename T>
using MegaOhm = unit::Unit<T, nut::unit::Type::ohm, unit::mega>;

template<typename T>
using Henry = unit::Unit<T, nut::unit::Type::henry>;
template<typename T>
using MicroHenry = unit::Unit<T, nut::unit::Type::henry, unit::micro>;

template<typename T>
using Farad = unit::Unit<T, nut::unit::Type::farad>;
template<typename T>
using MicroFarad = unit::Unit<T, nut::unit::Type::farad, unit::micro>;

/* 非メンバ関数 */
/**
 * @brief +pi~-piに正規化
 * @param[in] rad 角度
 * @return +pi~-piの角度
 */
template<typename T>
constexpr Radian<T> NormalizeRadian(Radian<T> rad){
	if(!std::isfinite(rad.value()))return rad;
	T tmp = std::fmod(rad.value(), M_2PI_f);
	return (tmp < -M_PI) ? tmp + M_2PI_f : ((tmp > M_PI) ? tmp - M_2PI_f : tmp);
}
template<typename T>
constexpr auto sin(Radian<T> rad){
	return std::sin(rad.value());
}
template<typename T>
constexpr auto cos(Radian<T> rad){
	return std::cos(rad.value());
}
template<typename T>
constexpr auto tan(Radian<T> rad){
	return std::tan(rad.value());
}

template<typename T, unit::Type U, class P>
constexpr auto abs(unit::Unit<T, U, P> value){
	return unit::Unit<T, U, P>(std::abs(value.value()));
}
}

