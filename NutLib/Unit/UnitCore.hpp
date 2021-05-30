/**
 * @file UnitCore.hpp
 * @brief 全単位系をインクルードする
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Unit.hpp"

namespace nut{
template<typename T>
using Second = unit::Unit<T, nut::unit::Type::second>;
template<typename T>
using MilliSecond = unit::Unit<T, nut::unit::Type::second, unit::milli>;
template<typename T>
using MicroSecond = unit::Unit<T, nut::unit::Type::second, unit::micro>;

template<typename T>
using Minute = unit::Unit<T, nut::unit::Type::minute>;

template<typename T>
using Meter = unit::Unit<T, nut::unit::Type::meter>;
template<typename T>
using MilliMeter = unit::Unit<T, nut::unit::Type::meter, unit::milli>;


template<typename T>
using Gram = unit::Unit<T, nut::unit::Type::gram>;
template<typename T>
using MilliGram = unit::Unit<T, nut::unit::Type::gram, unit::milli>;
template<typename T>
using KiloGram = unit::Unit<T, nut::unit::Type::gram, unit::kilo>;


template<typename T>
using Radian = unit::Unit<T, nut::unit::Type::radian>;

template<typename T>
using Degre = unit::Unit<T, nut::unit::Type::degre>;

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

/* 変換 */
namespace unit{
template<typename T>
constexpr Second<T> ToSecond(Minute<T> minute){return static_cast<T>(minute * 60.0);}
template<typename T>
constexpr Minute<T> ToMinute(Second<T> second){return static_cast<T>(second / 60.0);}
template<typename T>
constexpr Radian<T> ToRadian(Degre<T> degre){return static_cast<T>(degre / 180.0 * M_PI);}
template<typename T>
constexpr Degre<T> ToDegre(Radian<T> radian){return static_cast<T>(radian / M_PI * 180.0);}

}
}

