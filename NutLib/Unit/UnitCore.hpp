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
}
