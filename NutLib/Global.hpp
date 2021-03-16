/**
 * @file Global.hpp
 * @brief ライブラリの基幹部分
 * @author Horie
 * @date 2020/10
 * @details 未対応のSTMマイコンがあればインクルードファイルを書き足してください
 */
#pragma once

/*
 * HALのインクルード
 * 使用マイコンのバリエーションが増えるたびに書き足してください
 */
extern "C"{
#if defined STM32F042x6
#include "stm32f0xx_hal.h"
#elif defined STM32F103xB
#include "stm32f1xx_hal.h"
#elif defined STM32F303x8
#include "stm32f3xx_hal.h"
#elif defined STM32F407xx || STM32F446xx
#include "stm32f4xx_hal.h"
#elif defined STM32F767xx
#include "stm32f7xx_hal.h"
#endif
}

#ifndef UNUSE_NUTLIB_CALLBACKS
//!< HAL default callback function をライブラリに移譲します
#define USE_NUTLIB_CALLBACKS
#endif

//Eigen代数計算ライブラリ
#include "Eigen/Core"

/**
 * STL
 *c++20以降であれば_USE_MATH_DEFINES及びM_PI等の定義を消し<numbers>に変更
 */
/**
 * cmathの定数制御マクロ
 */
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdint>

#ifndef M_PI//!< ビルド環境によってcmathマクロが機能しない場合の定義(数学定数はc++の仕様定義に無いのでコンパイラの独自拡張です)
#define M_E 		2.71828182845904523536
#define M_LOG2E 	1.44269504088896340736
#define M_LOG10E	0.434294481903251827651
#define M_LN2		0.693147180559945309417
#define M_LN10		2.30258509299404568402
#define M_PI 		3.14159265358979323846
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.785398163397448309616
#define M_1_PI		0.318309886183790671538
#define M_2_PI		0.636619772367581343076
#define M_2_SQRTPI	1.12837916709551257390
#define M_SQRT2		1.41421356237309504880
#define M_SQRT1_2	0.707106781186547524401
#endif

/* 良く使うやつの拡張 */
constexpr float M_2PI_f 	= static_cast<float>(M_PI * 2.0);
constexpr float M_PI_f 		= static_cast<float>(M_PI);
constexpr float M_PI_2_f 	= static_cast<float>(M_PI / 2.0);


/**
 * @namespace nut
 * @brief ライブラリの統括名前空間
 */
namespace nut{

#ifdef USE_NUTLIB_CALLBACKS//あたまわるわる
static constexpr bool USE_NUTLIB_CALLBACKS_FLAG = true;
#else
static constexpr bool USE_NUTLIB_CALLBACKS_FLAG = false;
#endif


template<typename T>
constexpr auto BitSwap(T data) -> std::enable_if_t<std::is_integral_v<T>, T>{
	constexpr size_t size = sizeof(T);
	static_assert(size <= 32, "bit size over");
	T tmp = data;
	switch(size){
	case 32:
		tmp = ((tmp & 0x0000ffff) << 16) | ((tmp >> 16) & 0x0000ffff);
		[[fallthrough]];
	case 16:
		tmp = ((tmp & 0x00ff00ff) <<  8) | ((tmp >>  8) & 0x00ff00ff);
		[[fallthrough]];
	case 8:
		tmp = ((tmp & 0x0f0f0f0f) <<  4) | ((tmp >>  4) & 0x0f0f0f0f);
		[[fallthrough]];
	case 4:
		tmp = ((tmp & 0x33333333) <<  2) | ((tmp >>  2) & 0x33333333);
		[[fallthrough]];
	case 2:
		tmp = ((tmp & 0x55555555) <<  1) | ((tmp >>  1) & 0x55555555);
		[[fallthrough]];
	default:
		break;
	}
	return tmp;
}
}
