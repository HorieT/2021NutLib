/**
 * @file Global.hpp
 * @brief ライブラリの基幹部分
 * @author Horie
 * @date 2020/9
 * @details 未対応のSTMマイコンがあればインクルードファイルを書き足してください
 */
#pragma once

/*
 * HALのインクルード
 * 使用マイコンのバリエーションが増えるたびに書き足してください
 */
extern "C"{
#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#elif defined STM32F042x6
#include "stm32f0xx_hal.h"
#elif defined STM32F767xx
#include "stm32f7xx_hal.h"
#endif
}

//Eigen代数計算ライブラリ
#include "Eigen/Core"

/**
 * STL
 *c++20以降であれば_USE_MATH_DEFINES及びM_PIの定義を消し<numbers>に変更
 */
/**
 * cmathの定数制御マクロ
 */
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdint>

#ifdef M_PI
#else
#define M_PI 3.14159265358979323846//!< ビルド環境によってcmathマクロが機能しない場合の定義
#endif


/**
 * @namespace nut
 * @brief ライブラリの統括名前空間
 */
namespace nut{

/*Metafunction*/


}
