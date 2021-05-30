/**
 * @file Unit.hpp
 * @brief 単位系
 * @author Horie
 * @date 2020/9
 * @attention 構造上,組み単位が作りにくい
 */
#pragma once

#include "../Global.hpp"
#include "Prefix.hpp"
#include <ratio>

namespace nut{
/**
 * @brief 単位系名前空間
 */
namespace unit{
/**
 * @brief 単位
 */
enum class Type : uint16_t{
	second = 0U,	//!< 秒
	minute,			//!< 分
	meter,			//!< メートル
	radian,			//!< rad
	degre,			//!< deg
	gram,			//!< グラム
	ampere,			//!< アンペア
	volt,			//!< ボルト
	ohm,			//!< オーム
	henry,			//!< ヘンリー
	farad,			//!< ファラド
};



/**
 * @brief 単位系クラス
 * @tparam T 数値型
 * @attention Tが数値型でなければアサートを吐きます
 * @tparam U 単位の種類
 * @tparam P 単位の接頭辞
 * @attention PがPrefix<>でなければ不適格です
 */
template<typename T, Type U, class P = Prefix<1, 1>>
class Unit{
	static_assert(std::is_arithmetic_v<T>, "Type is not arithmetic.");
	static_assert(std::is_base_of_v<PrefixBase, P> && !std::is_same_v<PrefixBase, P>, "Type is not Prefix.");

private:
	T _value;

public:
	constexpr float f()const{
		return static_cast<float>(_value);
	}
	constexpr double d()const{
		return static_cast<double>(_value);
	}
	constexpr int8_t i8()const{
		return static_cast<int8_t>(_value);
	}
	constexpr int16_t i16()const{
		return static_cast<int16_t>(_value);
	}
	constexpr int32_t i32()const{
		return static_cast<int32_t>(_value);
	}
	constexpr uint8_t u8()const{
		return static_cast<uint8_t>(_value);
	}
	constexpr uint16_t u16()const{
		return static_cast<uint16_t>(_value);
	}
	constexpr uint32_t u32()const{
		return static_cast<uint32_t>(_value);
	}





	constexpr Unit(T value = 0) : _value(value){}
	/**
	 * @brief 同一単位系の代入構築
	 * @tparam TR 右オペランドの数値型
	 * @tparam PR 右オペランドの接頭辞
	 * @return this
	 */
	template<typename TR, class PR>
	constexpr Unit(const Unit<TR, U, PR>& r_operand){
		_value = static_cast<T>((static_cast<TR>(r_operand) * P::den * PR::num) / (P::num * PR::den));
	}
	/**
	 * @brief 同一単位系の代入構築
	 * @tparam TR 右オペランドの数値型
	 * @tparam PR 右オペランドの接頭辞
	 * @return this
	 */
	template<typename TR, class PR>
	constexpr Unit(Unit<TR, U, PR>&& r_operand){
		_value = static_cast<T>((static_cast<TR>(r_operand) * P::den * PR::num) / (P::num * PR::den));
	}

	/**
	 * @brief 同一単位系の代入
	 * @tparam TR 右オペランドの数値型
	 * @tparam PR 右オペランドの接頭辞
	 * @return this
	 */
	template<typename TR, class PR>
	constexpr Unit<T, U, P>& operator=(const Unit<TR, U, PR>& r_operand) &{
		_value = static_cast<T>((static_cast<TR>(r_operand) * P::den * PR::num) / (P::num * PR::den));
		return *this;
	}
	/**
	 * @brief 同一単位系の代入
	 * @tparam TR 右オペランドの数値型
	 * @tparam PR 右オペランドの接頭辞
	 * @return this
	 */
	template<typename TR, class PR>
	constexpr Unit<T, U, P>& operator=(Unit<TR, U, PR>&& r_operand) & noexcept{
		_value = static_cast<T>((static_cast<TR>(r_operand) * P::den * PR::num) / (P::num * PR::den));
		return *this;
	}



	/**
	 * @brief 明示的なキャスト
	 * @tparam A 数値型
	 * @return 値
	 */
	template<typename A>
	explicit constexpr operator A () const noexcept {
		static_assert(std::is_arithmetic<A>::value, "Type is not arithmetic.");
		return static_cast<A>(_value);
	}

	/**
	 * @brief 符号演算子
	 */
	constexpr Unit<T, U, P> operator+() const{
		return +_value;
	}
	/**
	 * @brief 符号演算子
	 */
	constexpr Unit<T, U, P> operator-() const{
		return -_value;
	}



	/*arithmetic and unit operator*/
	/**
	 * @brief 数値型との演算
	 * @tparam TR 右オペランドの型
	 * @return this
	 */
	template<typename TR>
	constexpr auto operator=(const TR& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<TR>, Unit<T, U, P>&>{
		_value = static_cast<T>(r_operand);
		return *this;
	}

	/**
	 * @brief 数値型との演算
	 * @tparam TR 右オペランドの型
	 * @return this
	 */
	template<typename TR>
	constexpr auto operator+=(const TR& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<TR>, Unit<T, U, P>&>{
		_value += static_cast<T>(r_operand);
		return *this;
	}
	/**
	 * @brief 数値型との演算
	 * @tparam TR 右オペランドの型
	 * @return this
	 */
	template<typename TR>
	constexpr auto operator-=(const TR& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<TR>, Unit<T, U, P>&>{
		_value -= static_cast<T>(r_operand);
		return *this;
	}
	/**
	 * @brief 数値型との演算
	 * @tparam TR 右オペランドの型
	 * @return this
	 */
	template<typename TR>
	constexpr auto operator*=(const TR& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<TR>, Unit<T, U, P>&>{
		_value *= static_cast<T>(r_operand);
		return *this;
	}
	/**
	 * @brief 数値型との演算
	 * @tparam TR 右オペランドの型
	 * @return this
	 */
	template<typename TR>
	constexpr auto operator/=(const TR& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<TR>, Unit<T, U, P>&>{
		_value /= static_cast<T>(r_operand);
		return *this;
	}


	/*same unit operator*/

	/**
	 * @brief 同一単位系の演算
	 * @tparam TR 右オペランドの数値型
	 * @tparam PR 右オペランドの接頭辞
	 * @return this
	 */
	template<typename TR, class PR>
	constexpr Unit<T, U, P>& operator+=(const Unit<TR, U, PR>& r_operand){
		_value += static_cast<T>((static_cast<TR>(r_operand) * P::den * PR::num) / (P::num * PR::den));
		return *this;
	}
	/**
	 * @brief 同一単位系の演算
	 * @tparam TR 右オペランドの数値型
	 * @tparam PR 右オペランドの接頭辞
	 * @return this
	 */
	template<typename TR, class PR>
	constexpr Unit<T, U, P>& operator-=(const Unit<TR, U, PR>& r_operand){
		_value -= static_cast<T>((static_cast<TR>(r_operand) * P::den * PR::num) / (P::num * PR::den));
		return *this;
	}
};



/*arithmetic and unit operator*/

/**
 * @brief 数値型との比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator<(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, bool>{
	return static_cast<T1>(l_operand) < r_operand;
}
/**
 * @brief 数値型との比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator>(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, bool>{
	return static_cast<T1>(l_operand) > r_operand;
}
/**
 * @brief 数値型との比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator<=(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, bool>{
	return !(l_operand > r_operand);
}
/**
 * @brief 数値型との比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator>=(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, bool>{
	return !(l_operand < r_operand);
}
/**
 * @brief 数値型との比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator==(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, bool>{
	return static_cast<T1>(l_operand) == r_operand;
}
/**
 * @brief 数値型との比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator!=(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, bool>{
	return !(l_operand == r_operand);
}

/**
 * @brief 数値型との演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 演算結果　Unit<T1, U, P>
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator+(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, Unit<T1, U, P>>{
	return Unit<T1, U, P>(l_operand) += r_operand;
}
/**
 * @brief 数値型との演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 演算結果　Unit<T1, U, P>
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator-(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, Unit<T1, U, P>>{
	return Unit<T1, U, P>(l_operand) -= r_operand;
}
/**
 * @brief 数値型との演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 演算結果　Unit<T1, U, P>
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator*(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, Unit<T1, U, P>>{
	return Unit<T1, U, P>(l_operand) *= r_operand;
}
/**
 * @brief 数値型との演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 左オペランドの接頭辞
 * @return 演算結果　Unit<T1, U, P>
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator/(const Unit<T1, U, P>& l_operand, const T2& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T2>, Unit<T1, U, P>>{
	return Unit<T1, U, P>(l_operand) /= r_operand;
}
/**
 * @brief 数値型との演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 右オペランドの接頭辞
 * @return 演算結果　Unit<T2, U, P>
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator+(const T1& l_operand, const Unit<T2, U, P>& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T1>, Unit<T2, U, P>>{
	return Unit<T2, U, P>(l_operand + static_cast<T2>(r_operand));
}
/**
 * @brief 数値型との演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 右オペランドの接頭辞
 * @return 演算結果　Unit<T2, U, P>
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator-(const T1& l_operand, const Unit<T2, U, P>& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T1>, Unit<T2, U, P>>{
	return Unit<T2, U, P>(l_operand - static_cast<T2>(r_operand));
}
/**
 * @brief 数値型との演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 右オペランドの接頭辞
 * @return 演算結果　Unit<T2, U, P>
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator*(const T1& l_operand, const Unit<T2, U, P>& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T1>, Unit<T2, U, P>>{
	return Unit<T2, U, P>(l_operand * static_cast<T2>(r_operand));
}
/**
 * @brief 数値型との演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 右オペランドの接頭辞
 * @return 演算結果　Unit<T2, U, P>
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator/(const T1& l_operand, const Unit<T2, U, P>& r_operand) -> typename std::enable_if_t<std::is_arithmetic_v<T1>, Unit<T2, U, P>>{
	return Unit<T2, U, P>(l_operand / static_cast<T2>(r_operand));
}



/*same unit operator*/

/**
 * @brief 同一単位系の比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P1 左オペランドの接頭辞
 * @tparam P2 右オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P1, class P2>
constexpr bool operator<(const Unit<T1, U, P1>& l_operand, const Unit<T2, U, P2>& r_operand) {
	return static_cast<T1>(l_operand) * P1::RATIO < static_cast<T2>(r_operand) * P2::RATIO;
}
/**
 * @brief 同一単位系の比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P1 左オペランドの接頭辞
 * @tparam P2 右オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P1, class P2>
constexpr bool operator>(const Unit<T1, U, P1>& l_operand, const Unit<T2, U, P2>& r_operand) {
	return static_cast<T1>(l_operand) * P1::RATIO > static_cast<T2>(r_operand) * P2::RATIO;
}
/**
 * @brief 同一単位系の比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P1 左オペランドの接頭辞
 * @tparam P2 右オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P1, class P2>
constexpr bool operator<=(const Unit<T1, U, P1>& l_operand, const Unit<T2, U, P2>& r_operand) {
	return !(l_operand > r_operand);
}
/**
 * @brief 同一単位系の比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P1 左オペランドの接頭辞
 * @tparam P2 右オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P1, class P2>
constexpr bool operator>=(const Unit<T1, U, P1>& l_operand, const Unit<T2, U, P2>& r_operand) {
	return !(l_operand < r_operand);
}
/**
 * @brief 同一単位系の比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P1 左オペランドの接頭辞
 * @tparam P2 右オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P1, class P2>
constexpr bool operator==(const Unit<T1, U, P1>& l_operand, const Unit<T2, U, P2>& r_operand) {
	return (static_cast<T1>(l_operand) * P1::num) / P2::den > (static_cast<T2>(r_operand) * P2::num) / P2::den;
}
/**
 * @brief 同一単位系の比較
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P1 左オペランドの接頭辞
 * @tparam P2 右オペランドの接頭辞
 * @return 比較結果
 */
template<typename T1, typename T2, Type U, class P1, class P2>
constexpr bool operator!=(const Unit<T1, U, P1>& l_operand, const Unit<T2, U, P2>& r_operand) {
	return !(l_operand == r_operand);
}
/**
 * @brief 同一単位系の演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 接頭辞
 * @return 演算結果 Unit<T, U, P>
 * @details TはT1とT2で算術演算した際の自動昇格型です
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator+(const Unit<T1, U, P>& l_operand, const Unit<T2, U, P>& r_operand) -> Unit<decltype(T1(0) + T2(0)), U, P>{
	return l_operand + static_cast<T2>(r_operand);
}
/**
 * @brief 同一単位系の演算
 * @tparam T1 左オペランドの数値型
 * @tparam T2 右オペランドの数値型
 * @tparam U 単位の種類
 * @tparam P 接頭辞
 * @return 演算結果 Unit<T, U, P>
 * @details TはT1とT2で算術演算した際の自動昇格型です
 */
template<typename T1, typename T2, Type U, class P>
constexpr auto operator-(const Unit<T1, U, P>& l_operand, const Unit<T2, U, P>& r_operand) -> Unit<decltype(T1(0) + T2(0)), U, P>{
	return l_operand - static_cast<T2>(r_operand);
}
}
}
