/**
 * @file Coordinate.hpp
 * @brief 平面座標
 * @details Eigenとの互換性あり
 * @author Horie
 * @date 2020/9
 * @attention インターフェースが変わるような大幅更新を行う予定なので注意
 */
#pragma once

#include "Global.hpp"
#include "Unit/UnitCore.hpp"
#include <array>

namespace nut{
/**
 * @brief  平面座標(x,y,theta)を表すクラス
 * 	デフォで単位系はmeter,radian
 * @tparam T 内部パラメータの数値型
 * @attention Tが数値型でなければアサートを吐きます
 * @tparam P_XY xy座標の接頭辞
 * @attention U_XYがunit::PrefixBaseの派生でなければアサートを吐きます
 */
template<typename T, typename P_XY = unit::Prefix<1, 1>>
class Coordinate{
	static_assert(std::is_arithmetic_v<T>, "Type is not arithmetic.");
	static_assert(std::is_base_of_v<unit::PrefixBase, P_XY>, "Type is not base of unit::PrefixBase.");
private:
	using Unit_XY = unit::Unit<T, nut::unit::Type::meter, P_XY>;
	Unit_XY _x;
	Unit_XY _y;
	Radian<T> _theta;
public:
	constexpr const Unit_XY& x() const&{return _x;}
	constexpr const Unit_XY& y() const&{return _y;}
	constexpr const Radian<T>& theta() const&{return _theta;}
	constexpr Unit_XY& x() &{return _x;}
	constexpr Unit_XY& y() &{return _y;}
	constexpr Radian<T>& theta() &{return _theta;}
	constexpr Unit_XY x() const&&{return _x;}
	constexpr Unit_XY y() const&&{return _y;}
	constexpr Radian<T> theta() const&&{return _theta;}



	/**
	 * @brief  デフォルトコンストラクタ
	 */
	constexpr Coordinate() : _x(0.0), _y(0.0), _theta(0.0){}
	/**
	 * @brief  要素初期化のデフォルトコンストラクタ
	 * @param[in] x x座標
	 * @param[in] y ｙ座標
	 * @param[in] rad 回転座標
	 */
	constexpr Coordinate(T x, T y , T rad = 0.0) : _x(x), _y(y), _theta(rad){}
	/**
	 * @brief  要素初期化のデフォルトコンストラクタ
	 * @param[in] x x座標
	 * @param[in] y ｙ座標
	 * @param[in] rad 回転座標
	 */
	constexpr Coordinate(Unit_XY x, Unit_XY y , Radian<T> rad = 0.0) : _x(x), _y(y), _theta(rad){}




	/**
	 * @brief x,yの平面ベクトルを返します
	 * @tparam V ベクトル型
	 * @param[out] vector ベクトル
	 */
	template<class V>
	constexpr void GetVector(V& vector) const{
		vector.x() = _x.f();
		vector.y() = _y.f();
	}
	/**
	 * @brief x,yベクトルのノルムを返します
	 * @return ノルム
	 * @details	Tがdoubleであればdouble、それ以外ならfloatが返ります
	 */
	constexpr auto Norm() const -> unit::Unit<decltype(T(0) + 0.0f), nut::unit::Type::meter, P_XY> {
		if constexpr(std::is_same_v<T, double>)
			return sqrt(pow(_x.d(), 2.0) + pow(_y.d(), 2.0));
		else
			return sqrtf(powf(_x.f(), 2.0f) + powf(_y.f(), 2.0f));
	}

	/**
	 * @brief x,yベクトルの角度を返します
	 * @return ベクトル角
	 * @details	Tがdoubleであればdouble、それ以外ならfloatが返ります
	 */
	constexpr auto Angle() const -> Radian<decltype(T(0) + 0.0f)> {
		if constexpr(std::is_same_v<T, double>)
			return atan2(_y.d(), _x.d());
		else
			return atan2f(_y.f(), _x.f());
	}


	/**
	 * @brief 座標を回転させて返します
	 * @return 回転角
	 */
	template<typename U>
	constexpr auto Rotation(Radian<U> rad) const{
		Coordinate<T, P_XY> val;
		val.x() = x() * std::cos(rad.value()) - y() * std::sin(rad.value());
		val.y() = x() * std::sin(rad.value()) + y() * std::cos(rad.value());
		val.theta() = theta() + rad;
		return val;
	}



	/*Compound Assignmentのオーバーロード*/
	template<typename T2, typename P>
	constexpr auto& operator+=(const Coordinate<T2, P>& r_operand){
		_x += r_operand.x();
		_y += r_operand.y();
		_theta += r_operand.theta();
		return *this;
	}
	template<typename T2, typename P>
	constexpr auto& operator-=(const Coordinate<T2, P>& r_operand){
		_x -= r_operand.x();
		_y -= r_operand.y();
		_theta -= r_operand.theta();
		return *this;
	}

	template<typename V>
	constexpr auto& operator+=(const V& r_operand){
		_x += static_cast<T>(r_operand.x());
		_y += static_cast<T>(r_operand.y());
		return *this;
	}
	template<typename V>
	constexpr auto& operator-=(const V& r_operand){
		_x -= static_cast<T>(r_operand.x());
		_y -= static_cast<T>(r_operand.y());
		return *this;
	}

	template<typename T2>
	constexpr auto operator*=(const T2& r_operand) -> std::enable_if_t<std::is_arithmetic_v<T2>, decltype(*this)&>{
		_x *= static_cast<T>(r_operand);
		_y *= static_cast<T>(r_operand);
		return *this;
	}
	template<typename T2>
	constexpr auto operator/=(const T2& r_operand) -> std::enable_if_t<std::is_arithmetic_v<T2>, decltype(*this)&>{
		_x /= static_cast<T>(r_operand);
		_y /= static_cast<T>(r_operand);
		return *this;
	}


	/*Castのオーバーロード*/
	explicit operator std::array<T, 3> ()const noexcept{
		return std::array<T, 3>(_x.value(), _y.value(), _theta.value());
	}
	explicit operator std::initializer_list<T> ()const noexcept{
		return std::initializer_list<T>(_x.value(), _y.value(), _theta.value());
	}


	/*Array Subscriptのオーバーロード*/
	/*
	[[deprecated("")]]
	constexpr const T& operator[](size_t index) const& {return *((&_x.f()) + index);}
	constexpr T& operator[](size_t index) & {return *((&_x.f()) + index);}
	constexpr T operator[](size_t index) const&& {return *((&_x.f()) + index);}
	*/
};




/*Arithmeticのオーバーロード*/
template<typename T, typename P_XY>
constexpr nut::Coordinate<T, P_XY> operator+(const nut::Coordinate<T, P_XY>& l_operand, const nut::Coordinate<T, P_XY>& r_operand){
	return nut::Coordinate<T, P_XY>(l_operand) += r_operand;
}
template<typename T, typename P_XY>
constexpr nut::Coordinate<T, P_XY> operator-(const nut::Coordinate<T, P_XY>& l_operand, const nut::Coordinate<T, P_XY>& r_operand){
	return nut::Coordinate<T, P_XY>(l_operand) -= r_operand;
}


template<typename T, typename P_XY, typename V>
constexpr nut::Coordinate<T, P_XY> operator+(const nut::Coordinate<T, P_XY>& l_operand, const V& r_operand){
	return nut::Coordinate<T, P_XY>(l_operand) += r_operand;
}
template<typename T, typename P_XY, typename V>
constexpr nut::Coordinate<T, P_XY> operator+(const V& l_operand, const nut::Coordinate<T, P_XY>& r_operand){
	return nut::Coordinate<T, P_XY>(r_operand) += l_operand;
}
template<typename T, typename P_XY, typename V>
constexpr nut::Coordinate<T, P_XY> operator-(const nut::Coordinate<T, P_XY>& l_operand, const V& r_operand){
	return nut::Coordinate<T, P_XY>(l_operand) -= r_operand;
}
template<typename T, typename P_XY, typename V>
constexpr nut::Coordinate<T, P_XY> operator-(const V& l_operand, const nut::Coordinate<T, P_XY>& r_operand){
	return nut::Coordinate<T, P_XY>(T{l_operand.x()}-r_operand.x(), T{l_operand.y()}-r_operand.y(), r_operand.theta());
}

template<typename T, typename P_XY, typename T2>
constexpr auto operator*(const nut::Coordinate<T, P_XY>& l_operand, const T2& r_operand) -> std::enable_if_t<std::is_arithmetic_v<T2>, nut::Coordinate<T, P_XY>>{
	return nut::Coordinate<T, P_XY>(l_operand) *= r_operand;
}
template<typename T, typename P_XY, typename T2>
constexpr auto operator/(const nut::Coordinate<T, P_XY>& l_operand, const T2& r_operand) -> std::enable_if_t<std::is_arithmetic_v<T2>, nut::Coordinate<T, P_XY>>{
	return nut::Coordinate<T, P_XY>(l_operand) /= r_operand;
}
}
