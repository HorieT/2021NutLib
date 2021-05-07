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
#include <array>

namespace nut{
/**
 * @brief  平面座標(x,y,theta)を表すクラス
 * @tparam T 内部パラメータの数値型
 * @attention Tが数値型でなければアサートを吐きます
 */
template<typename T>
class Coordinate{
	static_assert(std::is_arithmetic<T>::value, "Type is not arithmetic.");
private:
	T _x;
	T _y;
	T _theta;
public:
	constexpr const T& x() const&{return _x;}
	constexpr const T& y() const&{return _y;}
	constexpr const T& theta() const&{return _theta;}
	constexpr T& x() &{return _x;}
	constexpr T& y() &{return _y;}
	constexpr T& theta() &{return _theta;}
	constexpr T x() const&&{return _x;}
	constexpr T y() const&&{return _y;}
	constexpr T theta() const&&{return _theta;}



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
	 * @brief x,yの平面ベクトルを返します
	 * @tparam V ベクトル型
	 * @param[out] vector ベクトル
	 */
	template<class V>
	constexpr void GetVector(V& vector) const{
		vector.x() = _x;
		vector.y() = _y;
	}
	/**
	 * @brief x,yベクトルのノルムを返します
	 * @return ノルム
	 * @details	Tがdoubleであればdouble、それ以外ならfloatが返ります
	 */
	inline constexpr auto Norm() const -> decltype(_x + 0.0f) {
		return sqrtf(powf(_x, 2.0f) + powf(_y, 2.0f));
	}

	/**
	 * @brief x,yベクトルの角度を返します
	 * @return ベクトル角
	 * @details	Tがdoubleであればdouble、それ以外ならfloatが返ります
	 */
	constexpr auto Angle() const ->decltype(_x + 0.0f) {
		return atan2f(_y, _x);
	}

	/*Compound Assignmentのオーバーロード*/
	constexpr Coordinate<T>& operator+=(const Coordinate<T>& r_operand){
		_x += r_operand.x();
		_y += r_operand.y();
		_theta += r_operand.theta();
		return *this;
	}
	template<typename V>
	constexpr Coordinate<T>& operator+=(const V& r_operand){
		_x += static_cast<T>(r_operand.x());
		_y += static_cast<T>(r_operand.y());
		return *this;
	}
	constexpr Coordinate<T>& operator-=(const Coordinate<T>& r_operand){
		_x -= r_operand.x();
		_y -= r_operand.y();
		_theta -= r_operand.theta();
		return *this;
	}
	template<typename V>
	constexpr Coordinate<T>& operator-=(const V& r_operand){
		_x -= static_cast<T>(r_operand.x());
		_y -= static_cast<T>(r_operand.y());
		return *this;
	}
	template<typename V>
	constexpr Coordinate<T>& operator*=(const V& r_operand){
		_x *= static_cast<T>(r_operand);
		_y *= static_cast<T>(r_operand);
		return *this;
	}
	template<typename V>
	constexpr Coordinate<T>& operator/=(const V& r_operand){
		_x /= static_cast<T>(r_operand);
		_y /= static_cast<T>(r_operand);
		return *this;
	}


	/*Castのオーバーロード*/
	explicit operator std::array<T, 3> ()const noexcept{return std::array<T, 3>(_x, _y, _theta);}
	explicit operator std::initializer_list<T> ()const noexcept{return std::initializer_list<T>(_x, _y, _theta);}


	/*Array Subscriptのオーバーロード*/
	constexpr const T& operator[](size_t index) const& {return *((&_x) + index);}
	constexpr T& operator[](size_t index) & {return *((&_x) + index);}
	constexpr T operator[](size_t index) const&& {return *((&_x) + index);}
};

/**
 * Coordinate<T>のdouble特殊化
 */
template<>
constexpr auto Coordinate<double>::Norm() const ->decltype(_x + 0.0f) {return sqrt(pow(_x, 2.0) + pow(_y, 2.0));}
/**
 * Coordinate<T>のdouble特殊化
 */
template<>
constexpr auto Coordinate<double>::Angle() const ->decltype(_x + 0.0f) {return atan2(_y, _x);}


/*Arithmeticのオーバーロード*/
template<typename T>
constexpr nut::Coordinate<T> operator+(const nut::Coordinate<T>& l_operand, const nut::Coordinate<T>& r_operand){
	return nut::Coordinate<T>(l_operand) += r_operand;
}
template<typename T>
constexpr nut::Coordinate<T> operator-(const nut::Coordinate<T>& l_operand, const nut::Coordinate<T>& r_operand){
	return nut::Coordinate<T>(l_operand) -= r_operand;
}
template<typename T>
constexpr nut::Coordinate<T> operator*(const nut::Coordinate<T>& l_operand, const double& r_operand){
	return nut::Coordinate<T>(l_operand) *= r_operand;
}
template<typename T>
constexpr nut::Coordinate<T> operator/(const nut::Coordinate<T>& l_operand, const double& r_operand){
	return nut::Coordinate<T>(l_operand) /= r_operand;
}

template<typename T, typename U>
constexpr nut::Coordinate<T> operator+(const nut::Coordinate<T>& l_operand, const U& r_operand){
	return nut::Coordinate<T>(l_operand) += r_operand;
}
template<typename T, typename U>
constexpr nut::Coordinate<T> operator+(const U& l_operand, const nut::Coordinate<T>& r_operand){
	return nut::Coordinate<T>(r_operand) += l_operand;
}
template<typename T, typename U>
constexpr nut::Coordinate<T> operator-(const nut::Coordinate<T>& l_operand, const U& r_operand){
	return nut::Coordinate<T>(l_operand) -= r_operand;
}
template<typename T, typename U>
constexpr nut::Coordinate<T> operator-(const U& l_operand, const nut::Coordinate<T>& r_operand){
	return nut::Coordinate<T>(T{l_operand.x()}-r_operand.x(), T{l_operand.y()}-r_operand.y(), r_operand.theta());
}
}
