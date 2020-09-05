/*
 * 座標系クラス
 *行列計算は基本的にEigenで行う
 */
#pragma once

#include <NutLib/Global.hpp>
#include <array>

namespace nut{
template<typename T>
struct Coordinate{
	static_assert(std::is_arithmetic<T>::value, "Type is not arithmetic.");
	T x;
	T y;
	T theta;//方向角

	/*
	 * コンストラクタ
	 * 第一引数:X座標,第二引数:Y座標,第三引数:方向角
	 */
	constexpr Coordinate(T X = 0.0, T Y = 0.0, T Rad = 0.0) : x(X), y(Y), theta(Rad){}
	//明示的なコピーコンストラクタ(必須)
	constexpr Coordinate(const Coordinate<T>& copy) : x(copy.x), y(copy.y), theta(copy.theta){}
	constexpr Coordinate& operator=(const Coordinate<T>& obj){
		x = obj.x;
		y = obj.y;
		theta = obj.theta;
		return *this;
	}


	/*
	 * ベクトル取得関数
	 * 第一引数:取得ベクトル
	 */
	template<class V>
	constexpr void GetVector(V& vector) const{
		vector.x() = x;
		vector.y() = y;
	}
	/*
	 * 原点からの距離算出関数
	 * 戻り値:距離
	 */
	inline constexpr auto Norm() const -> decltype(x + 0.0f) {return sqrtf(powf(x, 2.0f) + powf(y, 2.0f));}//自動昇格でサイズ抑え

	/*
	 * 位置ベクトルの傾き算出関数
	 * 戻り値:角度
	 */
	constexpr auto Angle() const ->decltype(x + 0.0f) {return atan2f(y, x);}

	/*
	 * 算術演算子オーバーロード
	 * EigenのVectorとも一部互換
	 */
	constexpr Coordinate<T>& operator+=(const Coordinate<T>& r_operand){
		x += r_operand.x;
		y += r_operand.y;
		theta += r_operand.theta;
		return *this;
	}
	template<typename V>
	constexpr Coordinate<T>& operator+=(const V& r_operand){
		x += static_cast<T>(r_operand.x());
		y += static_cast<T>(r_operand.y());
		return *this;
	}
	constexpr Coordinate<T>& operator-=(const Coordinate<T>& r_operand){
		x -= r_operand.x;
		y -= r_operand.y;
		theta -= r_operand.theta;
		return *this;
	}
	template<typename V>
	constexpr Coordinate<T>& operator-=(const V& r_operand){
		x -= static_cast<T>(r_operand.x());
		y -= static_cast<T>(r_operand.y());
		return *this;
	}
	template<typename V>
	constexpr Coordinate<T>& operator*=(const V& r_operand){
		x *= static_cast<T>(r_operand);
		y *= static_cast<T>(r_operand);
		theta *= static_cast<T>(r_operand);
		return *this;
	}
	template<typename V>
	constexpr Coordinate<T>& operator/=(const V& r_operand){
		x /= static_cast<T>(r_operand);
		y /= static_cast<T>(r_operand);
		theta /= static_cast<T>(r_operand);
		return *this;
	}

	/*
	 * キャストのオーバーロード
	 */
	operator std::array<T, 3> ()const noexcept{return std::array<T, 3>(x, y, theta);}
	operator std::initializer_list<T> ()const noexcept{return std::initializer_list<T>(x, y, theta);}


	/*
	 * 添え字による要素アクセス
	 */
	constexpr const T& operator[](size_t index) const& {return *((&x) + index);}
	constexpr T& operator[](size_t index) & {return *((&x) + index);}
	constexpr T operator[](size_t index) const&& {return *((&x) + index);}
};

/*doubleに対する特殊化*/
template<>
constexpr auto Coordinate<double>::Norm() const ->decltype(x + 0.0f) {return sqrt(pow(x, 2.0) + pow(y, 2.0));}
template<>
constexpr auto Coordinate<double>::Angle() const ->decltype(x + 0.0f) {return atan2(y, x);}

/*a算術演算子オーバーロード(主にEigen　Vector系)*/
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
	return nut::Coordinate<T>(T{l_operand.x}-r_operand.x, T{l_operand.y}-r_operand.y, r_operand.theta);
}
}
