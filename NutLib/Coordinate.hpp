/*
 * ���W�n�N���X
 *�s��v�Z�͊�{�I��Eigen�ōs��
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
	T theta;//�����p

	/*
	 * �R���X�g���N�^
	 * ������:X���W,������:Y���W,��O����:�����p
	 */
	constexpr Coordinate(T X = 0.0, T Y = 0.0, T Rad = 0.0) : x(X), y(Y), theta(Rad){}
	//�����I�ȃR�s�[�R���X�g���N�^(�K�{)
	constexpr Coordinate(const Coordinate<T>& copy) : x(copy.x), y(copy.y), theta(copy.theta){}
	constexpr Coordinate& operator=(const Coordinate<T>& obj){
		x = obj.x;
		y = obj.y;
		theta = obj.theta;
		return *this;
	}


	/*
	 * �x�N�g���擾�֐�
	 * ������:�擾�x�N�g��
	 */
	template<class V>
	constexpr void GetVector(V& vector) const{
		vector.x() = x;
		vector.y() = y;
	}
	/*
	 * ���_����̋����Z�o�֐�
	 * �߂�l:����
	 */
	inline constexpr auto Norm() const -> decltype(x + 0.0f) {return sqrtf(powf(x, 2.0f) + powf(y, 2.0f));}//�������i�ŃT�C�Y�}��

	/*
	 * �ʒu�x�N�g���̌X���Z�o�֐�
	 * �߂�l:�p�x
	 */
	constexpr auto Angle() const ->decltype(x + 0.0f) {return atan2f(y, x);}

	/*
	 * �Z�p���Z�q�I�[�o�[���[�h
	 * Eigen��Vector�Ƃ��ꕔ�݊�
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
	 * �L���X�g�̃I�[�o�[���[�h
	 */
	operator std::array<T, 3> ()const noexcept{return std::array<T, 3>(x, y, theta);}
	operator std::initializer_list<T> ()const noexcept{return std::initializer_list<T>(x, y, theta);}


	/*
	 * �Y�����ɂ��v�f�A�N�Z�X
	 */
	constexpr const T& operator[](size_t index) const& {return *((&x) + index);}
	constexpr T& operator[](size_t index) & {return *((&x) + index);}
	constexpr T operator[](size_t index) const&& {return *((&x) + index);}
};

/*double�ɑ΂�����ꉻ*/
template<>
constexpr auto Coordinate<double>::Norm() const ->decltype(x + 0.0f) {return sqrt(pow(x, 2.0) + pow(y, 2.0));}
template<>
constexpr auto Coordinate<double>::Angle() const ->decltype(x + 0.0f) {return atan2(y, x);}

/*a�Z�p���Z�q�I�[�o�[���[�h(���Eigen�@Vector�n)*/
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
