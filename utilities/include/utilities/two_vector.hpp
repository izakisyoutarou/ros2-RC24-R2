#pragma once
/*******************************************
 * TwoVector ver2.0
 * 2D vector calculation class
 *
 * [Dependency]
 *
 * [Note]
 *
 * [Author]
 * Yuta Uehara
 *
 * [Change history]
 * ver2.1 2022/02/24 Refactor. Use cmath.
 * ver2.0 2022/02/06 Merge RC21_robot_lib into ichigoplus.
 * ver1.2 2021/01/12 change length(), angle(), operator+() and operator-() to const member function.
 * ver1.1 2021/01/07 add include math.h.
 * ver1.0 2020/03/05 The first version
 ******************************************/

#include <cmath>

namespace utils {
template <class T0>
class TwoVector {
public:
	constexpr TwoVector(){}
	constexpr TwoVector(T0 x, T0 y): x(x), y(y) {}
	constexpr void cartesianInit(T0 x, T0 y) { this->x=x; this->y=y; }
	constexpr void polarInit(T0 length, T0 angle) { x = length * std::cos(angle); y = length * std::sin(angle); }

	constexpr T0 length() const { return std::sqrt(x*x + y*y); }
	constexpr T0 angle() const { return std::atan2(y,x); }

	T0 x = 0;
	T0 y = 0;

	template <class T1>
	constexpr void operator=(TwoVector<T1> vec) {
		x = vec.x;
		y = vec.y;
	}
	constexpr const TwoVector<T0> operator+() const {
		return *this;
	}
	constexpr const TwoVector<T0> operator-() const {
		return TwoVector<T0>(-x,-y);
	}
	template <class T1>
	constexpr void operator+=(const TwoVector<T1> &vec) {
		x += vec.x;
		y += vec.y;
	}
	template <class T1>
	constexpr void operator-=(const TwoVector<T1>& vec) {
		x -= vec.x;
		y -= vec.y;
	}
	constexpr void operator*=(T0 value) {
		x *= value;
		y *= value;
	}
	constexpr void operator/=(T0 value) {
		x /= value;
		y /= value;
	}

	template <class T_CAST>
	constexpr operator TwoVector<T_CAST>() const {
		return TwoVector<T_CAST>(static_cast<T_CAST>(x), static_cast<T_CAST>(y));
	}
};

template <class T0, class T1>
inline constexpr const auto innerProduct(const TwoVector<T0>& vec0, const TwoVector<T1>& vec1) {
	return vec0.x * vec1.x + vec0.y * vec1.y;
}

template <class T0, class T1>
inline constexpr const auto operator+(const TwoVector<T0>& vec0, const TwoVector<T1>& vec1) {
	TwoVector<decltype(T0()+T1())> vec(vec0);
	vec += vec1;
	return vec;
}
template <class T0, class T1>
inline constexpr const auto operator-(const TwoVector<T0>& vec0, const TwoVector<T1>& vec1) {
	TwoVector<decltype(T0()-T1())> vec(vec0);
	vec -= vec1;
	return vec;
}
template <class T0, class T1>
inline constexpr const auto operator*(const TwoVector<T0>& vec0, const T1 value) {
	TwoVector<decltype(T0()*T1())> vec(vec0);
	vec *= value;
	return vec;
}
template <class T0, class T1>
inline constexpr const auto operator*(const T0 value, const TwoVector<T1>& vec0) {
	TwoVector<decltype(T0()*T1())> vec(vec0);
	vec *= value;
	return vec;
}
template <class T0, class T1>
inline constexpr const auto operator/(const TwoVector<T0>& vec0, const T1 value) {
	TwoVector<decltype(T0()/T1())> vec(vec0);
	vec /= value;
	return vec;
}

using TwoVectord = TwoVector<double>;
using TwoVectorf = TwoVector<float>;

}
