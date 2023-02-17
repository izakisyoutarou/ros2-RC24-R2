#pragma once
namespace utils{

// math defines
static constexpr double d_pi = 3.1415926535897932384626433832795;
static constexpr float f_pi = (float)d_pi;

// Pi
template <class T>
constexpr T pi() { return d_pi; }
template <>
inline constexpr float pi() { return f_pi; }

template <class T>
constexpr T constrain(T x, T min, T max) {
	if(x<min) return min;
	else if(max<x) return max;
	else return x;
}

template <class T>
constexpr T dtor(T x) { return pi<T>()*x/180; }

template <class T>
constexpr T rtod(T x) { return x*180/pi<T>(); }

}
