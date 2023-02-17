#pragma once
namespace utils{

template <class T>
constexpr T constrain(T x, T min, T max) {
	if(x<min) return min;
	else if(max<x) return max;
	else return x;
}

}
