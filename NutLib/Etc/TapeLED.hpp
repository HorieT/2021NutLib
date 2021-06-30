/*
 *
 */
#pragma once

#include "../Global.hpp"
#include "PWMch.hpp"


namespace nut{
class TapeLED{
private:
	PWMch _r_ch;
	PWMch _g_ch;
	PWMch _b_ch;

public:
	TapeLED(PWMch r_ch, PWMch g_ch, PWMch b_ch) : _r_ch(r_ch), _g_ch(g_ch), _b_ch(b_ch){

	}


	void Start(){
		_r_ch.Start();
		_g_ch.Start();
		_b_ch.Start();
	}

	void Stop(){
		_r_ch.Stop();
		_g_ch.Stop();
		_b_ch.Stop();
	}

	/**
	 * @brief 0~1までの値を入力
	 */
	void SetRGB(float r, float g, float b){
		_r_ch.WriteParCCR(r * 100.0f);
		_g_ch.WriteParCCR(g * 100.0f);
		_b_ch.WriteParCCR(b * 100.0f);
	}
	/**
	 * @brief 0~1までの値を入力
	 */
	void SetRGB(std::array<float, 3> val){
		SetRGB(val[0], val[1], val[2]);
	}
	/**
	 * @brief 0~255までの値を入力
	 */
	void SetRGB(uint8_t r, uint8_t g, uint8_t b){
		SetRGB(r / 255.0f, g / 255.0f, b / 255.0f);
	}
	/**
	 * @brief 0~255までの値を入力
	 */
	void SetRGB(std::array<uint8_t, 3> val){
		SetRGB(val[0], val[1], val[2]);
	}


	/**
	 * @brief 0~1までの値を入力
	 */
	void SetHSV(float h, float s, float v){
		float r = v;
		float g = v;
		float b = v;
		if (s > 0.0f) {
		    h *= 6.0f;
		    float f = h - std::rint(h);
		    switch (static_cast<uint8_t>(std::rint(h))) {
		        default:
		        case 0:
		            g *= 1 - s * (1 - f);
		            b *= 1 - s;
		            break;
		        case 1:
		            r *= 1 - s * f;
		            b *= 1 - s;
		            break;
		        case 2:
		            r *= 1 - s;
		            b *= 1 - s * (1 - f);
		            break;
		        case 3:
		            r *= 1 - s;
		            g *= 1 - s * f;
		            break;
		        case 4:
		            r *= 1 - s * (1 - f);
		            g *= 1 - s;
		            break;
		        case 5:
		            g *= 1 - s;
		            b *= 1 - s * f;
		            break;
		    }
		}
		SetRGB(r, g, b);
	}
	/**
	 * @brief 0~1までの値を入力
	 */
	void SetHSV(std::array<float, 3> val){
		SetHSV(val[0], val[1], val[2]);
	}
	/**
	 * @brief 0~255までの値を入力
	 */
	void SetHSV(uint8_t r, uint8_t g, uint8_t b){
		SetHSV(r / 255.0f, g / 255.0f, b / 255.0f);
	}
	/**
	 * @brief 0~255までの値を入力
	 */
	void SetHSV(std::array<uint8_t, 3> val){
		SetHSV(val[0], val[1], val[2]);
	}
};
}
