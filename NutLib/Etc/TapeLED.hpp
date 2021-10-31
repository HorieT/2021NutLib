/*
 *
 */
#pragma once

#include "../Global.hpp"
#include "PWMch.hpp"


namespace nut{
/**
 * @brief テープLEDクラス
 */
class TapeLED{
private:
	PWMch _r_ch;
	PWMch _g_ch;
	PWMch _b_ch;

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] r_ch 赤ch
	 * @param[in] g_ch 緑ch
	 * @param[in] b_ch 青ch
	 */
	TapeLED(PWMch r_ch, PWMch g_ch, PWMch b_ch) : _r_ch(r_ch), _g_ch(g_ch), _b_ch(b_ch){

	}


	/**
	 * @brief LED制御開始
	 */
	void Start(){
		_r_ch.Start();
		_g_ch.Start();
		_b_ch.Start();
	}
	/**
	 * @brief LED制御停止
	 */
	void Stop(){
		_r_ch.Stop();
		_g_ch.Stop();
		_b_ch.Stop();
	}

	/**
	 * @brief RGBセット
	 * @details 0~1までの値を入力
	 * @param[in] r R値
	 * @param[in] g G値
	 * @param[in] b B値
	 */
	void SetRGB(float r, float g, float b){
		_r_ch.WriteParCCR(r * 100.0f);
		_g_ch.WriteParCCR(g * 100.0f);
		_b_ch.WriteParCCR(b * 100.0f);
	}
	/**
	 * @brief RGBセット
	 * @details 0~1までの値を入力
	 * @param[in] val RGB値セット
	 */
	void SetRGB(std::array<float, 3> val){
		SetRGB(val[0], val[1], val[2]);
	}
	/**
	 * @brief RGBセット
	 * @details 0~255までの値を入力
	 * @param[in] r R値
	 * @param[in] g G値
	 * @param[in] b B値
	 */
	void SetRGB(uint8_t r, uint8_t g, uint8_t b){
		SetRGB(r / 255.0f, g / 255.0f, b / 255.0f);
	}
	/**
	 * @brief RGBセット
	 * @details 0~255までの値を入力
	 * @param[in] val RGB値セット
	 */
	void SetRGB(std::array<uint8_t, 3> val){
		SetRGB(val[0], val[1], val[2]);
	}


	/**
	 * @brief HSVセット
	 * @details 0~1までの値を入力
	 * @param[in] h H値
	 * @param[in] s S値
	 * @param[in] v V値
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
	 * @brief HSVセット
	 * @details 0~1までの値を入力
	 * @param[in] val HSV値セット
	 */
	void SetHSV(std::array<float, 3> val){
		SetHSV(val[0], val[1], val[2]);
	}
	/**
	 * @brief HSVセット
	 * @details 0~255までの値を入力
	 * @param[in] h H値
	 * @param[in] s S値
	 * @param[in] v V値
	 */
	void SetHSV(uint8_t h, uint8_t s, uint8_t v){
		SetHSV(h / 255.0f, s / 255.0f, v / 255.0f);
	}
	/**
	 * @brief HSVセット
	 * @details 0~255までの値を入力
	 * @param[in] val HSV値セット
	 */
	void SetHSV(std::array<uint8_t, 3> val){
		SetHSV(val[0], val[1], val[2]);
	}
};
}
