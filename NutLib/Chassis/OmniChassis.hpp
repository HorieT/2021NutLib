/**
 * @file OmniChassis.hpp
 * @brief オムニ足回り
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "../Global.hpp"
#include "../Coordinate.hpp"
#include "../Odmetry.hpp"
#include "Chassis.hpp"
#include "../Motor/DriveWheel.hpp"
#include <memory>


namespace nut{
/**
 * @brief オムニ足回りクラス
 * @details オムニは同一円状に等角配置されている前提です<br>
 * オムニは極座標で0°から反時計回りで定義します
 * @tparam N オムニの数
 * @attention N<2でアサートを吐きます
 */
template<uint8_t N>
class OmniChassis : public Chassis{
	static_assert(N > 1U, "?????????????");

protected:
	std::array<std::shared_ptr<DriveWheel>, N> _wheel;//!< 駆動輪
	const Coordinate<float> _wheel_position;//!< 駆動輪位置
	const bool _wheel_polarity;//!< 駆動輪極性

	/*計算リソース*/
	const float _wheel_length = 0;
	std::array<std::array<float, 2>, N> _coefficient{{0.0f}};

	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		std::array<float, N> input;
		float rot_component = _wheel_length * _target_velocity.theta();
		//極性
		if(_wheel_polarity){
			uint8_t i = 0;
			for(auto& in : input){
				in = _target_velocity.x() * _coefficient[i][0] + _target_velocity.y() * _coefficient[i][1] + rot_component;
				++i;
			}

		}else{
			uint8_t i = 0;
			for(auto& in : input){
				in = -_target_velocity.x() * _coefficient[i][0] - _target_velocity.y() * _coefficient[i][1] - rot_component;
				++i;
			}

		}
		{
			uint8_t i = 0;
			for(auto& w : _wheel){
				w->SetMps(input[i]);
				++i;
			}
		}
	}

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] period 制御周期
	 * @param[in] odmetry オドメータインスタンス
	 * @details オドメータを使わない場合はヌルポを入れてください
	 * @param[in] wheel オムニたち
	 * @details オムニは極座標で0°から反時計回りで定義します
	 * @param[in] first_wheel_position wheel[0]のオムニの座標
	 * @param[in] wheel_polarity オムニの極性←いらないのでは？
	 */
	OmniChassis(
			MilliSecond<uint32_t> period,
			std::shared_ptr<Odmetry> odmetry,
			std::array<std::shared_ptr<DriveWheel>, N> wheel,
			Coordinate<float> first_wheel_position,
			bool wheel_polarity = true)
				: Chassis(period, odmetry), _wheel(wheel), _wheel_position(first_wheel_position), _wheel_polarity(wheel_polarity)
	{
		const_cast<float&>(_wheel_length) = _wheel_position.Norm() * 0.001;

		uint8_t i = 0;
		for(auto& c : _coefficient){
			c[0] = -std::cos(_wheel_position.theta() + static_cast<float>(M_PI) * 2.0f * static_cast<float>(i) / static_cast<float>(N));
			c[1] = -std::sin(_wheel_position.theta() + static_cast<float>(M_PI) * 2.0f * static_cast<float>(i)/ static_cast<float>(N));
			++i;
		}
	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~OmniChassis(){

	}
};
}
