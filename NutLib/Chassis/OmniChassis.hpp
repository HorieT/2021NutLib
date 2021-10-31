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
#include "../Motor/Motor.hpp"
#include <memory>


namespace nut{
/**
 * @example Chassis.cpp
 */
/**
 * @brief オムニ足回りクラス
 * @tparam N オムニの数
 * @attention N<2でアサートを吐きます
 */
template<uint8_t N>
class OmniChassis : public Chassis{
	static_assert(N > 1U, "?????????????");
private:
	std::array<std::shared_ptr<DriveWheel>, N> _wheel;//!< 駆動輪
	std::array<Coordinate<float>, N> _wheel_position;//!< 駆動輪位置

	/*計算リソース*/
	std::array<Meter<float>, N> _wheel_length = {0.0f};
	std::array<std::array<float, 2>, N> _coefficient{{0.0f}};


	/**
	 * @brief 周期コールバック関数
	 */
	virtual void ScheduleTask() override{
		std::array<Meter<float>, N> input;
		{//演算
			uint8_t i = 0;
			for(auto& in : input){
				auto rot_component = _wheel_length[i] * _target_velocity.theta().f();
				in = _target_velocity.x() * _coefficient[i][0] + _target_velocity.y() * _coefficient[i][1] + rot_component;
				++i;
			}
		}
		{//入力
			uint8_t i = 0;
			for(auto& w : _wheel){
				w->SetMps(input[i].f());
				++i;
			}
		}
	}


	static std::array<std::shared_ptr<DriveWheel>, N> MakeWheelMirror(
			std::array<std::shared_ptr<Motor>, N> wheel_motor,
			Coordinate<float> first_wheel_position,
			Meter<float> wheel_diameter)
	{
		std::array<std::shared_ptr<DriveWheel>, N> wheel;

		uint8_t i = 0;
		for(auto& wh : wheel){
			Radian<float> rad = M_2PI_f * static_cast<float>(i) / static_cast<float>(N);
			wh = std::make_shared<DriveWheel>(wheel_motor[i], wheel_diameter, first_wheel_position.Rotation(rad));
			++i;
		}
		return wheel;
	}
	static std::array<std::shared_ptr<DriveWheel>, N> MakeWheel(
				std::array<std::shared_ptr<Motor>, N> wheel_motor,
				std::array<Coordinate<float>, N> wheel_position,
				Meter<float> wheel_diameter)
		{
			std::array<std::shared_ptr<DriveWheel>, N> wheel;

			uint8_t i = 0;
			for(auto& wh : wheel){
				wh = std::make_shared<DriveWheel>(wheel_motor[i], wheel_diameter, wheel_position[i]);
				++i;
			}
			return wheel;
		}


public:
	/**
	 * @brief コンストラクタ
	 * @param[in] period 制御周期
	 * @param[in] odmetry オドメータインスタンス
	 * @details オドメータを使わない場合はヌルポを入れてください
	 * @param[in] wheel オムニたち
	 */
	OmniChassis(
			MilliSecond<uint32_t> period,
			const std::shared_ptr<Odmetry>& odmetry,
			std::array<std::shared_ptr<DriveWheel>, N> wheel)
				: Chassis(period, odmetry), _wheel(wheel)
	{
		uint8_t i = 0;
		for(auto& c : _coefficient){
			float wheel_angle = _wheel[i]->GetPos().Angle().f() + M_PI_2_f;

			if(NormalizeRadian(_wheel[i]->GetPos().theta() - _wheel[i]->GetPos().Angle().f()) < 0.0){
				c[0] = -std::cos(wheel_angle);
				c[1] = -std::sin(wheel_angle);
				_wheel_length[i] = -_wheel[i]->GetPos().Norm();
			}
			else{
				c[0] = std::cos(wheel_angle);
				c[1] = std::sin(wheel_angle);
				_wheel_length[i] = _wheel[i]->GetPos().Norm();
			}

			++i;
		}
	}

	/**
	 * @brief コンストラクタ
	 * @param[in] period 制御周期
	 * @param[in] odmetry オドメータインスタンス
	 * @details オドメータを使わない場合はヌルポを入れてください
	 * @param[in] wheel_motor オムニのモータたち
	 * @param[in] wheel_position wheel_motorのオムニの座標
	 * @param[in] wheel_diameter ホイール径
	 */
	OmniChassis(
			MilliSecond<uint32_t> period,
			const std::shared_ptr<Odmetry>& odmetry,
			std::array<std::shared_ptr<Motor>, N> wheel_motor,
			std::array<Coordinate<float>, N> wheel_position,
			Meter<float> wheel_diameter)
				: OmniChassis(period, odmetry, MakeWheel(wheel_motor, wheel_position, wheel_diameter))
	{

	}
	/**
	 * @brief コンストラクタ(車輪は等角配置)
	 * @details オムニは同一円状に等角配置されている前提です<br>
	 * @param[in] period 制御周期
	 * @param[in] odmetry オドメータインスタンス
	 * @details オドメータを使わない場合はヌルポを入れてください
	 * @param[in] wheel_motor オムニのモータたち
	 * @param[in] first_wheel_position wheel_motor[0]のオムニの座標
	 * @details オムニは極座標で0°から反時計回りで定義します
	 * @param[in] wheel_diameter ホイール径
	 * @attention オムニの位置情報のthetaはオムニの向き判定にのみ使われます(同心円接線前提のため)
	 */
	OmniChassis(
			MilliSecond<uint32_t> period,
			const std::shared_ptr<Odmetry>& odmetry,
			std::array<std::shared_ptr<Motor>, N> wheel_motor,
			Coordinate<float> first_wheel_position,
			Meter<float> wheel_diameter)
				: OmniChassis(period, odmetry, MakeWheelMirror(wheel_motor, first_wheel_position, wheel_diameter))
	{

	}




	/**
	 * @brief デストラクタ
	 */
	virtual ~OmniChassis(){

	}
};
}
