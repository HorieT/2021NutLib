/**
 * @file Odmetry.hpp
 * @brief オドメータ
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Global.hpp"
#include "TimeScheduler.hpp"
#include "Coordinate.hpp"
#include "Sensor/Encoder/EncoderWheel.hpp"
#include "Sensor/IMU/IMU.hpp"
#include "ControlSystem/MinCutoff.hpp"
#include <array>
#include <memory>
#include <functional>

namespace nut {

/**
 * @brief オドメータのクラス<br>
 * 			二輪エンコーダと一つのジャイロに対応
 */
class Odmetry{
private:
	TimeScheduler<void> _scheduler;
	const std::shared_ptr<IMU> _imu;
	std::array<const std::shared_ptr<EncoderWheel>, 2> _encoder;

	Coordinate<float> _start_position;
	Coordinate<float> _position;
	Coordinate<float> _velocity;
	MinCutoff<float> _cutoff_filter{1.0e-9, -1.0e-9};
	std::function<void(Coordinate<float>, Odmetry*)>_calc_callback;

	/*
	 *  計算リソース
	 */
	std::array<Radian<float>, 2> _encoder_radPos;//エンコーダ座標角
	std::array<Meter<float>, 2> _encoder_norm;//エンコーダ座標ノルム
	float _sin_encoder_directiion_diff;
	std::array<float, 2> _cos_encoder_direction;
	std::array<float, 2> _sin_encoder_direction;
	std::array<float, 2> _cos_encoder_rad;
	std::array<float, 2> _sin_encoder_rad;



	/**
	 * @brief 周期ごとの座標計算
	 */
	void CalcPos(){
		//a回転量
		const float rot_vel = _imu->GetGlobalRot().z() * Second<float>(_scheduler.GetPeriod()).f();

		//2測距輪
		std::array<Meter<float>, 2> distance {
			_encoder.at(0)->GetDistanceAndReset(),
			_encoder.at(1)->GetDistanceAndReset()
		};
		// last pos stack
		const Coordinate<float> last_pos = _position;
		//a代入時間差緩和
		Coordinate<float> now_position = _position;

		now_position.theta() = _start_position.theta() + _imu->GetGlobalAngle().z();
		if(fabs(now_position.theta().f()) > M_PI_f)//piより大きい角度の修正
			now_position.theta() += (now_position.theta() > 0.0f) ? -M_2PI_f : 2.0f*static_cast<float>(M_PI);

		//a移動量座標変換
		std::array<Eigen::Vector2f, 2> distance_vec{
			Eigen::Vector2f{distance.at(0).f() * _cos_encoder_direction.at(0), distance.at(0).f() * _sin_encoder_direction.at(0)},
			Eigen::Vector2f{distance.at(1).f() * _cos_encoder_direction.at(1), distance.at(1).f() * _sin_encoder_direction.at(1)}};


		for(uint8_t i = 0;i < 2;++i){
			//a回転による測距輪の偏差
			float rot_len = rot_vel * _encoder_norm.at(i).f();
			Eigen::Vector2f calc_vec =
					distance_vec.at(i) - Eigen::Vector2f(rot_len * -_sin_encoder_rad.at(i), rot_len * _cos_encoder_rad.at(i));
			distance.at(i) = sqrtf(calc_vec.x() * calc_vec.x() + calc_vec.y() * calc_vec.y()) * cos(_encoder.at(i)->GetPosition().theta() - atan2f(calc_vec.y(), calc_vec.x()));
		}
		//a移動値積分
		now_position.x() += _cutoff_filter.Calculate(
				((distance.at(0) * sin(_encoder.at(1)->GetPosition().theta() + last_pos.theta())
						- distance.at(1) * sin(_encoder.at(0)->GetPosition().theta() + last_pos.theta())) /
				(-_sin_encoder_directiion_diff)).f());
		now_position.y() += _cutoff_filter.Calculate(
				((distance.at(0) * cos(_encoder.at(1)->GetPosition().theta() + last_pos.theta())
						- distance.at(1) * cos(_encoder.at(0)->GetPosition().theta() + last_pos.theta())) /
				(_sin_encoder_directiion_diff)).f());


		//a計算値
		Coordinate<float> tmp_vel = (now_position - last_pos) / Second<float>(_scheduler.GetPeriod()).f();
		tmp_vel.theta() = _imu->GetGlobalRot().z();
		_position = now_position;
		_velocity = tmp_vel;

		if(_calc_callback)_calc_callback({distance[0], distance[1], rot_vel}, this);
	}
public:
	/**
	 * @brief コンストラクタ
	 * @param[in] period 計算周期
	 * @param[in] imu ジャイロ持ちIMUのshared_ptr
	 * @param[in] enc1  測距輪のshared_ptr
	 * @param[in] enc2  測距輪のshared_ptr
	 * @param[in] start 初期位置
	 */
	Odmetry(MilliSecond<uint32_t> period,
			const std::shared_ptr<IMU>& imu,
			const std::shared_ptr<EncoderWheel>& enc1,
			const std::shared_ptr<EncoderWheel>& enc2,
			Coordinate<float> start) :
		_scheduler([this]{CalcPos();}, period), _imu(imu), _encoder({enc1, enc2}), _start_position(start){

		_position = _start_position;

		//a計算リソース
		for(uint8_t i = 0;i < 2;++i){
			_encoder_radPos.at(i) = _encoder.at(i)->GetPosition().Angle();
			_encoder_norm.at(i) = _encoder.at(i)->GetPosition().Norm();
			_cos_encoder_direction.at(i) = cos(_encoder.at(i)->GetPosition().theta());
			_sin_encoder_direction.at(i) = sin(_encoder.at(i)->GetPosition().theta());
			_cos_encoder_rad.at(i) = cos(_encoder_radPos.at(i));
			_sin_encoder_rad.at(i) = sin(_encoder_radPos.at(i));
			_sin_encoder_directiion_diff = sin(_encoder.at(0)->GetPosition().theta() - _encoder.at(1)->GetPosition().theta());
		}
	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~Odmetry(){}


	/**
	 * @brief 初期化関数
	 * @details センサ類及び周期処理を初期化します
	 */
	void Init(){
		_scheduler.Set();
		_imu->Init();
		_encoder.at(0)->Init();
		_encoder.at(1)->Init();
	}

	/**
	 * @brief 現在位置取得
	 * @return 現在位置
	 */
	const Coordinate<float>& GetPosition() const{
		return _position;
	}
	/**
	 * @brief 現在速度取得
	 * @return 現在速度
	 */
	const Coordinate<float>& GetVelocity() const{
		return _velocity;
	}

	/**
	 * @brief　位置のリセット
	 * @param[in] position リセット座標
	 */
	void ResetPosition(const Coordinate<float>& position){
		_start_position = position;
		_position = _start_position;
	}

	/**
	 * @brief 位置計算時コールバックセット
	 * @param[in] callback コールバック関数(第一引数[測距データ], 第二引数thisポインタ)
	 */
	void SetCalculationCallback(std::function<void(Coordinate<float>, Odmetry*)> callback){
		_calc_callback = callback;
	}
};
}
