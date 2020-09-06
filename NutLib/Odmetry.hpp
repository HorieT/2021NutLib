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
#include <array>
#include <memory>

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

	/*計算リソース*/
	std::array<float, 2> _encoder_radPos;//エンコーダ座標角
	std::array<float, 2> _encoder_norm;//エンコーダ座標ノルム
	float _sin_encoder_directiion_diff;
	std::array<float, 2> _cos_encoder_direction;
	std::array<float, 2> _sin_encoder_direction;
	std::array<float, 2> _cos_encoder_rad;
	std::array<float, 2> _sin_encoder_rad;



	/**
	 * @brief 周期ごとの座標計算
	 */
	void CalcPos(){
		//測距輪
		std::array<float, 2> distance {
			_encoder.at(0)->GetDistanceAndReset(),
			_encoder.at(1)->GetDistanceAndReset()
		};
		//前回座標
		const Coordinate<float> last_pos = _position;
		//代入時間差緩和
		Coordinate<float> now_position = _position;
		now_position.theta() = _start_position.theta() + _imu->GetGlobalAngle().z();
		if(fabs(now_position.theta()) > static_cast<float>(M_PI))//piより大きい角度の修正
			now_position.theta() += (now_position.theta() > 0.0f) ? -2.0f*static_cast<float>(M_PI) : 2.0f*static_cast<float>(M_PI);

		//回転量
		const float rot_vel = _imu->GetGlobalRot().z() * static_cast<float>(_scheduler.GetPeriod()) / 1000.0f;

		//移動量座標変換
		std::array<Eigen::Vector2f, 2> distance_vec{
			Eigen::Vector2f{distance.at(0) * _cos_encoder_direction.at(0), distance.at(0) * _sin_encoder_direction.at(0)},
			Eigen::Vector2f{distance.at(1) * _cos_encoder_direction.at(1), distance.at(1) * _sin_encoder_direction.at(1)}};


		for(uint8_t i = 0;i < 2;++i){
			//回転による測距輪の偏差
			float rot_len = rot_vel * _encoder_norm.at(i);
			Eigen::Vector2f calc_vec =
					distance_vec.at(i) - Eigen::Vector2f(rot_len * -_sin_encoder_rad.at(i), rot_len * _cos_encoder_rad.at(i));
			distance.at(i) = sqrtf(calc_vec.x() * calc_vec.x() + calc_vec.y() * calc_vec.y()) * cos(_encoder.at(i)->GetPosition().theta() - atan2f(calc_vec.y(), calc_vec.x()));
		}
		//移動値積分
		now_position.x() +=
				(distance.at(0) * sin(_encoder.at(1)->GetPosition().theta() + last_pos.theta()) - distance.at(1) * sin(_encoder.at(0)->GetPosition().theta() + last_pos.theta())) /
				(-_sin_encoder_directiion_diff);
		now_position.y() +=
				(distance.at(0) * cos(_encoder.at(1)->GetPosition().theta() + last_pos.theta()) - distance.at(1) * cos(_encoder.at(0)->GetPosition().theta()+ last_pos.theta())) /
				(_sin_encoder_directiion_diff);


		//計算値
		_position = now_position;
		_velocity = (now_position - last_pos) / static_cast<float>(_scheduler.GetPeriod()) * 1000.0f;
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
	Odmetry(uint32_t period, const std::shared_ptr<IMU>& imu, const std::shared_ptr<EncoderWheel>& enc1, const std::shared_ptr<EncoderWheel>& enc2, Coordinate<float> start) :
		_scheduler([this]{CalcPos();}, period), _imu(imu), _encoder({enc1, enc2}), _start_position(start){

		_position = _start_position;

		//計算リソース
		for(uint8_t i = 0;i < 2;++i){
			_encoder_radPos.at(i) = _encoder.at(i)->GetPosition().Angle();
			_encoder_norm.at(i) = _encoder.at(i)->GetPosition().Norm();
			_cos_encoder_direction.at(i) = cos(_encoder.at(i)->GetPosition().theta());
			_sin_encoder_direction.at(i) = sin(_encoder.at(i)->GetPosition().theta());
			_cos_encoder_rad.at(i) = cosf(_encoder_radPos.at(i));
			_sin_encoder_rad.at(i) = sinf(_encoder_radPos.at(i));
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
		_position = position;
	}
};
}
