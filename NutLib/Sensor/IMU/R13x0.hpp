/**
 * @file R13x0.hpp
 * @brief R1370P,R1350N共通
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "IMU.hpp"
#include "../../TimeScheduler.hpp"
#include "../../HALCallbacks/UART.hpp"
#include "Eigen/Geometry"
#include <cstring>



namespace nut{
/**
 * @brief R1370P,R1350N共通クラス
 */
class R13x0 : public IMU{
private:
	using RxCallbackIt = decltype(callback::UART_RxHalfComplete)::ExCallbackIterator;

	static constexpr uint8_t GYRO_DATA_SIZE = 15;
	static constexpr uint8_t GYRO_BUFF_SIZE = GYRO_DATA_SIZE * 2;
	static constexpr uint8_t GYRO_BUFF_SIZE_D = GYRO_BUFF_SIZE - 1;
	static constexpr uint32_t TIMEOUT_TIME = 25;
	static constexpr std::array<uint8_t, 13> RESET_COMMAND{'$', 'M', 'I', 'B', ',', 'R', 'E', 'S', 'E', 'T', '*', '8', '7'};

	static constexpr int8_t UART_PRIORITY =	2;//コールバックの優先度

	TimeScheduler<void> _scheduler;
	std::array<uint8_t, GYRO_BUFF_SIZE> _buff;
	UART_HandleTypeDef* const _huart;
	std::function<void(void)> _receive_callback;

	RxCallbackIt _rx_it;

	bool _init_flag = false;
	float _roll_offset = 0.0f;
	float _pitch_offset = 0.0f;
	Eigen::Matrix3f _roll_AxisAngle;
	Eigen::Matrix3f _pitch_AxisAngle;


	/**
	 * @brief タイムアウト関数
	 */
	void Timeout(){
		_sensor_acc = {0.0f, 0.0f, 0.0f};
		_sensor_rot = {0.0f, 0.0f, 0.0f};
		_global_acc = {0.0f, 0.0f, 0.0f};
		_global_rot = {0.0f, 0.0f, 0.0f};
		//_global_angle = {0.0f, 0.0f, 0.0f};
	}

	/**
	 * @brief 受信関数
	 * @param[in] huart uartハンドル
	 * @return 受信処理成功の可否
	 */
	virtual bool Receive(UART_HandleTypeDef* huart) final{
		if(huart == _huart){
			std::array<uint8_t, GYRO_BUFF_SIZE> tmp_buff(_buff);
			uint32_t ndtr_ptr = _huart->hdmarx->Instance->NDTR;

			for(uint8_t j = 0;j < GYRO_BUFF_SIZE;++j){
				if((tmp_buff[j] == 0xAA) &&
						(tmp_buff[(j + 1) % GYRO_BUFF_SIZE] == 0x00) &&
						(((j + ndtr_ptr) % GYRO_BUFF_SIZE) < GYRO_DATA_SIZE)){
					std::array<uint8_t, GYRO_DATA_SIZE - 2> read_data;

					for(uint8_t i = 0;i < GYRO_DATA_SIZE - 2;i++)
						read_data[i] = tmp_buff[(j + i + 2) % GYRO_BUFF_SIZE];

					volatile uint8_t check_sum = 0;
					for(auto it = read_data.begin(), end = std::next(read_data.end(), -1); it != end;++it)
						check_sum += *it;

					if(read_data[12] == check_sum){
						_scheduler.Reset();

						[[maybe_unused]]int16_t angle, angle_vel, acc;//隴ｦ蜻雁屓驕ｿ縺ｧunused莉倥￠縺ｦ繧九￠縺ｩ蠢�隕∽ｸ�譎ょ､画焚
						memcpy(&angle, &read_data[1], 2);
						memcpy(&angle_vel, &read_data[3], 2);
						_global_angle.z() = - static_cast<float>(angle) * M_PI_f / 18000.0f;
						_global_rot.z()=  - static_cast<float>(angle_vel) * M_PI_f / 18000.0f;

						memcpy(&acc, &read_data[5], 2);
						_sensor_acc.x() = static_cast<float>(acc);
						memcpy(&acc, &read_data[7], 2);
						_sensor_acc.y() = static_cast<float>(acc);
						memcpy(&acc, &read_data[9], 2);
						_sensor_acc.z() = static_cast<float>(acc);


						//accセンサ座標初期化
						if(!_init_flag){
							_init_flag = true;
							_roll_offset = std::atan2(_sensor_acc.y(), _sensor_acc.z());
							_pitch_offset = std::atan2(-_sensor_acc.x(), std::sqrt(std::pow(_sensor_acc.y(), 2) + std::pow(_sensor_acc.z(), 2)));

							Eigen::Vector3f rot_axis{1, 0, 0};
							_roll_AxisAngle = Eigen::AngleAxisf(_roll_offset, rot_axis);
							rot_axis << 0, 1, 0;
							_pitch_AxisAngle = Eigen::AngleAxisf(_pitch_offset, rot_axis);
						}


						Eigen::Vector3f tmp_acc = _roll_AxisAngle * _sensor_acc;
						_global_acc = _pitch_AxisAngle * tmp_acc;

						if(_receive_callback)_receive_callback();
						return true;
					}
				}
			}
		}
		return false;
	}


public:
	/**
	 * @brief コンストラクタ
	 * @param[in] huart uartハンドル
	 * @details uartは事前に通信仕様通りの設定を行い、DMA設定でCircularにしてください
	 */
	R13x0(UART_HandleTypeDef* huart) : _scheduler([this]{Timeout();}, TIMEOUT_TIME), _huart(huart){
	}
	/**
	 * @brief デストラクタ
	 */
	virtual ~R13x0(){_scheduler.Erase();}


	/**
	 * @brief 初期化関数
	 */
	virtual void Init() override final{
		if(_is_init)return;
		_is_init = true;
		_rx_it = callback::UART_RxHalfComplete.AddExclusiveCallback(UART_PRIORITY, [this](UART_HandleTypeDef* huart){return Receive(huart);});
		HAL_UART_AbortReceive(_huart);
		HAL_UART_Receive_DMA(_huart, _buff.data(), GYRO_BUFF_SIZE);
		_scheduler.Set();
	}
	/**
	 * @brief 非初期化関数
	 */
	virtual void Deinit() override final{
		if(!_is_init)return;
		_is_init = false;
		HAL_UART_AbortReceive(_huart);
		callback::UART_RxHalfComplete.EraseExclusiveCallback(_rx_it);
		_scheduler.Erase();
	}
	/**
	 * @brief リセット
	 * @details モージュールリセットするのでマシンを静止させてください
	 */
	virtual void Reset() override final{
		Timeout();
		_scheduler.Reset();
		HAL_UART_Transmit_IT(_huart, const_cast<uint8_t*>(RESET_COMMAND.data()), RESET_COMMAND.size());
		_init_flag = false;
	}

	/**
	 * @brief 受信コールバック関数のリセット
	 * @param[in] func コールバック関数
	 */
	virtual void ResetReceiveCallback(std::function<void(void)> func) final{
		_receive_callback = func;
	}
};
}
