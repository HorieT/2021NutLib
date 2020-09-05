/*
 * R1370P,R1350N縺ｮ繧ｯ繝ｩ繧ｹ�ｼ�70P縺ｮ譁ｹ縺瑚ｨｭ螳壹〒縺阪ｋ鬆�逶ｮ縺悟ｰ代↑縺�縲ゅせ繝壹ャ繧ｯ縺ｯ螟壼�蜷後§�ｼ�
 * 險ｭ螳夐�∽ｿ｡縺ｫ髢｢縺励※縺ｯ縺ｾ縺�譖ｸ縺�縺ｦ縺�縺ｪ縺�
 * 蝓ｺ蠎輔↓IMU繧ｯ繝ｩ繧ｹ
 * 菴ｿ逕ｨUART縺ｯ115200bps縺ｧCircular縺ｮDMA縺ｫ縺吶ｋ縺薙→
 */

#include "IMU.hpp"
#include "../../TimeScheduler.hpp"
#include <cstring>

namespace nut{
class R13x0 : public IMU{
private:
	static constexpr uint8_t GYRO_DATA_SIZE = 15;
	static constexpr uint8_t GYRO_BUFF_SIZE = GYRO_DATA_SIZE * 2;
	static constexpr uint8_t GYRO_BUFF_SIZE_D = GYRO_BUFF_SIZE - 1;
	static constexpr uint32_t TIMEOUT_TIME = 25;

	TimeScheduler<void> _scheduler;
	std::array<uint8_t, GYRO_BUFF_SIZE> _buff;
	UART_HandleTypeDef* const _huart;

	void Timeout(){
		_sensor_acc = {0.0f, 0.0f, 0.0f};
		_sensor_rot = {0.0f, 0.0f, 0.0f};
		_global_acc = {0.0f, 0.0f, 0.0f};
		_global_rot = {0.0f, 0.0f, 0.0f};
		_global_angle = {0.0f, 0.0f, 0.0f};
	}
public:
	R13x0(UART_HandleTypeDef* huart) : _scheduler([this]{Timeout();}, TIMEOUT_TIME), _huart(huart){}
	virtual ~R13x0(){_scheduler.Erase();}

	virtual void Init() override final{
		HAL_UART_Receive_DMA(_huart, _buff.data(), GYRO_BUFF_SIZE);
		_scheduler.Set();
	}
	virtual void Reset() override final{
		Timeout();
		_scheduler.Reset();
	}


	/*
	 * 蜿嶺ｿ｡髢｢謨ｰﾂ�
	 *  HAL_UART_RxHalfCpltCallback()蜀�縺ｧ蜻ｼ縺ｳ蜃ｺ縺吶％縺ｨ
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
						_global_angle.z() = - static_cast<float>(angle) * static_cast<float>(M_PI) / 18000.0f;
						_global_rot.z()=  - static_cast<float>(angle_vel) * static_cast<float>(M_PI) / 18000.0f;

						memcpy(&acc, &read_data[5], 2);
						_sensor_acc.x() = static_cast<float>(acc);
						memcpy(&acc, &read_data[7], 2);
						_sensor_acc.y() = static_cast<float>(acc);
						memcpy(&acc, &read_data[9], 2);
						_sensor_acc.z() = static_cast<float>(acc);
						return true;
					}
				}
			}
		}
		return false;
	}
};
}
