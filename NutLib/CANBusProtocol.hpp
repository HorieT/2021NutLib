/**
 * @brief CANバスの通信仕様
 * @details 学ロボ2021での仕様、esa参照
 */
#pragma once

#include "Global.hpp"

namespace nut{

namespace can_protocol{
static constexpr uint16_t DEVICE_MSK = 0x700;
static constexpr uint16_t DEVICE_SHIFT = 8;
static constexpr uint16_t DEVICE_NUM_MSK = 0x0F0;
static constexpr uint16_t DEVICE_NUM_SHIFT = 4;
static constexpr uint16_t BOAD_MSK = DEVICE_MSK | DEVICE_NUM_MSK;
static constexpr uint16_t DATA_MSK = 0x00F;


enum class Device : uint8_t{
	allNode = 0x0U,
	powerSupplay,
	microcomputer,
	PC,
	motorDriver,
	solenoidValve
};


/* uniq board */

namespace power{
enum class DataType : uint8_t{
	specialOperation = 0x00,
	paramsInput,
	controlInput
};
}


namespace micom{

}

/* pc */
namespace pc{

}

/* motorDriver */
namespace motor{
/* data type */
enum class DataType : uint8_t{
	specialOperation = 0x00,
	paramsInput,
	controlInput
};
/* special operation */
enum class SpecialOperation :uint8_t{
	writeFlash = 0x01,
	emergency = 0x80
};
/* paramas */
enum class ParamsInput : uint8_t{
	velocityP = 0x01,
	velocityI = 0x02,
	velocityD = 0x03,
	velocityLimit = 0x04,
	radianP = 0x11,
	radianI = 0x12,
	radianD = 0x13,
	radianLimit = 0x14,
	currentP = 0x21,
	currentI = 0x22,
	currentD = 0x23,
	currentLimit = 0x24,
	encoderResolusion = 0x30,
	encoderMode = 0x3F,
};
/* encoder mode */
enum class EncoderMode :uint8_t{
	Inc = 0x00,
	Abs
};
/* control */
enum class ControlInput{
	duty = 0x00,
	rps = 0x01,
	rad = 0x02,
	current = 0x03
};
}

/* solenoidValve */
namespace sv{

}


/* functions */
static constexpr uint16_t GetDeviceMsk(Device device){
	return static_cast<uint16_t>(device) << DEVICE_SHIFT;
}
static constexpr uint16_t GetBoardID(Device device, uint8_t num){
	return (static_cast<uint16_t>(device) << DEVICE_SHIFT) | (num << DEVICE_NUM_SHIFT);
}
}
}
