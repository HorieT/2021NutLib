/**
 * @brief CANバスの通信仕様
 * @details 学ロボ2021での仕様、esa参照
 */
#pragma once

#include "Global.hpp"
#include <type_traits>

namespace nut{

namespace can_protocol{
static constexpr uint16_t DEVICE_TYPE_MSK = 0x700;
static constexpr uint16_t DEVICE_TYPE_SHIFT = 8;
static constexpr uint16_t DEVICE_NUM_MSK = 0x0F0;
static constexpr uint16_t DEVICE_NUM_SHIFT = 4;
static constexpr uint16_t DEVICE_ID_MSK = DEVICE_TYPE_MSK | DEVICE_NUM_MSK;
static constexpr uint16_t DATA_MSK = 0x00F;


enum class Device : uint8_t{
	allNode = 0x0U,
	powerSupplay,
	microcomputer,
	PC,
	motorDriver,
	solenoidValve
};


/* all node */
namespace all{
enum class NotificationType : uint8_t{
	power = 0x0,
	error = 0x1,
	general = 0x02
};
enum class Power : uint8_t{
	emergencyStop = 0x0,
	currentOver = 0x1,
	fuseDown = 0x2,
	userStop = 0x3,
	irregularFetOff = 0x4,
	emergencyRelease = 0x8,
	fetOn = 0xC,
};
enum class Error : uint8_t{
};
enum class General : uint8_t{
	init = 0x0,
};
}



/* uniq board */

namespace power{
enum class DataType : uint8_t{
	fetState = 0x0,
	currentLimit = 0x1,
	FuseState = 0x2
};
}


namespace micom{
enum class DataType : uint8_t{
	controlMode = 0x0,
};

enum class ControlMode : uint8_t{
	init = 0x00,
	debug = 0x01,
	sequence = 0x02,
	emergency = 0x80
};
}

/* pc */
namespace pc{
enum class DataType : uint8_t{
	error = 0x0,
	xPos = 0x1,
	yPos = 0x2,
	thetaPos = 0x3,
	controller = 0x8
};
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
	singleStart = 0x01,
	//steerStart = 0x02,
	stop = 0x80,
	//writeFlash = 0xFF,
};
/* paramas */
enum class ParamsInput : uint8_t{
	velocityP = 0x01,
	velocityI = 0x02,
	velocityD = 0x03,
	velocityLimit = 0x04,
	velocityILimit = 0x05,
	radianP = 0x11,
	radianI = 0x12,
	radianD = 0x13,
	radianLimit = 0x14,
	radianILimit = 0x15,
	currentP = 0x21,
	currentI = 0x22,
	currentD = 0x23,
	currentLimit = 0x24,
	currentILimit = 0x24,
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
	radps = 0x01,
	rad = 0x02,
	//current = 0x03
};
}

/* solenoidValve */
namespace sv{

}


/* functions */
static constexpr uint8_t MakeDeviceType(Device device){
	return static_cast<uint8_t>(device) << (DEVICE_TYPE_SHIFT - DEVICE_NUM_SHIFT);
}
static constexpr uint8_t MakeDeviceID(Device device, uint8_t num){
	return (static_cast<uint8_t>(device) << (DEVICE_TYPE_SHIFT - DEVICE_NUM_SHIFT)) | num ;
}
template<typename T>
static constexpr auto MakeCANID(uint8_t device_id, T data_type)
	-> typename std::enable_if_t<std::is_enum_v<T>, uint16_t>{
	return (static_cast<uint16_t>(device_id) << DEVICE_NUM_SHIFT) | static_cast<uint8_t>(data_type);
}
static constexpr uint16_t MakeCANID(uint8_t device_id, uint8_t data_type){
	return (static_cast<uint16_t>(device_id) << DEVICE_NUM_SHIFT) | data_type;
}
template<typename T>
static constexpr auto MakeCANID(Device device, uint8_t num, T data_type)
-> typename std::enable_if_t<std::is_enum_v<T>, uint16_t>{
	return (static_cast<uint16_t>(device) << DEVICE_TYPE_SHIFT) | (num << DEVICE_NUM_SHIFT) | static_cast<uint8_t>(data_type);
}
static constexpr uint16_t MakeCANID(Device device, uint8_t num, uint8_t data_type){
	return (static_cast<uint16_t>(device) << DEVICE_TYPE_SHIFT) | (num << DEVICE_NUM_SHIFT) | data_type;
}
static constexpr Device GetDeviceType(uint16_t id){
	return static_cast<Device>((id & DEVICE_TYPE_MSK) >> DEVICE_TYPE_SHIFT);
}
static constexpr uint8_t GetDeviceID(uint16_t id){
	return static_cast<uint8_t>((id & DEVICE_ID_MSK) >> DEVICE_NUM_SHIFT);
}
static constexpr uint8_t GetDataType(uint16_t id){
	return static_cast<uint8_t>(id & DATA_MSK);
}
}
}
