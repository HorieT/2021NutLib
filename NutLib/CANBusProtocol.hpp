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

//<! @brief デバイス型
enum class Device : uint8_t{
	allNode = 0x0U,
	powerSupplay,
	microcomputer,
	PC,
	motorDriver,
	solenoidValve,
	tof,
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
	wakeUp= 0x0,
	uninit
};
}



/* unique board */

/* power board */
namespace power{
enum class DataType : uint8_t{
	fetState = 0x0,
};
enum class RemoteType : uint8_t{
	fetState 	= 0x0,
	current 	= 0x1,
	fuseState 	= 0x2,
};
}


/* micon */
namespace micom{
enum class DataType : uint8_t{
	controlMode = 0x0,
	xSpead,
	ySpead,
	thetaSpead,
	steering,
};
enum class ControlMode : uint8_t{
	init 		= 0x00,
	debug 		= 0x01,
	sequence 	= 0x02,
	emergency 	= 0x80
};
enum class Steering : uint8_t{
	origin 	= 0x00,
	lock 	= 0x01,
};
enum class DataTypeTof : uint8_t{
	length = 0x5,
};
}

/* pc */
namespace pc{
enum class DataType : uint8_t{
	error 		= 0x0,
	odmeter 	= 0x1,
	speed 		= 0x2,
	acc 		= 0x3,
	uniqe 	= 0x8
};
}


/* motorDriver */
namespace motor{
/* data type */
enum class DataType : uint8_t{
	specialOperation = 0x00,
	paramsInput,
	controlInput,
	steerControlInput
};
/* special operation */
enum class SpecialOperation :uint8_t{
	singleStartInc 			= 0x00,
	singleStartIncCurrent	= 0x01,
	singleStartAbs 			= 0x02,
	singleStartAbsCurrent	= 0x03,
	steerStart 				= 0x04,
	steerStartCurrent 		= 0x05,
	singleStartIncDebug 		= 0x10,
	singleStartIncCurrentDebug 	= 0x11,
	singleStartAbsDebug 		= 0x12,
	singleStartAbsCurrentDebug	= 0x13,
	steerStartDebug 			= 0x14,
	steerStartDebugCurrent		= 0x15,
	stop 					= 0x80,
	dedicatedFirmware 		= 0x90,
	//writeFlash = 0xFF,
};
/* paramas */
enum class ParamsInput : uint8_t{
	velocityP 			= 0x01,
	velocityI 			= 0x02,
	velocityD 			= 0x03,
	velocityLimit 		= 0x04,
	velocityILimit 		= 0x05,
	radianP 			= 0x06,
	radianI 			= 0x07,
	radianD 			= 0x08,
	radianLimit 		= 0x09,
	radianILimit 		= 0x0A,
	currentP 			= 0x0B,
	currentI 			= 0x0C,
	currentD 			= 0x0D,
	currentLimit 		= 0x0E,
	currentILimit 		= 0x0F,
	encoderResolusion 	= 0x10,
	overCurrentLimit 	= 0x11,
};
/* control */
enum class ControlInput{
	duty 						= 0x00,
	radps 						= 0x01,
	radMulti					= 0x02,
	radSingle					= 0x03,
	radSinglePolarity			= 0x04,
	current						= 0x80,
};
}


/* solenoidValve */
namespace sv{
enum class DataType : uint8_t{
	specialOperation = 0x00,
	valveState,
	valveSetAnd,
	valveSetOr,
};
/*special operation*/
enum class SpecialOperation :uint8_t{
};
}


/* tof */
namespace tof{
enum class DataType : uint8_t{
	specialOperation = 0x00,
};
/*special operation*/
enum class SpecialOperation :uint8_t{
	regularModeStart = 0x01,
	regularModeStop = 0x08
};
}





/* functions */
/*
 * @brief デバイスID変換
 * @param[in] device デバイス型
 * @return デバイスID
 */
static constexpr uint8_t MakeDeviceID(Device device){
	return static_cast<uint8_t>(device) << (DEVICE_TYPE_SHIFT - DEVICE_NUM_SHIFT);
}
/*
 * @brief デバイスID変換
 * @param[in] device デバイス型
 * @param[in] num デバイスナンバー
 * @return デバイスID
 */
static constexpr uint8_t MakeDeviceID(Device device, uint8_t num){
	return (static_cast<uint8_t>(device) << (DEVICE_TYPE_SHIFT - DEVICE_NUM_SHIFT)) | (num & 0x0F);
}
/*
 * @brief CAN ID変換
 * @param[in] device_id デバイスID
 * @param[in] data_type データ型
 * @return CAN ID
 */
template<typename T>
static constexpr auto MakeCANID(uint8_t device_id, T data_type)
	-> typename std::enable_if_t<std::is_enum_v<T>, uint16_t>{
	return (static_cast<uint16_t>(device_id) << DEVICE_NUM_SHIFT) | static_cast<uint8_t>(data_type);
}
/*
 * @brief CAN ID変換
 * @param[in] device_id デバイスID
 * @param[in] data_type データ型
 * @return CAN ID
 */
static constexpr uint16_t MakeCANID(uint8_t device_id, uint8_t data_type){
	return (static_cast<uint16_t>(device_id) << DEVICE_NUM_SHIFT) | data_type;
}
/*
 * @brief CAN ID変換
 * @param[in] device デバイス型
 * @param[in] num デバイスナンバー
 * @param[in] data_type データ型
 * @return CAN ID
 */
template<typename T>
static constexpr auto MakeCANID(Device device, uint8_t num, T data_type)
-> typename std::enable_if_t<std::is_enum_v<T>, uint16_t>{
	return (static_cast<uint16_t>(device) << DEVICE_TYPE_SHIFT) | ((num & 0x0F) << DEVICE_NUM_SHIFT) | static_cast<uint8_t>(data_type);
}
/*
 * @brief CAN ID変換
 * @param[in] device デバイス型
 * @param[in] num デバイスナンバー
 * @param[in] data_type データ型
 * @return CAN ID
 */
static constexpr uint16_t MakeCANID(Device device, uint8_t num, uint8_t data_type){
	return (static_cast<uint16_t>(device) << DEVICE_TYPE_SHIFT) | ((num & 0x0F) << DEVICE_NUM_SHIFT) | data_type;
}
/*
 * @brief CAN ID変換(全体通知)
 * @param[in] device デバイス型
 * @param[in] data_type 通知型
 * @param[in] data 通知データ
 * @return CAN ID
 */
template<typename T>
static constexpr auto MakeCANID(Device device, can_protocol::all::NotificationType data_type, T data)
-> typename std::enable_if_t<std::is_enum_v<T>, uint16_t>{
	return (static_cast<uint16_t>(device) << DEVICE_TYPE_SHIFT) | (static_cast<uint8_t>(data_type) << DEVICE_NUM_SHIFT) | static_cast<uint8_t>(data);
}
/*
 * @brief デバイス型取得
 * @param[in] can_id CAN ID
 * @return デバイス型
 */
static constexpr Device GetDeviceType(uint16_t can_id){
	return static_cast<Device>((can_id & DEVICE_TYPE_MSK) >> DEVICE_TYPE_SHIFT);
}
/*
 * @brief デバイスID取得
 * @param[in] can_id CAN ID
 * @return デバイスID
 */
static constexpr uint8_t GetDeviceID(uint16_t can_id){
	return static_cast<uint8_t>((can_id & DEVICE_ID_MSK) >> DEVICE_NUM_SHIFT);
}
/*
 * @brief データ型取得
 * @param[in] can_id CAN ID
 * @return データ型
 */
static constexpr uint8_t GetDataType(uint16_t can_id){
	return static_cast<uint8_t>(can_id & DATA_MSK);
}
}
}
