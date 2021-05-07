/**
 * @file OmniChassis.hpp
 * @brief Xboxコントローラ
 * @author Horie
 * @date 2021/3
 */
#pragma once

#include "../Global.hpp"

namespace nut{
/*
 * @brief Xboxコントローラのデータクラス
 */
class XboxController{
public:
	enum class Button : uint32_t{
		A 		= 0x0001,
		B 		= 0x0002,
		X 		= 0x0004,
		Y 		= 0x0008,
		L1	 	= 0x0010,
		R1	 	= 0x0020,
		SELECT 	= 0x0040,
		START 	= 0x0080,
		L3	 	= 0x0100,
		R3 		= 0x0200,
	};
	enum class POV : uint32_t{
		UP 			= 0,
		UP_LEFT 	= 4500,
		LEFT 		= 9000,
		UNDER_LEFT 	= 13500,
		UNDER	 	= 18000,
		UNDER_RIGHT	= 22500,
		RIGHT 		= 27000,
		UP_RIGHT 	= 31500,
		NONE 		= 65535,
	};
private:
	struct ButtonData{
		uint32_t button;
		uint32_t pov;
		int8_t leftX;
		int8_t leftY;
		int8_t rightX;
		int8_t rightY;
	};
public:

};
}
