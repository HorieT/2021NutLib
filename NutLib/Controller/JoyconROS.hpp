/**
 * @file OmniChassis.hpp
 * @brief ROSトピックのjoyconと同一のデータ定義
 * @author Horie
 * @date 2021/3
 */
#pragma once

#include "../Global.hpp"

namespace nut{
/*
 * @brief ROSトピックのjoycon定義のコピークラス
 * @details
 * XboxController使おうと思ったけど、マイコン側でエンコードするのだるいから
 * ROSのトピックと同じ形の奴作った
 */
class JoyconROS{
public:
	enum class Button : uint32_t{
		SELECT	= 0x0001,
		L3 		= 0x0002,
		R3 		= 0x0004,
		START	= 0x0008,
		UP	 	= 0x0010,
		RIGHT 	= 0x0020,
		DOWN 	= 0x0040,
		LEFT 	= 0x0080,
		L2	 	= 0x0100,
		R2 		= 0x0200,
		L1	 	= 0x0400,
		R1 		= 0x0800,
		Y 		= 0x1000,
		B	 	= 0x2000,
		A	 	= 0x4000,
		X 		= 0x8000,
		HOME	= 0x10000,
	};
	struct ButtonData{
		uint32_t button;
		int8_t leftX;
		int8_t leftY;
		int8_t rightX;
		int8_t rightY;
	};
private:

};
}
