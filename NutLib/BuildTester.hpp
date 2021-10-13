/**
 * ライブラリに大きな更新を加える際,各依存関係オブジェクトに問題をきたさないか確認するためだけのファイル
 */
#pragma once

#include "AllLib.hpp"

namespace nut::BuildTestObject{
	/* chassis */
	inline OmniChassis<3>* _omni;
	inline SteerChassis<3>* _steer;
	inline SteerChassisSp<3>* _steer_sp;

	/*  */
}
