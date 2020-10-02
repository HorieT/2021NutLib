/**
 * @file PID.hpp
 * @brief PID制御器
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Global.hpp"
#include <type_traits>

namespace nut{
/**
 * @brief PID制御器クラス
 * @tparam T ゲイン及び入出力値の型
 * @attention Tが数値型でなければアサートを吐きます
 */
template<typename T>
class PID{
	static_assert(std::is_floating_point_v<T>, "Type is not floating point.");
private:
	T _I_stack;//!< I操作量スタック
	std::array<T, 2> _deviation = {0.0f, 0.0f};//!< 偏差

protected:
	T _P_gain;//!< Pゲイン
	T _I_gain;//!< Iゲイン
	T _D_gain;//!< Dゲイン


	T _I_limit = INFINITY ;//!< I上限値(絶対値)
	T _limit;//!< 操作量上限値(絶対値)

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] P_gain Pゲイン
	 * @param[in] I_gain Iゲイン
	 * @param[in] D_gain Dゲイン
	 * @param[in] limit 操作量上限値(絶対値)
	 */
	PID(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0, T limit = static_cast<T>(0))
		: _P_gain(P_gain), _I_gain(I_gain), _D_gain(D_gain),_limit(limit){}

	/**
	 * コピーコンストラクタ削除
	 */
	PID(const PID<T>& pid) = delete;
	/**
	 * コピーコンストラクタ削除
	 */
	PID& operator=(const PID<T>& pid) = delete;

	/**
	 * @brief デストラクタ
	 */
	virtual ~ PID(){}


	/**
	 * @brief フィードバック計算
	 * @param[in] deviation 偏差
	 * @param[in] ms 前回計算時からの経過時間
	 * @return 操作量
	 */
	const T Calculate(T deviation, uint32_t ms = 1){
		_deviation[1] = _deviation[0];
		_deviation[0] = deviation;
		T P_value = _P_gain * _deviation[0];
		_I_stack += _I_gain * static_cast<T>(ms) * (_deviation[0] + _deviation[1]) * 0.5 * 0.001;//sに直す
		T D_value = _D_gain / static_cast<T>(ms) * (_deviation[0] - _deviation[1]) * 1000.0;//sに直す
		if(!std::isinf(_I_limit)){
			if(_I_stack > _I_limit)_I_stack = _I_limit;
			else if(_I_stack < -_I_limit)_I_stack = -_I_limit;
		}
		T value = P_value + _I_stack + D_value;

		if((_limit != static_cast<T>(0)) && (std::abs(value) > _limit))
			value = (value > 0) ? _limit : -_limit;
		return value;
	}


	/**
	 * @brief 操作量リセット
	 */
	inline void Reset() {_I_stack = static_cast<T>(0);}



	/**
	 * @brief ゲインの設定
	 * @param[in] P_gain Pゲイン
	 * @param[in] I_gain Iゲイン
	 * @param[in] D_gain Dゲイン
	 * @attention 操作量はリセットされます
	 */
	inline void SetGaine(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0){
		Reset();

		_P_gain = P_gain;
		_I_gain = I_gain;
		_D_gain = D_gain;
	}
	/**
	 * @brief 操作量上限の設定
	 * @param[in] limit 操作量上限値
	 * @attention 操作量はリセットされます
	 */
	inline void SetLimitI(T I_limit){
		Reset();

		_I_limit = std::abs(I_limit);
	}

	/**
	 * @brief 操作量上限の設定
	 * @param[in] limit 操作量上限値
	 * @attention 操作量はリセットされます
	 */
	inline void SetLimit(T limit){
		Reset();

		_limit = std::abs(limit);
	}
};
}
