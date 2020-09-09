/**
 * @file PID.hpp
 * @brief PID制御器
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "Global.hpp"

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
	T _value;//!< 操作量
	std::array<T, 3> _deviation = {0.0f, 0.0f, 0.0f};//!< 偏差

protected:
	T _P_gain;//!< Pゲイン
	T _I_gain;//!< Iゲイン
	T _D_gain;//!< Dゲイン

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
		: _value(static_cast<T>(0)), _P_gain(P_gain), _I_gain(I_gain), _D_gain(D_gain),_limit(limit){}

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
	const T& Calculate(T deviation, uint32_t ms = 1){
		_deviation[2] = _deviation[1];
		_deviation[1] = _deviation[0];
		_deviation[0] = deviation;
		_value +=
				_P_gain * (_deviation[0] - _deviation[1]) +
				_I_gain * static_cast<T>(ms) * (_deviation[0]) +
				_D_gain / static_cast<T>(ms) * (_deviation[0] - 2.0 * _deviation[1] + _deviation[2]);

		if((_limit != static_cast<T>(0)) && (std::abs(_value) > _limit))
			_value = (_value > 0) ? _limit : -_limit;
		return _value;
	}


	/**
	 * @brief ゲインの設定
	 * @param[in] P_gain Pゲイン
	 * @param[in] I_gain Iゲイン
	 * @param[in] D_gain Dゲイン
	 * @attention 操作量はリセットされます
	 */
	inline void SetGaine(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0){
		_value = static_cast<T>(0);

		_P_gain = P_gain;
		_I_gain = I_gain;
		_D_gain = D_gain;
	}
	/**
	 * @brief 操作量上限の設定
	 * @param[in] limit 操作量上限値
	 * @attention 操作量はリセットされます
	 */
	inline void SetLimit(T limit){
		_value = static_cast<T>(0);

		_limit = limit;
	}


	/**
	 * @brief 操作量の取得
	 * @return 操作量
	 */
	inline const T& GetValue() const{return _value;}
	/**
	 * @brief 操作量の直接操作
	 * @param[in] value 操作量
	 */
	inline void SetValue(T value){_value = value;}
};
}
