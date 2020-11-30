/**
 * @file PIDBase.hpp
 * @brief PID制御器
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "../Controller.hpp"
#include <type_traits>

namespace nut{
/**
 * @brief PID制御器クラス
 * @tparam T ゲイン及び入出力値の型
 * @attention Tが数値型でなければアサートを吐きます
 */
template<typename T>
class PIDBase : public Controller<T, 1, 1>{
	static_assert(std::is_floating_point_v<T>, "Type is not floating point.");

protected:
	std::array<T, 3> _deviation = {0.0, 0.0, 0.0};

	T _P_gain;//!< Pゲイン
	T _I_gain;//!< Iゲイン
	T _D_gain;//!< Dゲイン

public:
	/**
	 * @brief コンストラクタ
	 * @param[in] P_gain Pゲイン
	 * @param[in] I_gain Iゲイン
	 * @param[in] D_gain Dゲイン
	 */
	PIDBase(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0)
		:Controller<T, 1, 1>(), _P_gain(P_gain), _I_gain(I_gain), _D_gain(D_gain){}

	/**
	 * コピーコンストラクタ削除
	 */
	PIDBase(const PIDBase<T>& pid) = delete;
	/**
	 * コピーコンストラクタ削除
	 */
	PIDBase& operator=(const PIDBase<T>& pid) = delete;

	/**
	 * @brief デストラクタ
	 */
	virtual ~PIDBase(){}


	/**
	 * @brief フィードバック計算
	 * @param[in] input 入力
	 * @param[in] ms 前回計算時からの経過時間
	 * @return 操作量
	 */
	virtual T Calculate(T input, uint32_t ms)override = 0;

	/**
	 * @brief 操作量リセット
	 */
	virtual void Reset() override{_deviation = {0.0};}



	/**
	 * @brief ゲインの設定
	 * @param[in] P_gain Pゲイン
	 * @param[in] I_gain Iゲイン
	 * @param[in] D_gain Dゲイン
	 * @attention 操作量はリセットされます
	 */
	inline virtual void SetGaine(T P_gain = 0.0, T I_gain = 0.0, T D_gain = 0.0) final{
		Reset();

		_P_gain = P_gain;
		_I_gain = I_gain;
		_D_gain = D_gain;
	}


	const T& P() const& {return _P_gain;}
	const T& I() const& {return _I_gain;}
	const T& D() const& {return _D_gain;}
	T P() const&& {return _P_gain;}
	T I() const&& {return _I_gain;}
	T D() const&& {return _D_gain;}
	T& P() & {
		Reset();
		return _P_gain;
	}
	T& I() & {
		Reset();
		return _I_gain;
	}
	T& D() & {
		Reset();
		return _D_gain;
	}
};
}
