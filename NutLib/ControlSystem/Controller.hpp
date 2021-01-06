/*
 * @file Controller.hpp
 * @brief 制御系基底
 * @author Horie
 * @date 2020/10
 */
#pragma once

#include "../Global.hpp"
#include "../Unit/UnitCore.hpp"
#include <array>
#include <limits>

namespace nut{

/**
 * @brief 制御系の基底クラス
 * @tparam T 制御系計算型(floating point)
 * @tparam Input 制御系入力数
 * @tparam Output 制御系出力数
 */
template<typename T, uint8_t Input, uint8_t Output>
class Controller{
	static_assert(std::is_floating_point_v<T>, "Type is not floating point.");
protected:
	T _limit = std::numeric_limits<T>::infinity();//!< 操作量上限値(絶対値):
public:
	Controller(){}
	virtual ~Controller(){}

	/**
	 * @brief 制御器演算
	 * @param[in] input 入力値
	 * @param[in] ms 制御周期時間[ms]
	 * @return 出力値
	 */
	virtual std::array<T, Output> Calculate(std::array<T, Input> input, nut::MilliSecond<float> ms) = 0;
	virtual void Reset() = 0;
	/**
	 * @brief 操作量上限の設定
	 * @param[in] limit 操作量上限値
	 * @attention 操作量はリセットされます
	 */
	void SetLimit(T limit){
		Reset();
		_limit = std::abs(limit);
	}
};


/**
 * @brief 制御系の基底クラス(1入力特殊化)
 * @tparam T 制御系計算型(floating point)
 * @tparam Output 制御系出力数
 */
template<typename T, uint8_t Output>
class Controller<T, 1, Output>{
	static_assert(std::is_floating_point_v<T>, "Type is not floating point.");
protected:
	T _limit = std::numeric_limits<T>::infinity();//!< 操作量上限値(絶対値):
public:
	Controller(){}
	virtual ~Controller(){}

	/**
	 * @brief 制御器演算
	 * @param[in] input 入力値
	 * @param[in] ms 制御周期時間[ms]
	 * @return 出力値
	 */
	virtual std::array<T, Output> Calculate(T input, nut::MilliSecond<float> ms) = 0;
	virtual void Reset() = 0;
	/**
	 * @brief 操作量上限の設定
	 * @param[in] limit 操作量上限値
	 * @attention 操作量はリセットされます
	 */
	void SetLimit(T limit){
		Reset();
		_limit = std::abs(limit);
	}
	/**
	 * @brief 操作量上限の取得
	 * @return 操作量上限値
	 */
	T GetLimit(){
		return _limit;
	}
};


/**
 * @brief 制御系の基底クラス(1出力特殊化)
 * @tparam T 制御系計算型(floating point)
 * @tparam Input 制御系入力数
 */
template<typename T, uint8_t Input>
class Controller<T, Input, 1>{
	static_assert(std::is_floating_point_v<T>, "Type is not floating point.");
protected:
	T _limit = std::numeric_limits<T>::infinity();//!< 操作量上限値(絶対値):
public:
	Controller(){}
	virtual ~Controller(){}

	/**
	 * @brief 制御器演算
	 * @param[in] input 入力値
	 * @param[in] ms 制御周期時間[ms]
	 * @return 出力値
	 */
	virtual T Calculate(std::array<T, Input> input, nut::MilliSecond<float> ms) = 0;
	virtual void Reset() = 0;
	/**
	 * @brief 操作量上限の設定
	 * @param[in] limit 操作量上限値
	 * @attention 操作量はリセットされます
	 */
	void SetLimit(T limit){
		Reset();
		_limit = std::abs(limit);
	}
	/**
	 * @brief 操作量上限の取得
	 * @return 操作量上限値
	 */
	T GetLimit(){
		return _limit;
	}
};


/**
 * @brief 制御系の基底クラス(1入力1出力特殊化)
 * @tparam T 制御系計算型(floating point)
 */
template<typename T>
class Controller<T, 1, 1>{
	static_assert(std::is_floating_point_v<T>, "Type is not floating point.");
protected:
	T _limit = std::numeric_limits<T>::infinity();//!< 操作量上限値(絶対値):
public:
	Controller(){}
	virtual ~Controller(){}

	/**
	 * @brief 制御器演算
	 * @param[in] input 入力値
	 * @param[in] ms 制御周期時間[ms]
	 * @return 出力値
	 */
	virtual T Calculate(T input, nut::MilliSecond<float> ms) = 0;
	virtual void Reset() = 0;
	/**
	 * @brief 操作量上限の設定
	 * @param[in] limit 操作量上限値
	 * @attention 操作量はリセットされます
	 */
	void SetLimit(T limit){
		Reset();
		_limit = std::abs(limit);
	}
	/**
	 * @brief 操作量上限の取得
	 * @return 操作量上限値
	 */
	T GetLimit(){
		return _limit;
	}
};

}
