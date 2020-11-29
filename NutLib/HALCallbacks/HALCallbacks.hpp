/**
 * @file HALCallbacks.hpp
 * @brief コールバック関数ラッパクラス
 * @author Horie
 * @date 2020/10
 */
#ifdef USE_NUTLIB_CALLBACKS

#pragma once

#include "../Global.hpp"
#include <type_traits>
#include <functional>
#include <memory>
#include <map>

namespace nut{
/**
 * @brief コールバック関数ラッパクラス
 * @tparam Args コールバック関数オブジェクト引数
 * @details copy&move禁止
 */
template<typename... Args>
class HALCallback{
private:
	using Function_type = void(Args...);
	using ExFunction_type = bool(Args...);
	using CallbackList = std::multimap<uint8_t, std::function<Function_type>>;
	using ExCallbackList = std::multimap<uint8_t, std::function<ExFunction_type>>;
protected:
	CallbackList _callbacks;
	ExCallbackList _exclusive_callbacks;

public:
	using CallbackIterator = typename CallbackList::iterator;
	using ExCallbackIterator = typename ExCallbackList::iterator;
	HALCallback(){}
	virtual ~HALCallback(){}

	/*copy&moveコンストラクタ削除 */
	HALCallback(const HALCallback<Args...>& ) = delete;
	HALCallback& operator=(const HALCallback<Args...>& ) = delete;
	HALCallback(HALCallback<Args...>&& ) = delete;
	HALCallback& operator=(HALCallback<Args...>&& ) = delete;

	/**
	 * @brief コールバック関数オブジェクト追加
	 * @param[in] priority 優先度
	 * @param[in] callback コールバック関数オブジェクト
	 * @return 構築されたイテレータ
	 */
	CallbackIterator AddCallback(uint8_t priority, std::function<Function_type> callback){
		return _callbacks.emplace(priority, callback);
	}
	/**
	 * @brief 排他的コールバック関数オブジェクト追加
	 * @details 排他的コールバックは優先順に呼び出され、trueを返した時点で以降の排他的コールバックの呼び出しをせずに終了します。
	 * @param[in] priority 優先度
	 * @param[in] callback 排他的コールバック関数オブジェクト
	 * @return 構築されたイテレータ
	 */
	ExCallbackIterator AddExclusiveCallback(uint8_t priority, std::function<ExFunction_type> callback){
		return _exclusive_callbacks.emplace(priority, callback);
	}
	/**
	 * @brief コールバック関数オブジェクト削除
	 * @param[in] it 削除対象のイテレータ
	 */
	void EraseCallback(CallbackIterator it){
		_callbacks.erase(it);
	}
	/**
	 * @brief コールバック関数オブジェクト削除
	 * @param[in] it 削除対象のイテレータ
	 */
	void EraseExclusiveCallback(ExCallbackIterator it){
		_exclusive_callbacks.erase(it);
	}
	/**
	 * @brief コールバック関数オブジェクト呼び出し
	 * @details 通常のコールバックを一通り呼び出した後に排他的コールバックを呼び出します。
	 */
	void ReadCallbacks(Args ...arg){
		for(auto& func : _callbacks){
			func.second(arg...);
		}
		for(auto& func : _exclusive_callbacks){
			if(func.second(arg...))break;
		}
	}
};


/**
 * @brief コールバック関数ラッパクラスのvoid特殊化
 * @details copy&move禁止
 */
template<>
class HALCallback<void>{
private:
	using CallbackList = std::multimap<uint8_t, std::function<void(void)>>;
	using ExCallbackList = std::multimap<uint8_t, std::function<bool(void)>>;
protected:
	CallbackList _callbacks;
	ExCallbackList _exclusive_callbacks;

public:
	using CallbackIterator = typename CallbackList::iterator;
	using ExCallbackIterator = typename ExCallbackList::iterator;
	HALCallback(){}
	virtual ~HALCallback(){}

	/*copy&moveコンストラクタ削除 */
	HALCallback(const HALCallback<void>& ) = delete;
	HALCallback& operator=(const HALCallback<void>& ) = delete;
	HALCallback(HALCallback<void>&& ) = delete;
	HALCallback& operator=(HALCallback<void>&& ) = delete;

	/**
	 * @brief コールバック関数オブジェクト追加
	 * @param[in] priority 優先度
	 * @param[in] callback コールバック関数オブジェクト
	 * @return 構築されたイテレータ
	 */
	CallbackIterator AddCallback(uint8_t priority, std::function<void(void)> callback){
		return _callbacks.emplace(priority, callback);
	}
	/**
	 * @brief 排他的コールバック関数オブジェクト追加
	 * @details 排他的コールバックは優先順に呼び出され、trueを返した時点で以降の排他的コールバックの呼び出しをせずに終了します。
	 * @param[in] priority 優先度
	 * @param[in] callback 排他的コールバック関数オブジェクト
	 * @return 構築されたイテレータ
	 */
	ExCallbackIterator AddExclusiveCallback(uint8_t priority, std::function<bool(void)> callback){
		return _exclusive_callbacks.emplace(priority, callback);
	}
	/**
	 * @brief コールバック関数オブジェクト削除
	 * @param[in] it 削除対象のイテレータ
	 */
	void EraseCallback(CallbackIterator it){
		_callbacks.erase(it);
	}
	/**
	 * @brief コールバック関数オブジェクト削除
	 * @param[in] it 削除対象のイテレータ
	 */
	void EraseExclusiveCallback(ExCallbackIterator it){
		_exclusive_callbacks.erase(it);
	}

	/**
	 * @brief コールバック関数オブジェクト呼び出し
	 * @details 通常のコールバックを一通り呼び出した後に排他的コールバックを呼び出します。
	 */
	void ReadCallbacks(){
		for(auto& func : _callbacks){
			func.second();
		}
		for(auto& func : _exclusive_callbacks){
			if(func.second())break;
		}
	}
};
}
#endif
