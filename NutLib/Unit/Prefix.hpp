/**
 * @file Unit.hpp
 * @brief 単位系接頭辞
 * @details conceptが使えない&ratioだとサイズが大きすぎることの回避<br>
 * c++20ならおとなしくconceptを使おう！
 * @author Horie
 * @date 2020/9
 */
#pragma once

#include "../Global.hpp"
#include <numeric>

namespace nut{
namespace unit{
/**
 * @brief　接頭辞基底クラス
 */
class PrefixBase{
protected:
	/**
	 * @brief　隠蔽コンストラクタ
	 */
	constexpr PrefixBase(){}
	/**
	 * @brief　デストラクタ
	 * @param[in] ratio 比
	 */
	virtual ~PrefixBase(){}
};


/**
 * @brief　接頭辞クラス
 * @tparam N 分子
 * @tparam D 分母
 */
template<uint32_t N, uint32_t D>
class Prefix : public PrefixBase{
public:
	static constexpr uint32_t num = N / std::gcd(static_cast<int64_t>(N), static_cast<int64_t>(D));//!< 分子 std::ratioとの互換
	static constexpr uint32_t den = D / std::gcd(static_cast<int64_t>(N), static_cast<int64_t>(D));//!< 分母 std::ratioとの互換
	static constexpr float RATIO = num / den;//<! 比
	using type = Prefix<num, den>;
};


using micro = Prefix<1, 1000000>;//<! マイクロ
using milli = Prefix<1, 1000>;//<! ミリ
using centi = Prefix<1, 100>;//<! センチ
using kilo = Prefix<1000, 1>;//<! キロ
}
}
