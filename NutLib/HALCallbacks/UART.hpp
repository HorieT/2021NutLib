/*
 * @file CAN.hpp
 * @brief CANのコールバックハンドラ類まとめ
 */
#pragma once
#ifdef HAL_UART_MODULE_ENABLED

#include "HALCallbacks.hpp"

namespace nut{
/* callback class*/
namespace callback{
inline HALCallback<UART_HandleTypeDef*> UART_TxComplete;//<! 全文送信完了割込み
inline HALCallback<UART_HandleTypeDef*> UART_TxHalfComplete;//<! 半文送信完了割込み
inline HALCallback<UART_HandleTypeDef*> UART_RxComplete;//<! 全文受信完了割込み
inline HALCallback<UART_HandleTypeDef*> UART_RxHalfComplete;//<! 半文受信完了割込み
inline HALCallback<UART_HandleTypeDef*> UART_Error;//<! エラー割込み
inline HALCallback<UART_HandleTypeDef*> UART_AbortComplete;//<! 中止完了割込み
inline HALCallback<UART_HandleTypeDef*> UART_AbortTransmitComplete;//<! 送信中止完了割込み
inline HALCallback<UART_HandleTypeDef*> UART_AbortReceiveComplete;//<! 受信中止完了割込み
}
}
#endif
