/**
 * @file CAN.hpp
 * @brief CANのコールバックハンドラ類まとめ
 */
#pragma once
#ifdef HAL_CAN_MODULE_ENABLED

#include "HALCallbacks.hpp"

namespace nut{
/* callback class*/
namespace callback{
inline HALCallback<CAN_HandleTypeDef*> CAN_TxMailboxComplete;//<! メールボックス送信完了割込み
inline HALCallback<CAN_HandleTypeDef*> CAN_TxMailboxAbort;//<! メールボックス送信中止割込み
inline HALCallback<CAN_HandleTypeDef*> CAN_RxFifo0MsgPending;//<! FIFO0着信割込み
inline HALCallback<CAN_HandleTypeDef*> CAN_RxFifo1MsgPending;//<! FIFO1着信割込み
inline HALCallback<CAN_HandleTypeDef*> CAN_RxFifo0Full;//<! FIFO0メールボックス充填割込み
inline HALCallback<CAN_HandleTypeDef*> CAN_RxFifo1Full;//<! FIFO1メールボックス充填割込み
}
}
#endif
