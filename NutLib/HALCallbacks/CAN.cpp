/*
 * @brief 割込み実装部
 */
#include "../Global.hpp"
#include "CAN.hpp"

#if defined(USE_NUTLIB_CALLBACKS) && !defined(UNUSE_NUTLIB_CAN_CALLBACKS) && defined(HAL_CAN_MODULE_ENABLED)
/* HAL callback functions */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_TxMailboxComplete.ReadCallbacks(hcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_TxMailboxComplete.ReadCallbacks(hcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_TxMailboxComplete.ReadCallbacks(hcan);
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_TxMailboxAbort.ReadCallbacks(hcan);
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_TxMailboxAbort.ReadCallbacks(hcan);
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_TxMailboxAbort.ReadCallbacks(hcan);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_RxFifo0MsgPending.ReadCallbacks(hcan);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_RxFifo1MsgPending.ReadCallbacks(hcan);
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_RxFifo0Full.ReadCallbacks(hcan);
}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan){
	nut::callback::CAN_RxFifo1Full.ReadCallbacks(hcan);
}
#endif
