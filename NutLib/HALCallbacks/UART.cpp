/**
 * @brief 割込み実装部
 */
#include "../Global.hpp"
#include "UART.hpp"

#if defined(USE_NUTLIB_CALLBACKS) && !defined(UNUSE_NUTLIB_UART_CALLBACKS) && defined(HAL_UART_MODULE_ENABLED)

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	nut::callback::UART_TxComplete.ReadCallbacks(huart);
}
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){
	nut::callback::UART_TxHalfComplete.ReadCallbacks(huart);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	nut::callback::UART_RxComplete.ReadCallbacks(huart);
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
	nut::callback::UART_RxHalfComplete.ReadCallbacks(huart);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	nut::callback::UART_Error.ReadCallbacks(huart);
}
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart){
	nut::callback::UART_AbortComplete.ReadCallbacks(huart);
}
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart){
	nut::callback::UART_AbortTransmitComplete.ReadCallbacks(huart);
}
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart){
	nut::callback::UART_AbortReceiveComplete.ReadCallbacks(huart);
}
#endif
