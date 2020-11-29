#include "../Global.hpp"
#include "../TimeScheduler.hpp"


/* NUT callback */
#ifdef USE_NUTLIB_CALLBACKS

/* HAL's global object *//*
extern __IO uint32_t uwTick;//<! HALのデフォルト変数
extern HAL_TickFreqTypeDef uwTickFreq;//<! HALのデフォルト変数
*/
/*
 * @brief HALのデフォルトコールバック関数
 */
void HAL_IncTick(void){
	uwTick += uwTickFreq;

	nut::TimeSchedulerBase::TimeCheck();
	nut::callback::Systick.ReadCallbacks();
}
#endif
