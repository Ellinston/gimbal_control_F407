#ifndef IMU_ANGLE_TEST_MAIN_H
#define IMU_ANGLE_TEST_MAIN_H
#include "stm32f4xx_hal.h"
typedef struct { uint32_t CYCCNT, CTRL; } TestDWT;
typedef struct { uint32_t DEMCR; } TestCoreDebug;
extern TestDWT test_dwt;
extern TestCoreDebug test_core_debug;
#define DWT (&test_dwt)
#define CoreDebug (&test_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#define __NOP() (++test_dwt.CYCCNT)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#endif
