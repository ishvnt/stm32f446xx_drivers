#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

#define HSI_Frequency 16000000
#define HSE_Frequency 8000000

// returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

// returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);
#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
