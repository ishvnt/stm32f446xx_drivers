#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct {
	GPIO_Pin_No_t GPIO_PinNumber;
	GPIO_Mode_t GPIO_PinMode;
	GPIO_Speed_t GPIO_PinSpeed;
	GPIO_PUPD_t GPIO_PinPuPdControl;
	GPIO_Outout_Type_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;       		/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;   	/* This holds GPIO pin configuration settings */

}GPIO_Handle_t;

/*
 * GPIO pin numbers
 */
typedef enum {
	GPIO_PIN_NO_0,
	GPIO_PIN_NO_1,
  	GPIO_PIN_NO_2,
   	GPIO_PIN_NO_3,
    GPIO_PIN_NO_4, 
	GPIO_PIN_NO_5, 
	GPIO_PIN_NO_6, 
	GPIO_PIN_NO_7, 
	GPIO_PIN_NO_8, 
	GPIO_PIN_NO_9, 
	GPIO_PIN_NO_10, 
	GPIO_PIN_NO_11, 
	GPIO_PIN_NO_12, 
	GPIO_PIN_NO_13, 
	GPIO_PIN_NO_14, 
	GPIO_PIN_NO_15
} GPIO_Pin_No_t;

/*
 * GPIO pin possible modes
 */
typedef enum {
	GPIO_MODE_IN, 
	GPIO_MODE_OUT, 
	GPIO_MODE_ALTFN,
	GPIO_MODE_ANALOG,
	GPIO_MODE_IT_FT,  
	GPIO_MODE_IT_RT,  
	GPIO_MODE_IT_RFT
} GPIO_Mode_t;

/*
 * GPIO pin possible output speeds
 */
typedef enum {
	GPIO_SPEED_LOW,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_FAST,
	GPOI_SPEED_HIGH
} GPIO_Speed_t;

/*
 * GPIO pin possible output types
 */
typedef enum {
	GPIO_OP_TYPE_PP,
	GPIO_OP_TYPE_OD
} GPIO_Outout_Type_t;

/*
 * GPIO pin pull up AND pull down configuration macros
 */
typedef enum {
	GPIO_NO_PUPD,
	GPIO_PIN_PU,
	GPIO_PIN_PD,
} GPIO_PUPD_t;

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En_or_Di);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, GPIO_Pin_No_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, GPIO_Pin_No_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, GPIO_Pin_No_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(GPIO_Pin_No_t PinNumber);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
