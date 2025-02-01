#ifndef INC_stm32f446XX_SPI_DRIVER_H_
#define INC_stm32f446XX_SPI_DRIVER_H_

#include "stm32f446xx.h"

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct {
	SPI_Mode_t SPI_DeviceMode;
	SPI_BusConfig_t SPI_BusConfig;
	SPI_Clk_Div_t SPI_SclkSpeed;
	SPI_DFF_Bits_t SPI_DFF;
	SPI_Pol_t SPI_CPOL;
	SPI_Phase_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

/*
 *Handle structure for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t 	*pSPIx;   /* This holds the base address of SPIx(x:0,1,2,3) peripheral */
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer; /* To store the app. Tx buffer address  */
	uint8_t 		*pRxBuffer;	/* To store the app. Rx buffer address  */
	uint32_t 		TxLen;		/* To store Tx len  */
	uint32_t 		RxLen;		/* To store Tx len  */
	uint8_t 		TxState;	/* To store Tx state  */
	uint8_t 		RxState;	/* To store Rx state  */
} SPI_Handle_t;

/*
 * SPI application states
 */
typedef enum {
	SPI_READY,
	SPI_BUSY_IN_RX,
	SPI_BUSY_IN_TX
} SPI_Status_t;

/*
 * Possible SPI Application events
 */
typedef enum {
 	SPI_EVENT_TX_CMPLT = 1,
 	SPI_EVENT_RX_CMPLT,
 	SPI_EVENT_OVR_ERR,
 	SPI_EVENT_CRC_ERR 
} SPI_Event_t;

/*
 * @SPI_DeviceMode
 */
typedef enum {
 	SPI_DEVICE_MODE_SLAVE,
 	SPI_DEVICE_MODE_MASTER
} SPI_Mode_t;
/*
 * @SPI_BusConfig
 */
typedef enum {
	SPI_BUS_CONFIG_FD = 1,
	SPI_BUS_CONFIG_HD,
	SPI_BUS_CONFIG_SIMPLEX_RXONLY
} SPI_BusConfig_t;

/*
 * @SPI_SclkSpeed
 */
typedef enum {
	SPI_SCLK_SPEED_DIV2,
	SPI_SCLK_SPEED_DIV4,
	SPI_SCLK_SPEED_DIV8,
	SPI_SCLK_SPEED_DIV16,
	SPI_SCLK_SPEED_DIV32,
	SPI_SCLK_SPEED_DIV64,
	SPI_SCLK_SPEED_DIV128,
	SPI_SCLK_SPEED_DIV256
} SPI_Clk_Div_t;

/*
 * @SPI_DFF
 */
typedef enum {
	SPI_DFF_8BITS,
	SPI_DFF_16BITS
} SPI_DFF_Bits_t;
/*
 * @CPOL
 */
typedef enum {
	SPI_CPOL_LOW,
	SPI_CPOL_HIGH
} SPI_Pol_t;
/*
 * @CPHA
 */
typedef enum {
	SPI_CPHA_LOW,
	SPI_CPHA_HIGH
} SPI_Phase_t;

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN  1
#define SPI_SSM_DI  0

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG   ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG  ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG  ( 1 << SPI_SR_BSY)

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_stm32f446XX_SPI_DRIVER_H_ */
