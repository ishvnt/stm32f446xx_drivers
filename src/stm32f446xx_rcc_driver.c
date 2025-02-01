#include "stm32f446xx_rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t SystemClk;
	uint8_t clk_src = ((RCC->CFGR >> 2) & 0x3);
	if(clk_src == 0)
	{
		SystemClk = HSI_Frequency;
	}
	else if(clk_src == 1)
	{
		SystemClk = HSE_Frequency;
	}
	else if (clk_src == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	uint8_t temp = ((RCC->CFGR >> 4 ) & 0xF);
	uint8_t ahbp;
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);
	uint8_t apb1p;
	if(temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}
	
	uint32_t pclk1 =  (SystemClk/(ahbp*apb1p));
	return pclk1;
}

/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClk;
	uint8_t clk_src = (RCC->CFGR >> 2) & 0X3;
	if(clk_src == 0)
	{
		SystemClk = 16000000;
	}
	else if(clk_src == 1)
	{
		SystemClk = 8000000;
	}
	else if (clk_src == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	uint8_t tmp = (RCC->CFGR >> 4 ) & 0xF;
	uint8_t ahbp;
	if(tmp < 8)
	{
		ahbp = 1;
	}
	else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	uint8_t apb2p;
	if(tmp < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	uint32_t pclk2 = (SystemClk/(ahbp*apb2p));
	return pclk2;
}

/*********************************************************************
 * @fn      		  - RCC_GetPLLOutputClock
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint32_t  RCC_GetPLLOutputClock()
{
	uint32_t SystemClk;
	uint8_t pll_src = ((RCC->PLLCFGR >> 22) & 0x1);
	if(pll_src == 0)
	{
		SystemClk = HSI_Frequency;
	}
	else if(pll_src == 1)
	{
		SystemClk = HSE_Frequency;
	}

	uint8_t pll_in_div_factor = ((RCC->PLLCFGR >> 5) & 0x3F);
	if(pll_in_div_factor == 0 || pll_in_div_factor == 1)
	{
		return -1;
	}
	
	uint8_t pll_vco_mul_factor = ((RCC->PLLCFGR >> 6) & 0x1FF);
	if(pll_vco_mul_factor < 50 || pll_vco_mul_factor > 432)
	{
		return -1;
	}

	uint8_t pll_sysclk_div_factor = ((RCC->PLLCFGR >> 16) & 0x2);

	uint32_t pll_clk = ((SystemClk/pll_in_div_factor)*pll_vco_mul_factor)/pll_sysclk_div_factor;
	return pll_clk;
}

