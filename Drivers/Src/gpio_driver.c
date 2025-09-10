/***********************************************************************************
 * File        : gpio.c
 *
 * Description : GPIO driver implementation.
 *               - Clock control, initialization, de-initialization
 *               - Pin/port read and write operations
 *               - Toggle pin functionality
 *               - NVIC IRQ configuration and handling
 ***********************************************************************************/


#include "gpio_driver.h"



/***********************************************************************************
 * @function name                 - GPIO_PCLK_Control
 *
 * This function enables/disables the bus clock in the pointed GPIO port.
 *
 * @param[in]  pGPIOx             - Pointer to GPIO port base address (e.g., GPIOA, GPIOB)
 * @param[in]  EnorDi             - Enable (1) or Disable (0)
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

/***********************************************************************************
 * @function name                 - GPIO_Init
 *
 * This function initializes a GPIO pin according to the configuration in GPIO_Handle_t.
 *
 * @param[in]  pGPIOHandle        - Pointer to GPIO handle (port + pin configuration)
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	uint32_t pin_Number = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint32_t pinMode   = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
	uint32_t pos2      = pin_Number * 2U;
	/*GPIO Mode config up to IQR*/
	if (pinMode <= GPIO_MODE_ANALOG) {
	        pGPIOHandle->pGPIOx->MODER &= ~(0x3U << pos2);
	        pGPIOHandle->pGPIOx->MODER |=  ((pinMode & 0x3U) << pos2);
	    }else{


	    		if(pinMode == GPIO_MODE_IT_FT){
	    					EXTI->FTSR |=  (1 << pin_Number);
	    					EXTI->RTSR &= ~(1 <<  pin_Number);
	    		}else if(pinMode == GPIO_MODE_IT_RT){
							EXTI->RTSR |=  (1 << pin_Number);
							EXTI->FTSR &= ~(1 <<  pin_Number);
	    		}else if(pinMode == GPIO_MODE_IT_RFT){
	    					EXTI->RTSR |=  (1 << pin_Number);
	    					EXTI->FTSR |=  (1 << pin_Number);
	    		}

	    		uint8_t exti_line = pin_Number;
	    		uint8_t exti_reg_index = exti_line/4;
	    		uint8_t exti_pos = (exti_line % 4)*4;
	    		uint8_t port_code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	    		SYSCFG_PCLK_EN();
	    		SYSCFG->EXTICR[exti_reg_index] &= ~(0xF << exti_pos);
	    		SYSCFG->EXTICR[exti_reg_index] |= (port_code << exti_pos);

	    		EXTI->IMR |= (1 << pin_Number);

	    }

	/*GPIO Speed*/
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3U << pos2);
	pGPIOHandle->pGPIOx->OSPEEDR |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << pos2);

	/*GPIO OUTPUT TYPE*/
	pGPIOHandle->pGPIOx->OTYPER &= ~(1U << pin_Number);
	pGPIOHandle->pGPIOx->OTYPER |=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pin_Number);

	/*GPIO PUPD*/
	pGPIOHandle->pGPIOx->PUPDR	&= ~(0x3U << pos2);
	pGPIOHandle->pGPIOx->PUPDR	|=  (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << pos2);

	/*Alternate Function*/
	if (pinMode == GPIO_MODE_ALT)
	    {
	        uint32_t afr_index = pin_Number >> 3U;
	        uint32_t afr_pos   = (pin_Number & 0x7U) * 4U;
	        pGPIOHandle->pGPIOx->AFR[afr_index] &= ~(0xFU << afr_pos);
	        pGPIOHandle->pGPIOx->AFR[afr_index] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << afr_pos);
	    }
}

/***********************************************************************************
 * @function name                 - GPIO_DeInit
 *
 * This function resets all registers of the given GPIO port to their default state.
 *
 * @param[in]  pGPIOx             - Pointer to GPIO port base address
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

		if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_REG_RESET();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_REG_RESET();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_REG_RESET();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_REG_RESET();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_REG_RESET();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_REG_RESET();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_REG_RESET();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_REG_RESET();
		}
}

/***********************************************************************************
 * @function name                 - GPIO_ReadFromInputPin
 *
 * This function reads the logic level of a specific GPIO input pin.
 *
 * @param[in]  pGPIOx             - Pointer to GPIO port base address
 * @param[in]  PinNumber          - Pin number (0–15)
 *
 * @retval                        - 0 if LOW, 1 if HIGH
 *
 ***********************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}

/***********************************************************************************
 * @function name                 - GPIO_ReadFromInputPort
 *
 * This function reads the logic levels of all pins in the GPIO port.
 *
 * @param[in]  pGPIOx             - Pointer to GPIO port base address
 *
 * @retval                        - 16-bit value of the port input data
 *
 ***********************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;

}

/***********************************************************************************
 * @function name                 - GPIO_WriteToOutputPin
 *
 * This function writes a logic level to a specific GPIO output pin.
 *
 * @param[in]  pGPIOx             - Pointer to GPIO port base address
 * @param[in]  PinNumber          - Pin number (0–15)
 * @param[in]  Value              - 0 to reset (LOW), 1 to set (HIGH)
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

    if(Value == GPIO_PIN_SET){
        pGPIOx->ODR |=  (1 << PinNumber);
    }else{
        pGPIOx->ODR &= ~(1 << PinNumber);
    }

}

/***********************************************************************************
 * @function name                 - GPIO_WriteToOutputPort
 *
 * This function writes a 16-bit value to the GPIO port output data register.
 *
 * @param[in]  pGPIOx             - Pointer to GPIO port base address
 * @param[in]  Value              - 16-bit value to write to the port
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;

}

/***********************************************************************************
 * @function name                 - GPIO_ToggleOutputPin
 *
 * This function toggles the output state of a specific GPIO pin.
 *
 * @param[in]  pGPIOx             - Pointer to GPIO port base address
 * @param[in]  PinNumber          - Pin number (0–15)
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1<<PinNumber);

}

/***********************************************************************************
 * @function name                 - GPIO_IRQConfig
 *
 * This function configures NVIC for the given IRQ number with the specified priority
 * and enables/disables the interrupt in NVIC.
 *
 * @param[in]  IRQNumber          - IRQ number (device specific)
 * @param[in]  IRQPriority        - Priority value (device specific)
 * @param[in]  EnorDi             - Enable (1) or Disable (0)
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(IRQNumber <= 31){
				/*ISER0 Register*/
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){
				/*ISER1 Register*/
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));


		}else if(IRQNumber >= 64 && IRQNumber < 96){
				/*ISER2 Register*/
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}

	}else{

		if(IRQNumber <= 31){
				/*ICER0 Register*/
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64){
				/*ICER1 Register*/
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));


		}else if(IRQNumber >= 64 && IRQNumber < 96){
				/*ICER2 Register*/
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}




	}




}

/***********************************************************************************
 * @function name                 - GPIO_IRQPriorityConfig
 *
 * This function sets the priority of a given IRQ number in the NVIC.
 *
 * @param[in]  IRQNumber          - IRQ number (device specific, see vector table)
 * @param[in]  IRQPriority        - Priority value (0–15 for STM32F4, only upper 4 bits used)
 *
 * @note                          - STM32F4 implements only the upper 4 bits of priority
 *                                  (NO_PR_BITS_IMPLEMENTED = 4). Lower bits are ignored.
 *                                - Each IPR register holds priority fields for 4 IRQs.
 *                                - 'shift_amount' calculates the correct bit position for
 *                                  the target IRQ inside its IPR slot.
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint32_t iprx = IRQNumber / 4;
	uint32_t iprx_section = IRQNumber % 4;
	uint32_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);


}

/***********************************************************************************
 * @function name                 - GPIO_IRQHandling
 *
 * This function handles a GPIO/EXTI interrupt for the given pin (clears the pending flag).
 *
 * @param[in]  PinNumber          - Pin number (0–15) associated with the EXTI line
 *
 * @retval                        - none
 *
 ***********************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber){


	if(EXTI->PR & (1 << PinNumber)){

		EXTI->PR |= (1 << PinNumber);
	}



}
