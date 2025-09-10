#include "stm32f407xx.h"
#include "gpio_driver.h"

void delay(void){

	for(uint32_t i=0 ; i <500000 ; i ++);
}


int main(void){

	GPIO_Handle_t GpioLedGreen, GpioButton,GpioLedRed ;

	GpioLedGreen.pGPIOx = GPIOD;
	GpioLedGreen.GPIO_PinConfig.GPIO_PinNumber = 12;
	GpioLedGreen.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLedGreen.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLedGreen.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLedGreen.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PCLK_Control(GPIOD, ENABLE);
	GPIO_Init(&GpioLedGreen);


	GpioLedRed.pGPIOx = GPIOD;
	GpioLedRed.GPIO_PinConfig.GPIO_PinNumber = 14;
	GpioLedRed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLedRed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLedRed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLedRed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioLedRed);

	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = 0;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RFT;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PCLK_Control(GPIOA, ENABLE);
	GPIO_Init(&GpioButton);

	//IRQ
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, 15);
	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);


	while(1){


			GPIO_ToggleOutputPin(GPIOD, 12);
			delay();

	}

return 0;

}


void EXTI0_IRQHandler(void){


	GPIO_IRQHandling(0);
	GPIO_ToggleOutputPin(GPIOD, 14);















}
