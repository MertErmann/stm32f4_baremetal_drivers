/***********************************************************************************
 * File        : gpio_driver.h
 *
 * Description : GPIO driver public interface.
 *               - GPIO_PinConfig_t and GPIO_Handle_t structures
 *               - GPIO pin mode, speed, output type, pull-up/down, AF macros
 *               - Public API prototypes:
 *                   * GPIO_PCLK_Control()
 *                   * GPIO_Init(), GPIO_DeInit()
 *                   * Data read/write (pin/port)
 *                   * Toggle pin
 *                   * IRQ configuration, priority, and handling
 ***********************************************************************************/


#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_


#include "stm32f407xx.h"


typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;   			/*@Possible Modes from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;				/*@Possible Modes from @GPIO_SPEED_MODES*/
	uint8_t GPIO_PinPuPdControl;		/*@Possible Modes from @GPIO_OUTPUT_TYPE*/
	uint8_t GPIO_PinOPType;				/*@Possible Modes from @GPIO_PUPD*/
	uint8_t GPIO_PinAltFunMode;			/*@Possible Modes from @ALT_Function*/


}GPIO_PinConfig_t;


typedef struct{

	GPIO_RegDef_t 		*pGPIOx;
	GPIO_PinConfig_t 	 GPIO_PinConfig;


}GPIO_Handle_t;


/*GPIO_PIN_MODES*/
#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALT	 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*GPIO_OUTPUT_TYPE*/
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*GPIO_SPEED_MODES*/
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*GPIO_PUPD*/
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2

/*ALT_Function*/
#define GPIO_AF0   0x0U
#define GPIO_AF1   0x1U
#define GPIO_AF2   0x2U
#define GPIO_AF3   0x3U
#define GPIO_AF4   0x4U
#define GPIO_AF5   0x5U
#define GPIO_AF6   0x6U
#define GPIO_AF7   0x7U
#define GPIO_AF8   0x8U
#define GPIO_AF9   0x9U
#define GPIO_AF10  0xAU
#define GPIO_AF11  0xBU
#define GPIO_AF12  0xCU
#define GPIO_AF13  0xDU
#define GPIO_AF14  0xEU
#define GPIO_AF15  0xFU




/*Clock Enable for GPIO*/
void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi );

/*Init and De-Init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Data Read and Write*/
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ Config and Handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);



#endif /* INC_GPIO_DRIVER_H_ */
