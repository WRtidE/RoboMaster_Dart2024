#include "struct_typedef.h"
#include "gpio.h"

uint16_t get_switch(uint16_t switch_id)
{
	  if(switch_id == 0) //pitch��λ
		{
			return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6);
		}
	  if(switch_id == 1) //yaw��λ
		{
			return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);
		}
		if(switch_id == 2) //�Ƹ���λ
		{
			return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8);
		}
		if(switch_id == 3)
		{
			return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9);
		}
}
