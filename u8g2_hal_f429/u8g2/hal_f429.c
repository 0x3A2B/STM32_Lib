#include "u8x8.h"
#include "main.h"

I2C_HandleTypeDef *oled_i2c;

void u8g2_i2c_handle_init(I2C_HandleTypeDef *i2c){
   oled_i2c = i2c;
   return ;
}

uint8_t u8x8_gpio_and_delay_stm32f4_hal(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  GPIO_PinState pin_status ;
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
		  //HAL_Delay(100);
      break;							// can be used to setup pins
    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
      break;    
    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
			__NOP();
      break;
    case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
      //delay_us(arg_int * 10);
      break;
    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
      //HAL_Delay(arg_int);
      break;
    case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
      //if(arg_int == 1) delay_us(5);
      //else if(arg_int ==4) delay_us(1);
      break;	          						// arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_I2C_CLOCK:
      if(arg_int){                    // arg_int=1: Input dir with pullup high for I2C clock pin
        //HAL_GPIO_WritePin(OLED_GPIO, oled_scl, GPIO_PIN_SET);
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
      }
      else{                           // arg_int=0: Output low at I2C clock pin
        //HAL_GPIO_WritePin(OLED_GPIO, oled_scl, GPIO_PIN_RESET);
				//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
      }
      break;							
    case U8X8_MSG_GPIO_I2C_DATA:			
      if(arg_int){                    // arg_int=1: Input dir with pullup high for I2C data pin
        //HAL_GPIO_WritePin(OLED_GPIO, oled_sda, GPIO_PIN_SET);
      }
      else{                           // arg_int=0: Output low at I2C data pin
        //HAL_GPIO_WritePin(OLED_GPIO, oled_sda, GPIO_PIN_RESET);
      }
      break;
    case U8X8_MSG_GPIO_MENU_SELECT:
      pin_status = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			if(pin_status == GPIO_PIN_SET)
				u8x8_SetGPIOResult(u8x8, 1);
			else
				u8x8_SetGPIOResult(u8x8, 0);
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      pin_status = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
      if(pin_status == GPIO_PIN_SET)
				u8x8_SetGPIOResult(u8x8, 1);
			else
				u8x8_SetGPIOResult(u8x8, 0);
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      //pint_status = HAL_GPIO_ReadPin();
      u8x8_SetGPIOResult(u8x8, 0);
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
      //pint_status = HAL_GPIO_ReadPin();
      u8x8_SetGPIOResult(u8x8, 0);
      break;
    default:
      u8x8_SetGPIOResult(u8x8, 1);			// default return value
      break;
  }
  return 1;
}

uint8_t u8x8_byte_i2c_stm32_hal(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  static uint8_t buffer[32];		/* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;
 
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;      
      while( arg_int > 0 ){
				buffer[buf_idx++] = *data;
				data++;
				arg_int--;
      }      
      break;
    case U8X8_MSG_BYTE_INIT:
			//HAL_Delay(100);
      /* add your custom code to init i2c subsystem */
      break;
    case U8X8_MSG_BYTE_SET_DC:
      /* ignored for i2c */
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      HAL_I2C_Master_Transmit(oled_i2c, u8x8_GetI2CAddress(u8x8),  buffer, buf_idx,0x3fff); //u8x8_GetI2CAddress(u8x8) >> 1
      break;
    default:
      return 0;
  }
  return 1;
}
