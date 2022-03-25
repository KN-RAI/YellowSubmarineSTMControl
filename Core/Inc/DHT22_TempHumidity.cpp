#include <DHT22_TempHumidity.h>
#include"main.h"

//###############################################################################################################
void  DHT22::pinChangeCallBack()
{
  this->time = HAL_GetTick();
  if(this->cnt < sizeof(this->data)-1)
  {
	  this->data[this->cnt] = this->tim->Instance->CNT - this->lastCNT;
	  this->lastCNT = this->tim->Instance->CNT;
	  this->cnt++;
  }
}
//###############################################################################################################
void  DHT22::delayUs(uint16_t delay)
{
	this->tim->Instance->CNT=0;
  while(this->tim->Instance->CNT < delay);
}
//###############################################################################################################
void  DHT22::output()
{
  GPIO_InitTypeDef  gpio;
  this->gpio->BSRR = this->pin;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = this->pin;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(this->gpio,&gpio);
}
//###############################################################################################################
void  DHT22::input()
{
  GPIO_InitTypeDef  gpio;
  gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = this->pin;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(this->gpio,&gpio);
}
//###############################################################################################################
bool  DHT22::decode(uint8_t *byteArray)
{
  int8_t bit;
      if((this->data[0] < 60) || (this->data[0] > 100) || (this->data[1] < 60) || (this->data[1] > 100))
        return false;
      bit = 7;
      for(uint8_t i=0 ; i<80 ; i+=2)
      {
        if((this->data[i+2] >= 35) && (this->data[i+2] <= 70))
        {
          if((this->data[i+3] >= 10) && (this->data[i+3] <= 45))
            *byteArray &= ~(1<<bit);
          else if((this->data[i+3] >= 55) && (this->data[i+3] <= 95))
            *byteArray |= (1<<bit);
          else
            return false;
          bit--;
          if(bit == -1)
          {
            bit = 7;
            byteArray++;
          }
        }
        else
          return false;
      }

}
//###############################################################################################################
bool DHT22::init(TIM_HandleTypeDef *tim,uint16_t  timerBusFrequencyMHz, GPIO_TypeDef *gpio, uint16_t  pin)
{
	this->tim = tim;
	this->gpio = gpio;
	this->pin = pin;
  output();
  this->tim->Init.Prescaler = timerBusFrequencyMHz - 1;
  this->tim->Init.CounterMode = TIM_COUNTERMODE_UP;
  this->tim->Init.Period = 0xFFFF;
  this->tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(this->tim);
  HAL_TIM_Base_Start(this->tim);
  while(HAL_GetTick()<2000)
	   HAL_Delay(5);
  return true;
}
//###############################################################################################################
bool  DHT22::read()
{
  uint32_t  startTime;

      output();
      this->gpio->BSRR = (this->pin)<<16;
      HAL_Delay(1);
      this->gpio->BSRR = this->pin;
      delayUs(5);
      this->gpio->BSRR = (this->pin)<<16;
      delayUs(5);
      this->cnt = 0;
      this->lastCNT = 0;
      this->tim->Instance->CNT = 0;
      startTime = HAL_GetTick();
      input();
      while(1)
      {
        if(HAL_GetTick() - this->time > 1)
        {
          uint8_t data[5];
          if(decode(data) == false)
            return false;
          if(((data[0] + data[1] + data[2] + data[3]) & 0x00FF) != data[4])
            return false;

          this->temperature = (float)(data[2]*256 + data[3]) / 10.0f;
          this->humidity = (float)(data[0]*256 + data[1]) / 10.0f;

          this->dataValid = true;
          output();
          return true;
        }
      }

}
