#ifndef _DHT_H
#endif
#define _DHT_H

#ifndef _DHT_CONF_H
#define _DHT_CONF_H

#define     _DHT_USE_FREERTOS       1

#endif

#include <stdbool.h>
#include "main.h"


class DHT22
{
public:
  TIM_HandleTypeDef   *tim;
  GPIO_TypeDef        *gpio;
  uint16_t            pin;
  uint8_t             data[84];
  uint16_t            cnt;
  uint32_t            time;
  uint32_t            lastCNT;
  float               temperature;
  float               humidity;
  bool                dataValid;

  void  pinChangeCallBack();
  bool  init( TIM_HandleTypeDef *tim, uint16_t  timerBusFrequencyMHz, GPIO_TypeDef *gpio, uint16_t  pin);
  bool  read();

  void  delayUs(uint16_t DelayUs);
  void  output();
  void  input();
  bool  decode(uint8_t *byteArray);

};
