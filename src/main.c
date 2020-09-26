#include <stdio.h>
#include "fastm_uart.h"
#include "fastm_i2c.h"
#include "fastm_spi.h"

// TODO: remove
#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_gpio.h"

int main(void)
{
  u8 buff_i2c[10] = {0};
  u8 buff_spi[10] = {0};

  // GPIO init
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // I2C init/test
  printf("[%s]: i2c init!\n\r", __TIME__);
  i2c_init(1);
  printf("[%s]: i2c write!\n\r", __TIME__);
  i2c_write(1, 0x50, 0, 2, (u8 *)__TIME__, 8);
  mdelay(20); // Write cycle delay. For EEPROMS and other memory.
  printf("[%s]: i2c read!\n\r", __TIME__);
  i2c_read(1, 0x50, 0, 2, buff_i2c, 8);
  printf("[%s]: Read data from I2C: %s\n\r", __TIME__, buff_i2c);

  // SPI init/test
  printf("[%s]: spi init!\n\r", __TIME__);
  spi_init(1);
  printf("[%s]: spi write!\n\r", __TIME__);
  spi_write(1, (u8 *)__TIME__, 8);
  printf("[%s]: spi read!\n\r", __TIME__);
  spi_read(1, buff_spi, 8);
  printf("[%s]: Read data from SPI: %s\n\r", __TIME__, buff_spi);

  while (1) {
    printf("[%s]: UART printf test!\n\r", __TIME__);

    // GPIO toggle
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
    delay(1);
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
    delay(1);
  }
}
