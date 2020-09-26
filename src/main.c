#include <stdio.h>
#include "fastm_uart.h"
#include "fastm_i2c.h"
#include "fastm_spi.h"

// TODO: remove
#include "stm32f1xx.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_gpio.h"

void ssd1306Init(void)
{
  u8 cmd_buf[26] = {0x00, 0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D,
                    0x14, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB,
                    0x40, 0xA4, 0xA6, 0x20, 0x00, 0xAF};
  
  u8 dat_buf[4] = {0x00, 0x00, 0x10, 0x40};
  u8 data[1025] = {0x40};

  for (int i = 1; i <= 1024; ++i) {
    data[i] = rand();
  }
  
  i2c_write(1, 0x3C, 0, 2, cmd_buf, 26);
  i2c_write(1, 0x3C, 0, 2, dat_buf, 4);
  i2c_write(1, 0x3C, 0, 2, data, sizeof(data));
}

int main(void)
{
  u8 buff_i2c[8] = {0};
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
  //i2c_write(1, 0x3C, 0, 2, buff_i2c, 4);
  ssd1306Init();
  mdelay(20); // Write cycle delay. For EEPROMS and other memory.
  // printf("[%s]: i2c read!\n\r", __TIME__);
  // i2c_read(1, 0x50, 0, 2, buff_i2c, 8);
  // printf("[%s]: Read data from I2C: %s\n\r", __TIME__, buff_i2c);

  // // SPI init/test
  // printf("[%s]: spi init!\n\r", __TIME__);
  // spi_init(1);
  // printf("[%s]: spi write!\n\r", __TIME__);
  // spi_write(1, (u8 *)__TIME__, 8);
  // printf("[%s]: spi read!\n\r", __TIME__);
  // spi_read(1, buff_spi, 8);
  // printf("[%s]: Read data from SPI: %s\n\r", __TIME__, buff_spi);

  while (1) {
    printf("[%s]: UART printf test!\n\r", __TIME__);

    // GPIO toggle
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);
    delay(1);
    LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);
    delay(1);
  }
}
