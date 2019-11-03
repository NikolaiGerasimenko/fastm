#include "fastm_spi.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"

struct spi_config {
	uint8_t init;
	void *base;
} spi[FASTM_SPI_MAX_NUM];

static void * __attribute__((unused)) spi_get_base(uint8_t spi_num)
{
	switch (spi_num){
	case 1:
		return SPI1;
	case 2:
		return SPI2;
	default:
		return 0;
	}
}

int spi_init(uint8_t spi_num)
{
	LL_GPIO_InitTypeDef  spi_gpio_init;
	LL_SPI_InitTypeDef   spi_init;

	void *spi_base;

	if (spi_num > 8) return -1;
	if (spi[spi_num].init) return 0;

	spi_gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
	spi_gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    spi_gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	spi_gpio_init.Pull = LL_GPIO_PULL_DOWN;
	
	spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
	spi_init.Mode = LL_SPI_MODE_MASTER;
	spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	spi_init.ClockPolarity = LL_SPI_POLARITY_LOW;
	spi_init.ClockPhase = LL_SPI_PHASE_1EDGE;
	spi_init.NSS = LL_SPI_NSS_SOFT;
	spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV128;
	spi_init.BitOrder = LL_SPI_MSB_FIRST;

	switch (spi_num) {
	case 1:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

		LL_GPIO_AF_EnableRemap_SPI1();

		spi_gpio_init.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
		LL_GPIO_Init(GPIOB, &spi_gpio_init);
		break;
	default:
		// TODO: Add other SPIs
		return -1;
	}

	// Init SPI
	spi_base = spi_get_base(spi_num);

	LL_SPI_Init(spi_base, &spi_init);
	LL_SPI_Enable(spi_base);

	// Save config
	spi[spi_num].init = 1;
	spi[spi_num].base = spi_base;

	return 0;
}

int spi_read(u8 spi_num, u8 *data, ssize_t len)
{
	u32 i;
	if (!data) return -1;

	for (i = 0; i < len; ++i) {
		// Check if we already have some data in Rx buffer
		if (LL_SPI_IsActiveFlag_RXNE(spi[spi_num].base)) {
			data[i] = LL_SPI_ReceiveData8(spi[spi_num].base);
			continue;
		}

		// Send dummy to continue receiving data
		do {
			LL_SPI_TransmitData8(spi[spi_num].base, 0);
			while (LL_SPI_IsActiveFlag_BSY(spi[spi_num].base));
		} while (!LL_SPI_IsActiveFlag_RXNE(spi[spi_num].base));

		data[i] = LL_SPI_ReceiveData8(spi[spi_num].base);
	}

	return len;
}

int spi_write(u8 spi_num, const u8 *data, ssize_t len)
{
	u32 i;
	if (!data) return -1;

	for (i = 0; i < len; ++i) {
		LL_SPI_TransmitData8(spi[spi_num].base, data[i]);

		// Wait until byte will be transmitted
		while (!LL_SPI_IsActiveFlag_TXE(spi[spi_num].base));
		while (LL_SPI_IsActiveFlag_BSY(spi[spi_num].base));

		// Flush Rx buffer after send first byte of data
		if (i == 0) {
			while (LL_SPI_IsActiveFlag_RXNE(spi[spi_num].base))
				LL_SPI_ReceiveData8(spi[spi_num].base);
		}
	}

	return len;
}
