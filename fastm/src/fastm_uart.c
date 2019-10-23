#include "fastm_uart.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"

struct uart_config {
	uint8_t init;
	uint8_t speed;
	void *base;
} uart[FASTM_UART_MAX_NUM];

static void * __attribute__((unused)) uart_get_base(uint8_t uart_num)
{
	switch (uart_num){
	case 1:
		return USART1;
	case 2:
		return USART2;
	case 3:
		return USART3;
	default:
		return 0;
	}
}

int uart_init(uint8_t uart_num, uint32_t speed)
{
	LL_GPIO_InitTypeDef uart_gpio_init;
	LL_USART_InitTypeDef uart_init;

	void *uart_base;

	if (uart_num > 8) return -1;
	if (uart[uart_num].init) return 0;

	uart_gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
	uart_gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	uart_gpio_init.Pull = LL_GPIO_PULL_UP;
	uart_gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;

	uart_init.DataWidth = LL_USART_DATAWIDTH_8B;
	uart_init.StopBits = LL_USART_STOPBITS_1;
	uart_init.Parity = LL_USART_PARITY_NONE;
	uart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	uart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;

	switch (uart_num) {
	case 1:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

		LL_GPIO_AF_EnableRemap_USART1();

		uart_gpio_init.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
		LL_GPIO_Init(GPIOB, &uart_gpio_init);
		break;
	case 2:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

		LL_GPIO_AF_EnableRemap_USART2();

		uart_gpio_init.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 |
									LL_GPIO_PIN_5 | LL_GPIO_PIN_6 |
									LL_GPIO_PIN_7;
		LL_GPIO_Init(GPIOD, &uart_gpio_init);
		break;
	case 3:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

		LL_GPIO_AF_EnableRemap_USART3();

		uart_gpio_init.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 |
									LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |
									LL_GPIO_PIN_12;
		LL_GPIO_Init(GPIOD, &uart_gpio_init);
		break;
	default:
		return -1;
	}

	// Init USART
	uart_base = uart_get_base(uart_num);
	uart_init.BaudRate = speed;

	LL_USART_Init(uart_base, &uart_init);
	LL_USART_Enable(uart_base);

	// Save config
	uart[uart_num].init = 1;
	uart[uart_num].speed = speed;
	uart[uart_num].base = uart_base;

	return 0;
}

int uart_read(uint8_t uart_num, char *str, ssize_t len)
{
	ssize_t bytes_to_read = len;

	while (bytes_to_read) {
		if (str == NULL) return 0;

		while (!LL_USART_IsActiveFlag_RXNE(uart[uart_num].base));
		*str = LL_USART_ReceiveData8(uart[uart_num].base);

		while (!LL_USART_IsActiveFlag_TC(uart[uart_num].base));
		LL_USART_TransmitData8(uart[uart_num].base, *str++);

		--bytes_to_read;
	}

	return len;
}

int uart_write(uint8_t uart_num, const char *str, ssize_t len)
{
	ssize_t bytes_to_read = len;

	while (bytes_to_read) {
		if (str == NULL) return 0;

		while (!LL_USART_IsActiveFlag_TC(uart[uart_num].base));
		LL_USART_TransmitData8(uart[uart_num].base, *str++);

		--bytes_to_read;
	}

	return len;
}
