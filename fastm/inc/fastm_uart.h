#ifndef __FASTM_UART__
#define __FASTM_UART__

#include "fastm_common.h"

enum {
	FASTM_USART1 = 1,
	FASTM_USART2,
	FASTM_USART3,
	FASTM_UART_MAX_NUM
};

int uart_init(uint8_t uart_num, uint32_t speed);

int uart_read(uint8_t uart_num, char *str, ssize_t len);
int uart_write(uint8_t uart_num, const char *str, ssize_t len);

#endif /* ifndef __FASTM_UART_ */
