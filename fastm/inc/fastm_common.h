#ifndef __FASTM_COMMON__
#define __FASTM_COMMON__

#include <stdint.h>
#include <unistd.h>
#include "stm32f1xx.h"

#ifndef NULL
	#define NULL (void *)0
#endif /* ifndef NULL */

#ifndef PRINTF_UART_NUM
	#define PRINTF_UART_NUM FASTM_USART1
#endif /* ifndef PRINTF_UART_NUM */

#ifndef PRINTF_UART_SPEED
	#define PRINTF_UART_SPEED 115200
#endif /* ifndef PRINTF_UART_SPEED */

#define udelay(us)	{volatile u32 i = us * (HSE_VALUE / 2 / 1000000); while (--i);}
#define mdelay(ms)	{volatile u32 i = ms * (HSE_VALUE / 2 / 1000); while (--i);}
#define delay(s)	{volatile u32 i = s * (HSE_VALUE / 2); while (--i);}

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#endif /* ifndef __FASTM_COMMON__ */
