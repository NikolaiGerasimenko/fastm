#ifndef __FASTM_SPI__
#define __FASTM_SPI__

#include <sys/types.h>
#include "fastm_common.h"

enum {
	FASTM_SPI1 = 1,
	FASTM_SPI2,
	FASTM_SPI3,
	FASTM_SPI_MAX_NUM
};

int spi_init(uint8_t spi_num);
int spi_read(u8 spi_num, u8 *data, ssize_t len);
int spi_write(u8 spi_num, const u8 *data, ssize_t len);

#endif /* ifndef __FASTM_SPI_ */
