#ifndef __FASTM_I2C__
#define __FASTM_I2C__

#include "fastm_common.h"

enum {
	FASTM_I2C1 = 1,
	FASTM_I2C2,
	FASTM_I2C3,
	FASTM_I2C_MAX_NUM
};

int i2c_init(uint8_t i2c_num);
int i2c_read(u8 i2c_num, u8 slave_addr, u32 reg_addr, u8 reg_addr_size, u8 *data, ssize_t len);
int i2c_write(u8 i2c_num, u8 slave_addr, u32 reg_addr, u8 reg_addr_size, const u8 *data, ssize_t len);

#endif /* ifndef __FASTM_I2C_ */
