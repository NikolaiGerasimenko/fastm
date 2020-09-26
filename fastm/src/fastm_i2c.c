#include "fastm_i2c.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_i2c.h"

// TODO: remove
#include <stdio.h>

struct i2c_config {
	uint8_t init;
	void *base;
} i2c[FASTM_I2C_MAX_NUM];

static void * __attribute__((unused)) i2c_get_base(uint8_t i2c_num)
{
	switch (i2c_num){
	case 1:
		return I2C1;
	case 2:
		return I2C2;
	default:
		return 0;
	}
}

static int wait_for_idle(u8 i2c_num) {
	u32 timeout = HSI_VALUE;

	while(LL_I2C_IsActiveFlag_BUSY(i2c[i2c_num].base)) {
		// TODO: Fix HSI timeout
		if (!(timeout--)) return -1;
	}

	return 0;
}

int i2c_init(uint8_t i2c_num)
{
	LL_GPIO_InitTypeDef  i2c_gpio_init;
	LL_I2C_InitTypeDef   i2c_init;

	void *i2c_base;

    // TODO: fix
	if (i2c_num > 2) return -1;
	if (i2c[i2c_num].init) return 0;

	i2c_gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
	i2c_gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    i2c_gpio_init.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	i2c_gpio_init.Pull = LL_GPIO_PULL_UP;

	i2c_init.PeripheralMode = LL_I2C_MODE_I2C;
    i2c_init.ClockSpeed = 50000; // 50kHz (max 400)
	i2c_init.DutyCycle = LL_I2C_DUTYCYCLE_2;
	i2c_init.OwnAddress1 = 0x00;
	i2c_init.TypeAcknowledge = LL_I2C_ACK;
	i2c_init.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

	switch (i2c_num) {
	case 1:
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

		LL_GPIO_AF_EnableRemap_I2C1();

		i2c_gpio_init.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
		LL_GPIO_Init(GPIOB, &i2c_gpio_init);
		break;
	case 2:
		// TODO: Add I2C2
	default:
		return -1;
	}

	// Init I2C
	i2c_base = i2c_get_base(i2c_num);

	LL_I2C_Init(i2c_base, &i2c_init);
	LL_I2C_Enable(i2c_base);

	// Save config
	i2c[i2c_num].init = 1;
	i2c[i2c_num].base = i2c_base;

	return 0;
}

int i2c_read(u8 i2c_num, u8 slave_addr, u32 reg_addr, u8 reg_addr_size, u8 *data, ssize_t len)
{
	ssize_t bytes_to_rcv = len;
	u8 *buf = data;

	LL_I2C_GenerateStartCondition(i2c[i2c_num].base);
    while (!LL_I2C_IsActiveFlag_SB(i2c[i2c_num].base));

	LL_I2C_TransmitData8(i2c[i2c_num].base, slave_addr << 1);
    while (!LL_I2C_IsActiveFlag_ADDR(i2c[i2c_num].base));

	// Unstretch the clock
    // TODO: fix i2c1
	LL_I2C_ReadReg(I2C1, SR2);

	LL_I2C_TransmitData8(i2c[i2c_num].base, reg_addr);
    while (!LL_I2C_IsActiveFlag_BTF(i2c[i2c_num].base));

    LL_I2C_TransmitData8(i2c[i2c_num].base, reg_addr >> 8);
    while (!LL_I2C_IsActiveFlag_BTF(i2c[i2c_num].base));

	// Generate Start
	LL_I2C_GenerateStartCondition(i2c[i2c_num].base);
    while (!LL_I2C_IsActiveFlag_SB(i2c[i2c_num].base));

	// Send I2C Device Address and clear ADDR
	LL_I2C_TransmitData8(i2c[i2c_num].base, (slave_addr << 1) | 1);
    while (!LL_I2C_IsActiveFlag_ADDR(i2c[i2c_num].base));

	// Unstretch the clock
    // TODO: fix i2c1
	LL_I2C_ReadReg(I2C1, SR2);

	while ((bytes_to_rcv--) > 1) {
        while (!LL_I2C_IsActiveFlag_RXNE(i2c[i2c_num].base));
		*buf = LL_I2C_ReceiveData8(i2c[i2c_num].base);
		buf++;
	}

	LL_I2C_AcknowledgeNextData(i2c[i2c_num].base, LL_I2C_NACK);

	LL_I2C_GenerateStopCondition(i2c[i2c_num].base);

    while (!LL_I2C_IsActiveFlag_RXNE(i2c[i2c_num].base));
	*buf = LL_I2C_ReceiveData8(i2c[i2c_num].base);

	wait_for_idle(i2c_num);
	LL_I2C_AcknowledgeNextData(i2c[i2c_num].base, LL_I2C_ACK);

	return len;
}

int i2c_write(u8 i2c_num, u8 slave_addr, u32 reg_addr, u8 reg_addr_size, const u8 *data, ssize_t len)
{
	ssize_t bytes_to_send = len;
	ssize_t addr_size = reg_addr_size;
	u8 *buf = (u8 *)data;

	LL_I2C_GenerateStartCondition(i2c[i2c_num].base);
    	printf("EE wr1\n\r");
    while (!LL_I2C_IsActiveFlag_SB(i2c[i2c_num].base));

	LL_I2C_TransmitData8(i2c[i2c_num].base, (slave_addr << 1) | 0);
    	printf("EE wr2\n\r");
    while (!LL_I2C_IsActiveFlag_ADDR(i2c[i2c_num].base));

	// Unstretch the clock
    // TODO: fix i2c1
	LL_I2C_ReadReg(I2C1, SR2);
    printf("EE wr3\n\r");

	// Send reg addr
	while (addr_size--) {
		LL_I2C_TransmitData8(i2c[i2c_num].base, reg_addr >> (8 * (reg_addr_size - (addr_size + 1))));
        while (!LL_I2C_IsActiveFlag_TXE(i2c[i2c_num].base));
	}
	printf("EE wr4\n\r");
    while (!LL_I2C_IsActiveFlag_BTF(I2C1));

	// Writing Data
    printf("EE wr5\n\r");
	while (bytes_to_send--) {
		LL_I2C_TransmitData8(i2c[i2c_num].base, *buf++);
        while (!LL_I2C_IsActiveFlag_TXE(i2c[i2c_num].base));
	}
    printf("EE wr6\n\r");

	// Wait for the data to be transmitted
    while (!LL_I2C_IsActiveFlag_BTF(i2c[i2c_num].base));
    printf("EE wr7\n\r");

	LL_I2C_GenerateStopCondition(i2c[i2c_num].base);
	wait_for_idle(i2c_num);
    printf("EE wr8\n\r");

	return len;
}
