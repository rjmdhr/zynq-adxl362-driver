/**
 * main.c
 */

#include <stdio.h>
#include "adxl362.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "platform.h"
#include "sleep.h"

/*****************************************************************************/

/* SPI register control addresses */
#define SPI_RST_CTRL 	 	0xF800021CU
#define SPI_CLK_CTRL 	 	0xF8000158U
#define MIO_PIN_10	 	 	0xF8000728
#define MIO_PIN_11	 	 	0xF800072C
#define MIO_PIN_12	 		0xF8000730
#define MIO_PIN_13	 	 	0xF8000734
#define XSPIPS1_CR_OFFSET 	0xE0007000

/* Register access macros */
#define WRITE_REG(addr, val) (*((volatile uint32_t*)(addr)) = (val))
#define READ_REG(addr) 		 (*(volatile uint32_t*)(addr))

/* Function definitions */
int32_t init_spi(uint8_t device_id, XSpiPs* spi_instance, uint32_t options,
			 uint16_t clock_prescaler);


int main(void)
{
	init_platform();

	/*
	 * Configure SPI 1 peripheral
	 * Options: Master, manual CS, CPOL = 0, CPHA = 0
	 * 7.8 MHz clock
	 */
	XSpiPs spi_inst;
	uint32_t options = (XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION);
	init_spi(XPAR_PS7_SPI_1_DEVICE_ID, &spi_inst, options, XSPIPS_CLK_PRESCALE_16);

	adxl362 adx_inst;
	int32_t status = adxl362_initialize(&adx_inst, &spi_inst);
	if (status == -1)
		return 0;

	// reset registers and place device in standby
	adxl362_reset(&adx_inst);
	usleep(10000);

	// set to ultra low noise and measurement mode
	adxl362_set_noise_mode(&adx_inst, 2);
	adxl362_set_power_mode(&adx_inst, 2);
	adxl362_set_offset(&adx_inst, ADXL362_X_OFFSET, ADXL362_Y_OFFSET, ADXL362_Z_OFFSET);

	while (1) {
		adxl362_read_g(&adx_inst);
		printf("x: %.2f, y: %.2f, z: %.2f \n",
				adx_inst.acc[0],
				adx_inst.acc[1],
				adx_inst.acc[2]);
		usleep(100000);
	}

	return 0;
}

/**
 * Configures and initializes SPI peripheral
 * @ device_id is the peripheral ID defined in xparameters.h
 * @ spi_instance is pointer to the XSpiPs instance
 * @ options is the configuration options defined in xspips.h
 * @ clock_prescaler is the SPI reference clock divider
 */
int32_t init_spi(uint8_t device_id, XSpiPs* spi_instance, uint32_t options,
		 	 	 uint16_t clock_prescaler)
{
	int32_t status;
	XSpiPs_Config* spi_config;

	spi_config = XSpiPs_LookupConfig(device_id);
	status = XSpiPs_CfgInitialize(spi_instance, spi_config,
								  spi_config->BaseAddress);

	if ((spi_config == NULL) || (status != (int32_t)XST_SUCCESS))
		return (int32_t)XST_FAILURE;

	XSpiPs_SetOptions(spi_instance, options);
	XSpiPs_SetClkPrescaler(spi_instance, clock_prescaler);

	return (int32_t)XST_SUCCESS;
}















