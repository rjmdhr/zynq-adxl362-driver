/**
 * ADXL362 SPI driver header file
 *
 * Date: March 15, 2022
 * Author: Raj Mudhar
 *
 */

#ifndef ADXL362_SPI_DRIVER_H
#define ADXL362_SPI_DRIVER_H

#include "xspips.h"

/* ADXL362 Device Information */
#define ADXL362_DEVICE_ID				0xAD
#define ADXL362_MEMS_ID					0x1D
#define ADXL362_PART_ID					0xF2

/* SPI Commands */
#define ADXL362_WRITE_REG				0x0A
#define ADXL362_READ_REG				0x0B
#define ADXL362_READ_FIFO				0x0D

/* Registers */
#define ADXL362_REG_DEVID_AD			0x00
#define ADXL362_REG_DEVID_MST			0x01
#define ADXL362_REG_PARTID				0x02
#define ADXL362_REG_REVID				0x03
#define ADXL362_REG_XDATA				0x08
#define ADXL362_REG_YDATA				0x09
#define ADXL362_REG_ZDATA				0x0A
#define ADXL362_REG_STATUS				0x0B
#define ADXL362_REG_FIFO_L				0x0C
#define ADXL362_REG_FIFO_H				0x0D
#define ADXL362_REG_XDATA_L				0x0E
#define ADXL362_REG_XDATA_H				0x0F
#define ADXL362_REG_YDATA_L				0x10
#define ADXL362_REG_YDATA_H				0x11
#define ADXL362_REG_ZDATA_L				0x12
#define ADXL362_REG_ZDATA_H				0x13
#define ADXL362_REG_TEMP_L				0x14
#define ADXL362_REG_TEMP_H				0x15
#define ADXL362_REG_SOFT_RESET			0x1F
#define ADXL362_REG_THRESH_ACT_L		0x20
#define ADXL362_REG_THRESH_ACT_H    	0x21
#define ADXL362_REG_TIME_ACT        	0x22
#define ADXL362_REG_THRESH_INACT_L  	0x23
#define ADXL362_REG_THRESH_INACT_H		0x24
#define ADXL362_REG_TIME_INACT_L    	0x25
#define ADXL362_REG_TIME_INACT_H    	0x26
#define ADXL362_REG_ACT_INACT_CTL   	0x27
#define ADXL362_REG_FIFO_CTL        	0x28
#define ADXL362_REG_FIFO_SAMPLES    	0x29
#define ADXL362_REG_INTMAP1         	0x2A
#define ADXL362_REG_INTMAP2         	0x2B
#define ADXL362_REG_FILTER_CTL     	 	0x2C
#define ADXL362_REG_POWER_CTL       	0x2D
#define ADXL362_REG_SELF_TEST       	0x2E

/* Soft reset */
#define ADXL362_RESET_CODE				0x52

/* Filter control register masks */
#define	ADXL362_FILTER_CTL_RANGE  	  	(3 << 6)
#define ADXL362_FILTER_CTL_HALF_BW	  	(1 << 4)
#define	ADXL362_FILTER_CTL_EXT_SAMPLE	(1 << 3)
#define	ADXL362_FILTER_CTL_ODR			(3 << 0)

/* Filter control register settings */
//#define ADXL362_FILTER_CTL_RANGE_EN

/* Power control register masks */
#define	ADXL362_POWER_CTL_EXT_CLK		(1 << 6)
#define ADXL362_POWER_CTL_LOW_NOISE		(3 << 4)
#define	ADXL362_POWER_CTL_WAKEUP		(1 << 3)
#define	ADXL362_POWER_CTL_AUTOSLEEP		(1 << 2)
#define	ADXL362_POWER_CTL_MEASURE		(2 << 0)

/* Approximate datasheet offset calibration for 3V3 */
#define ADXL362_X_OFFSET				-37
#define ADXL362_Y_OFFSET				 21
#define ADXL362_Z_OFFSET				 155

/*
 * ADXL362 device structure
 * @spi_instance - pointer to the SPI instance
 * @acc - acceleration data (x,y,z) in m/s^2
 * @temp - temperature in degrees (C)
 * @x/y/z_offset - offset values for calibration
 */
typedef struct
{
	XSpiPs* spi_inst;
	float acc[3];
	float tilt[3];
	float temp;
	int16_t x_offset;
	int16_t y_offset;
	int16_t z_offset;
} adxl362;

/* Main utilization functions */
int32_t adxl362_initialize(adxl362* dev_inst, XSpiPs* spi_inst);
void adxl362_set_offset(adxl362* dev_inst, int16_t x_os, int16_t y_os, int16_t z_os);
void adxl362_read_raw_acc(adxl362* dev_inst, int16_t* x, int16_t* y, int16_t* z);
void adxl362_read_g(adxl362* dev_inst);
void adxl362_read_temp(adxl362* dev_inst);

/* Configuration functions */
void adxl362_reset(adxl362* dev_inst);
void adxl362_set_power_mode(adxl362* dev_inst, uint8_t mode);
void adxl362_set_noise_mode(adxl362* dev_inst, uint8_t mode);
void adxl362_set_ODR(adxl362* dev_inst, uint8_t data_rate);
void adxl362_config_fifo(adxl362* dev_inst, uint8_t fifo_mode, uint16_t wrt_mrk,
						 uint8_t temp_en, uint8_t num_samples);

/* Device register access functions */
void adxl362_read_reg(adxl362* dev_inst, uint8_t addr, uint8_t* rd_data);
void adxl362_burst_read(adxl362* dev_inst, uint8_t addr, uint8_t* rd_data, uint8_t num_bytes);
void adxl362_write_reg(adxl362* dev_inst, uint8_t addr, uint8_t wr_data);

#endif
