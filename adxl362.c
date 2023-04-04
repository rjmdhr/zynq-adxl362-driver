/*
 * ADXL362 SPI driver source file
 *
 * Date: March 15, 2022
 * Author: Raj Mudhar
 *
 * This driver is currently using XSpiPs polled methods. Support for an
 * interrupt driven SPI controller will be added later.
 *
 *
 * Features not implemented:
 * - Burst writes (single register write only)
 * - Range configuration
 * - Antialiasing filter bandwitdh configuration (ODR*1/4 is maintained)
 * - External sampling trigger
 * - External clock
 * - Wakeup/autosleep modes
 * - Self test
 * - Interrupt features
 */

#include "adxl362.h"

/* Register bit update definition */
#define BIT_SWAP(new, old, mask) ((old & ~mask) | (new & mask))

/*
 * Initializes adxl362 structure
 * @dev_inst - pointer to adxl362 device structure instance
 * @spi_inst - pointer to SPI peripheral structure instance
 * @return - 0 if initialization successful, -1 if error
 */
int32_t adxl362_initialize(adxl362* dev_inst, XSpiPs* spi_inst)
{
	dev_inst->spi_inst = spi_inst;
	dev_inst->acc[0] = 0.0f;
	dev_inst->acc[1] = 0.0f;
	dev_inst->acc[2] = 0.0f;
	dev_inst->tilt[0] = 0.0f;
	dev_inst->tilt[1] = 0.0f;
	dev_inst->tilt[2] = 0.0f;
	dev_inst->temp = 0.0f;
	dev_inst->x_offset = 0;
	dev_inst->y_offset = 0;
	dev_inst->z_offset = 0;
	int32_t status = 0;

	uint8_t reg_val;

	// check device IDs
	adxl362_read_reg(dev_inst, ADXL362_REG_DEVID_AD, &reg_val);
	if (reg_val != ADXL362_DEVICE_ID)
		status = -1;

	adxl362_read_reg(dev_inst, ADXL362_REG_DEVID_MST, &reg_val);
	if (reg_val != ADXL362_MEMS_ID)
		status = -1;

	adxl362_read_reg(dev_inst, ADXL362_REG_PARTID, &reg_val);
	if (reg_val != ADXL362_PART_ID)
		status = -1;

	return status;
}

/*
 * Sets offset values for stationary calibration (mg)
 * @dev_inst - pointer to adxl362 device structure instance
 * @x_offset - x axis offset (mg)
 * @y_offset - y axis offset (mg)
 * @z_offset - z axis offset (mg)
 */
void adxl362_set_offset(adxl362* dev_inst, int16_t x_offset, int16_t y_offset,
						int16_t z_offset)
{
	dev_inst->x_offset = x_offset;
	dev_inst->y_offset = y_offset;
	dev_inst->z_offset = z_offset;
}

/*
 * Reads raw 12-bit x/y/z axis register data
 */
void adxl362_read_raw_acc(adxl362* dev_inst, int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t buffer[6];

	adxl362_burst_read(dev_inst, ADXL362_REG_XDATA_L, buffer, 6);
	*x = (((int16_t)(buffer[1])) << 8) + (int16_t)buffer[0] - dev_inst->x_offset;
	*y = (((int16_t)(buffer[3])) << 8) + (int16_t)buffer[2] - dev_inst->y_offset;
	*z = (((int16_t)(buffer[5])) << 8) + (int16_t)buffer[4] - dev_inst->z_offset;
}

/*
 * Reads x/y/z axis data and converts it to g (m/s^2) and updates structure
 * instance
 */
void adxl362_read_g(adxl362* dev_inst)
{
	uint8_t buffer[6];
	adxl362_burst_read(dev_inst, ADXL362_REG_XDATA_L, buffer, 6);

	dev_inst->acc[0] = (int16_t)((((int16_t)(buffer[1])) << 8) + buffer[0] \
					    - dev_inst->x_offset)/1000.0;
	dev_inst->acc[1] = (int16_t)((((int16_t)(buffer[3])) << 8) + buffer[2] \
						- dev_inst->y_offset)/1000.0;
	dev_inst->acc[2] = (int16_t)((((int16_t)(buffer[5])) << 8) + buffer[4] \
						- dev_inst->z_offset)/1000.0;
}

/*
 * Reads x/y/z axis data, converts it to angular tilt, and updates structure
 * instance
 */
void adxl362_get_tilt(adxl362* dev_inst)
{}

/*
 * Reads temperature register and updates structure instance
 */
void adxl362_read_temp(adxl362* dev_inst)
{}

/*
 * Resets the device and clears all register settings
 * Latency of 500uS is required after reset
 * @dev_inst - pointer to adxl362 device structure instance
 */
void adxl362_reset(adxl362* dev_inst)
{
	adxl362_write_reg(dev_inst, ADXL362_REG_SOFT_RESET, ADXL362_RESET_CODE);
}

/*
 * Sets the power measurement mode. The device must be in standby mode when
 * changing register settings
 * @dev_inst - pointer to adxl362 device structure instance
 * @mode - 0 standby mode (not measuring)
 * 		 - 2 measurement mode
 */
void adxl362_set_power_mode(adxl362* dev_inst, uint8_t mode)
{
	uint8_t reg_val;
	adxl362_read_reg(dev_inst, ADXL362_REG_POWER_CTL, &reg_val);
	reg_val = BIT_SWAP(mode, reg_val, ADXL362_POWER_CTL_MEASURE);
	adxl362_write_reg(dev_inst, ADXL362_REG_POWER_CTL, reg_val);
}

/*
 * Sets the noise mode
 * @dev_inst - pointer to adxl362 device structure instance
 * @mode - 0 normal operation
 * 		 - 1 low noise
 * 		 - 2 ultra low noise
 */
void adxl362_set_noise_mode(adxl362* dev_inst, uint8_t mode)
{
	uint8_t reg_val;
	adxl362_read_reg(dev_inst, ADXL362_REG_POWER_CTL, &reg_val);
	reg_val = BIT_SWAP(mode << 4, reg_val, ADXL362_POWER_CTL_LOW_NOISE);
	adxl362_write_reg(dev_inst, ADXL362_REG_POWER_CTL, reg_val);
}

/*
 * Sets the output data sampling rate
 * @dev_inst - pointer to adxl362 device structure instance
 * @data_rate - 0 for 12.5 Hz
 * 			  - 1 for 25 Hz
 * 			  - 2 for 50 Hz
 * 			  - 3 for 100 Hz (default on reset)
 * 			  - 4 for 200 Hz
 * 			  - 5 for 400 Hz
 */
void adxl362_set_ODR(adxl362* dev_inst, uint8_t data_rate)
{
	uint8_t reg_val;
	adxl362_read_reg(dev_inst, ADXL362_REG_FILTER_CTL, &reg_val);
	reg_val = BIT_SWAP(data_rate, reg_val, ADXL362_FILTER_CTL_ODR);
	adxl362_write_reg(dev_inst, ADXL362_REG_FILTER_CTL, reg_val);
}

/*
 * Configures fifo with desired settings
 * @dev_inst - pointer to adxl362 device structure instance
 * @fifo_mode - 0 disable fifo
 * 			  - 1 oldest save mode
 * 			  - 2 stream mode
 * 			  - 3 triggered mode
 * @wtr_mrk - number of samples to store in fifo (0-511)
 * @temp_en	- 1 to enable temperature storage in fifo with xyz data
 * 			- 0 to disable
 *
 */
void adxl362_config_fifo(adxl362* dev_inst, uint8_t fifo_mode, uint16_t wrt_mrk,
						 uint8_t temp_en, uint8_t num_samples)
{

}

/*
 * Reads register data
 * @dev_inst - pointer to adxl362 device structure instance
 * @addr - address of register to read
 * @rd_data - pointer to data to be read
 */
void adxl362_read_reg(adxl362* dev_inst, uint8_t addr, uint8_t* rd_data)
{
	uint8_t send_buf[3] = {ADXL362_READ_REG, addr, 0};
	uint8_t recv_buf[3] = {0};

	XSpiPs_SetSlaveSelect(dev_inst->spi_inst, 0x00u);
	XSpiPs_PolledTransfer(dev_inst->spi_inst, send_buf, recv_buf, 3);

	*rd_data = recv_buf[2];
}

/*
 * Burst read multiple register data addresses
 * Maximum rd_data size is 32 (no dynamic memory allocation)
 * @dev_inst - pointer to adxl362 device structure instance
 * @addr - address of register to read first
 * @rd_data - pointer to array to copy read data to
 * @num_bytes - number of bytes (registers) to read
 */
void adxl362_burst_read(adxl362* dev_inst, uint8_t addr, uint8_t* rd_data,
					    uint8_t num_bytes)
{
	uint8_t buffer[34] = {0};
	buffer[0] = ADXL362_READ_REG;
	buffer[1] = addr;

	XSpiPs_SetSlaveSelect(dev_inst->spi_inst, 0x00u);
	XSpiPs_PolledTransfer(dev_inst->spi_inst, buffer, buffer, 2+num_bytes);

	for (int i = 0; i < num_bytes; i++) {
		*(rd_data+i) = buffer[i+2];
	}
}

/*
 * Writes data to device register
 * @dev_inst - pointer to adxl362 device structure instance
 * @addr - address of register to write to
 * @wr_data - pointer to data to write
 */
void adxl362_write_reg(adxl362* dev_inst, uint8_t addr, uint8_t wr_data)
{
	uint8_t send_buf[3] = {ADXL362_WRITE_REG, addr, wr_data};

	XSpiPs_SetSlaveSelect(dev_inst->spi_inst, 0x00u);
	XSpiPs_PolledTransfer(dev_inst->spi_inst, send_buf, NULL, 3);
}









