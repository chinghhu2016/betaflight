/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_IST8310

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_ist8310.h"

//#define DEBUG_MAG_DATA_READY_INTERRUPT

#define IST8310_MAG_I2C_ADDRESS 0x0C

/* ist8310 Slave Address Select : default address 0x0C
 *        CAD1  |  CAD0   |  Address
 *    ------------------------------
 *         VSS   |   VSS  |  0CH
 *         VSS   |   VDD  |  0DH
 *         VDD   |   VSS  |  0EH
 *         VDD   |   VDD  |  0FH
 * if CAD1 and CAD0 are floating, I2C address will be 0EH
 *
 *
 * CTRL_REGA: Control Register 1
 * Read Write
 * Default value: 0x0A
 * 7:4  0   Reserved.
 * 3:0  DO2-DO0: Operating mode setting
 *        DO3  |  DO2 |  DO1 |  DO0 |   mode
 *    ------------------------------------------------------
 *         0   |   0  |  0   |  0   |   Stand-By mode
 *         0   |   0  |  0   |  1   |   Single measurement mode
 *                                       Others: Reserved
 *
 * CTRL_REGB: Control Register 2
 * Read Write
 * Default value: 0x0B
 * 7:4  0   Reserved.
 * 3    DREN : Data ready enable control:
 *      0: disable
 *      1: enable
 * 2    DRP: DRDY pin polarity control
 *      0: active low
 *      1: active high
 * 1    0   Reserved.
 * 0    SRST: Soft reset, perform Power On Reset (POR) routine
 *      0: no action
 *      1: start immediately POR routine
 *      This bit will be set to zero after POR routine
 */

#define IST8310_REG_DATA 0x03
#define IST8310_REG_WAI 0x00
#define IST8310_REG_WAI_VALID 0x10
#define IST8310J_REG_WAI_VALID 0xA3

#define IST8310_REG_STAT1 0x02
#define IST8310_REG_STAT2 0x09

#define IST8310_DRDY_MASK 0x01

// I2C Control Register
#define IST8310_REG_CNTRL1 0x0A
#define IST8310_REG_CNTRL2 0x0B
#define IST8310_REG_AVERAGE 0x41
#define IST8310_REG_PDCNTL 0x42

#define IST8310_REG_XX_CROSS_L 0x9C // cross axis xx low byte
#define IST8310_REG_XX_CROSS_H 0x9D // cross axis xx high byte
#define IST8310_REG_XY_CROSS_L 0x9E // cross axis xy low byte
#define IST8310_REG_XY_CROSS_H 0x9F // cross axis xy high byte
#define IST8310_REG_XZ_CROSS_L 0xA0 // cross axis xz low byte
#define IST8310_REG_XZ_CROSS_H 0xA1 // cross axis xz high byte
#define IST8310_REG_YX_CROSS_L 0xA2 // cross axis yx low byte
#define IST8310_REG_YX_CROSS_H 0xA3 // cross axis yx high byte
#define IST8310_REG_YY_CROSS_L 0xA4 // cross axis yy low byte
#define IST8310_REG_YY_CROSS_H 0xA5 // cross axis yy high byte
#define IST8310_REG_YZ_CROSS_L 0xA6 // cross axis yz low byte
#define IST8310_REG_YZ_CROSS_H 0xA7 // cross axis yz high byte
#define IST8310_REG_ZX_CROSS_L 0xA8 // cross axis zx low byte
#define IST8310_REG_ZX_CROSS_H 0xA9 // cross axis zx high byte
#define IST8310_REG_ZY_CROSS_L 0xAA // cross axis zy low byte
#define IST8310_REG_ZY_CROSS_H 0xAB // cross axis zy high byte
#define IST8310_REG_ZZ_CROSS_L 0xAC // cross axis zz low byte
#define IST8310_REG_ZZ_CROSS_H 0xAD // cross axis zz high byte

// Parameter
// ODR = Output Data Rate, we use single measure mode for getting more data.
#define IST8310_ODR_SINGLE 0x01
#define IST8310_ODR_10_HZ 0x03
#define IST8310_ODR_20_HZ 0x05
#define IST8310_ODR_50_HZ 0x07
#define IST8310_ODR_100_HZ 0x06

#define IST8310_AVG_16  0x24
#define IST8310_PULSE_DURATION_NORMAL 0xC0

#define IST8310_CNTRL2_RESET 0x01
#define IST8310_CNTRL2_DRPOL 0x04
#define IST8310_CNTRL2_DRENA 0x08

#define IST8310_AXES_NUM 3
#define OTP_SENSITIVITY 330
#define CROSSAXIS_INV_BITSHIFT 16

static bool _crossaxis_enabled = false;
static int64_t _crossaxis_inv[9] = {0};
static int32_t _crossaxis_det = 0;

#define combine(msb, lsb) ((int16_t)(((uint16_t)(msb) << 8) | (lsb)))

static bool initCrossAxisMatrix(extDevice_t *dev)
{
	// Read cross-axis matrix data
	uint8_t crossx_buf[6], crossy_buf[6], crossz_buf[6];
    uint8_t ack;

    ack = busReadRegisterBufferStart(dev, IST8310_REG_XX_CROSS_L, crossx_buf, sizeof(crossx_buf));
    if (!ack) {
		//PX4_ERR("Failed to read X cross-axis data");
		return false;
	}
    _crossaxis_enabled = !((crossx_buf[0] == 0xFF) && (crossx_buf[1] == 0xFF));
    if (!_crossaxis_enabled) {
        // Set identity matrix
        _crossaxis_inv[0] = (1 << CROSSAXIS_INV_BITSHIFT);
        _crossaxis_inv[1] = 0;
        _crossaxis_inv[2] = 0;
        _crossaxis_inv[3] = 0;
        _crossaxis_inv[4] = (1 << CROSSAXIS_INV_BITSHIFT);
        _crossaxis_inv[5] = 0;
        _crossaxis_inv[6] = 0;
        _crossaxis_inv[7] = 0;
        _crossaxis_inv[8] = (1 << CROSSAXIS_INV_BITSHIFT);
        _crossaxis_det = 1;
        return true;
    }

    ack = busReadRegisterBufferStart(dev, IST8310_REG_YX_CROSS_L, crossy_buf, sizeof(crossy_buf));
    if (!ack) {
		//PX4_ERR("Failed to read Y cross-axis data");
		return false;
	}

    ack = busReadRegisterBufferStart(dev, IST8310_REG_ZX_CROSS_L, crossz_buf, sizeof(crossz_buf));
    if (!ack) {
		//PX4_ERR("Failed to read Z cross-axis data");
		return false;
	}

	// Parse cross-axis matrix based on OTP data format
	int16_t otp_crossaxis[9];

	otp_crossaxis[0] = combine(crossx_buf[1], crossx_buf[0]);
	otp_crossaxis[3] = combine(crossx_buf[3], crossx_buf[2]);
	otp_crossaxis[6] = combine(crossx_buf[5], crossx_buf[4]);
	otp_crossaxis[1] = combine(crossy_buf[1], crossy_buf[0]);
	otp_crossaxis[4] = combine(crossy_buf[3], crossy_buf[2]);
	otp_crossaxis[7] = combine(crossy_buf[5], crossy_buf[4]);
	otp_crossaxis[2] = combine(crossz_buf[1], crossz_buf[0]);
	otp_crossaxis[5] = combine(crossz_buf[3], crossz_buf[2]);
	otp_crossaxis[8] = combine(crossz_buf[5], crossz_buf[4]);
	//PX4_DEBUG("Cross-axis matrix from OTP:");
	//PX4_DEBUG("[[%d, %d, %d],", otp_crossaxis[0], otp_crossaxis[1], otp_crossaxis[2]);
	//PX4_DEBUG(" [%d, %d, %d],", otp_crossaxis[3], otp_crossaxis[4], otp_crossaxis[5]);
	//PX4_DEBUG(" [%d, %d, %d]]", otp_crossaxis[6], otp_crossaxis[7], otp_crossaxis[8]);

	// Calculate matrix determinant
	_crossaxis_det = ((int32_t)otp_crossaxis[0]) * otp_crossaxis[4] * otp_crossaxis[8] +
			 ((int32_t)otp_crossaxis[1]) * otp_crossaxis[5] * otp_crossaxis[6] +
			 ((int32_t)otp_crossaxis[2]) * otp_crossaxis[3] * otp_crossaxis[7] -
			 ((int32_t)otp_crossaxis[0]) * otp_crossaxis[5] * otp_crossaxis[7] -
			 ((int32_t)otp_crossaxis[2]) * otp_crossaxis[4] * otp_crossaxis[6] -
			 ((int32_t)otp_crossaxis[1]) * otp_crossaxis[3] * otp_crossaxis[8];

	//PX4_DEBUG("Cross-axis matrix determinant: %d", _crossaxis_det);

	if (_crossaxis_det == 0) {
		//PX4_WARN("Cross-axis determinant is zero, using identity matrix");
		_crossaxis_enabled = false;
		// Directly set identity matrix instead of recursive call
		_crossaxis_inv[0] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_inv[1] = 0;
		_crossaxis_inv[2] = 0;
		_crossaxis_inv[3] = 0;
		_crossaxis_inv[4] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_inv[5] = 0;
		_crossaxis_inv[6] = 0;
		_crossaxis_inv[7] = 0;
		_crossaxis_inv[8] = (1 << CROSSAXIS_INV_BITSHIFT);
		_crossaxis_det = 1;
		return true;
	}

	// Calculate inverse matrix
	int64_t inv[9];
	inv[0] = (int64_t)otp_crossaxis[4] * otp_crossaxis[8] - (int64_t)otp_crossaxis[5] * otp_crossaxis[7];
	inv[1] = (int64_t)otp_crossaxis[2] * otp_crossaxis[7] - (int64_t)otp_crossaxis[1] * otp_crossaxis[8];
	inv[2] = (int64_t)otp_crossaxis[1] * otp_crossaxis[5] - (int64_t)otp_crossaxis[2] * otp_crossaxis[4];
	inv[3] = (int64_t)otp_crossaxis[5] * otp_crossaxis[6] - (int64_t)otp_crossaxis[3] * otp_crossaxis[8];
	inv[4] = (int64_t)otp_crossaxis[0] * otp_crossaxis[8] - (int64_t)otp_crossaxis[2] * otp_crossaxis[6];
	inv[5] = (int64_t)otp_crossaxis[2] * otp_crossaxis[3] - (int64_t)otp_crossaxis[0] * otp_crossaxis[5];
	inv[6] = (int64_t)otp_crossaxis[3] * otp_crossaxis[7] - (int64_t)otp_crossaxis[4] * otp_crossaxis[6];
	inv[7] = (int64_t)otp_crossaxis[1] * otp_crossaxis[6] - (int64_t)otp_crossaxis[0] * otp_crossaxis[7];
	inv[8] = (int64_t)otp_crossaxis[0] * otp_crossaxis[4] - (int64_t)otp_crossaxis[1] * otp_crossaxis[3];

	for (int i = 0; i < 9; i++) {
		_crossaxis_inv[i] = (inv[i] << CROSSAXIS_INV_BITSHIFT) * OTP_SENSITIVITY;
	}

	//PX4_DEBUG("Inverse cross-axis matrix:");
	//PX4_DEBUG("[[%lld, %lld, %lld],", _crossaxis_inv[0], _crossaxis_inv[1], _crossaxis_inv[2]);
	//PX4_DEBUG(" [%lld, %lld, %lld],", _crossaxis_inv[3], _crossaxis_inv[4], _crossaxis_inv[5]);
	//PX4_DEBUG(" [%lld, %lld, %lld]]", _crossaxis_inv[6], _crossaxis_inv[7], _crossaxis_inv[8]);

	//PX4_INFO("Cross-axis calibration initialized successfully");
	return true;
}

static void crossAxisTransformation(extDevice_t *dev, int16_t *xyz)
{
	if (!_crossaxis_enabled) {
		return;
	}

	// Check if crossaxis matrix is initialized
	bool matrix_initialized = false;

	for (int i = 0; i < 9; i++) {
		if (_crossaxis_inv[i] != 0) {
			matrix_initialized = true;
			break;
		}
	}

	if (!matrix_initialized) {
		//PX4_WARN("Cross-axis matrix not initialized, reinitializing");
		initCrossAxisMatrix(dev);
		return;
	}

	// Apply cross-axis transformation
	int64_t output_tmp[3];

	output_tmp[0] = (int64_t)xyz[0] * _crossaxis_inv[0] +
			(int64_t)xyz[1] * _crossaxis_inv[1] +
			(int64_t)xyz[2] * _crossaxis_inv[2];

	output_tmp[1] = (int64_t)xyz[0] * _crossaxis_inv[3] +
			(int64_t)xyz[1] * _crossaxis_inv[4] +
			(int64_t)xyz[2] * _crossaxis_inv[5];

	output_tmp[2] = (int64_t)xyz[0] * _crossaxis_inv[6] +
			(int64_t)xyz[1] * _crossaxis_inv[7] +
			(int64_t)xyz[2] * _crossaxis_inv[8];

	// Apply determinant division and bit shift
	for (int i = 0; i < IST8310_AXES_NUM; i++) {
		output_tmp[i] = output_tmp[i] / _crossaxis_det;
	}

	xyz[0] = (int16_t)(output_tmp[0] >> CROSSAXIS_INV_BITSHIFT);
	xyz[1] = (int16_t)(output_tmp[1] >> CROSSAXIS_INV_BITSHIFT);
	xyz[2] = (int16_t)(output_tmp[2] >> CROSSAXIS_INV_BITSHIFT);
}

static bool ist8310Init(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    busDeviceRegister(dev);

    // Init setting
    bool ack = busWriteRegister(dev, IST8310_REG_AVERAGE, IST8310_AVG_16);
    delay(6);
    ack = ack && busWriteRegister(dev, IST8310_REG_PDCNTL, IST8310_PULSE_DURATION_NORMAL);
    delay(6);
    ack = ack && busWriteRegister(dev, IST8310_REG_CNTRL1, IST8310_ODR_SINGLE);

    magDev->magOdrHz = 100;
    // need to check what ODR is actually returned, may be a bit faster than 100Hz
    return ack;
}

static bool ist8310Read(magDev_t * magDev, int16_t *magData)
{
    extDevice_t *dev = &magDev->dev;

    static uint8_t buf[6];
    const int LSB2FSV = 3; // 3mG - 14 bit

    static enum {
        STATE_REQUEST_DATA,
        STATE_FETCH_DATA,
    } state = STATE_REQUEST_DATA;

    switch (state) {
        default:
        case STATE_REQUEST_DATA:
            if (busReadRegisterBufferStart(dev, IST8310_REG_DATA, buf, sizeof(buf))) {
                state = STATE_FETCH_DATA;
            }

            return false;
        case STATE_FETCH_DATA:
            // Looks like datasheet is incorrect and we need to invert Y axis to conform to right hand rule
            magData[X] =  (int16_t)(buf[1] << 8 | buf[0]) * LSB2FSV;
            magData[Y] = -(int16_t)(buf[3] << 8 | buf[2]) * LSB2FSV;
            magData[Z] =  (int16_t)(buf[5] << 8 | buf[4]) * LSB2FSV;

            int16_t xyz[3] = {magData[X], magData[Y], magData[Z]};
            crossAxisTransformation(dev, xyz);
            magData[X] = xyz[0];
            magData[Y] = xyz[1];
            magData[Z] = xyz[2];

            // Force single measurement mode for next read
            if (busWriteRegisterStart(dev, IST8310_REG_CNTRL1, IST8310_ODR_SINGLE)) {
                state = STATE_REQUEST_DATA;

                return true;
            }
            return false;
    }
    return false;
}

static bool deviceDetect(magDev_t * magDev)
{
    uint8_t result = 0;
    bool ack = busReadRegisterBuffer(&magDev->dev, IST8310_REG_WAI, &result, 1);

    return ack && (result == IST8310_REG_WAI_VALID || result == IST8310J_REG_WAI_VALID);
}

bool ist8310Detect(magDev_t * magDev)
{
    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = IST8310_MAG_I2C_ADDRESS;
    }

    if (deviceDetect(magDev)) {
        magDev->init = ist8310Init;
        magDev->read = ist8310Read;

        return true;
    }

    return false;
}

#endif
