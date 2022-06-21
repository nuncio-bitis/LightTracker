/*
 * This file is part of the LightTracker distribution
 *   (https://github.com/nuncio-bitis/LightTracker
 * Copyright (c) 2022 James P. Parziale.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * ADS1115.h
 *
 * ADS1115 I2C device class
 *  Note that the ADS1115 uses 16-bit registers, not 8-bit registers.
 * 
 *  Created on: March 1, 2020
 *      Author: jparziale
 */

#ifndef _ADS1115_H_
#define _ADS1115_H_
//******************************************************************************

#include "I2Cdev.h"

// -----------------------------------------------------------------------------
// Arduino-style "Serial.print" debug constant (uncomment to enable)
// -----------------------------------------------------------------------------
//#define ADS1115_SERIAL_DEBUG
#ifdef ADS1115_SERIAL_DEBUG
	#include "Serial.h"
#endif // ADS1115_SERIAL_DEBUG

// -----------------------------------------------------------------------------

#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin
#define ADS1115_DEFAULT_ADDRESS     ADS1115_ADDRESS_ADDR_GND

#define ADS1115_RA_CONVERSION       0x00
#define ADS1115_RA_CONFIG           0x01
#define ADS1115_RA_LO_THRESH        0x02
#define ADS1115_RA_HI_THRESH        0x03

#define ADS1115_CFG_OS_BIT          15
#define ADS1115_CFG_MUX_BIT         14
#define ADS1115_CFG_MUX_LENGTH      3
#define ADS1115_CFG_PGA_BIT         11
#define ADS1115_CFG_PGA_LENGTH      3
#define ADS1115_CFG_MODE_BIT        8
#define ADS1115_CFG_DR_BIT          7
#define ADS1115_CFG_DR_LENGTH       3
#define ADS1115_CFG_COMP_MODE_BIT   4
#define ADS1115_CFG_COMP_POL_BIT    3
#define ADS1115_CFG_COMP_LAT_BIT    2
#define ADS1115_CFG_COMP_QUE_BIT    1
#define ADS1115_CFG_COMP_QUE_LENGTH 2


#define ADS1115_MUX_P0_N1           0x00 // default
#define ADS1115_MUX_P0_N3           0x01
#define ADS1115_MUX_P1_N3           0x02
#define ADS1115_MUX_P2_N3           0x03
#define ADS1115_MUX_P0_NG           0x04
#define ADS1115_MUX_P1_NG           0x05
#define ADS1115_MUX_P2_NG           0x06
#define ADS1115_MUX_P3_NG           0x07

#define ADS1115_PGA_6P144           0x00
#define ADS1115_PGA_4P096           0x01
#define ADS1115_PGA_2P048           0x02 // default
#define ADS1115_PGA_1P024           0x03
#define ADS1115_PGA_0P512           0x04
#define ADS1115_PGA_0P256           0x05
#define ADS1115_PGA_0P256B          0x06
#define ADS1115_PGA_0P256C          0x07

// MV = millivolt, corresponding to gains (PGA) above
// Millivolts = 1000 * gain / 32768
#define ADS1115_MV_6P144            0.187500
#define ADS1115_MV_4P096            0.125000
#define ADS1115_MV_2P048            0.062500 // default
#define ADS1115_MV_1P024            0.031250
#define ADS1115_MV_0P512            0.015625
#define ADS1115_MV_0P256            0.007813
#define ADS1115_MV_0P256B           0.007813
#define ADS1115_MV_0P256C           0.007813

#define ADS1115_MODE_CONTINUOUS     0x00
#define ADS1115_MODE_SINGLESHOT     0x01 // default

#define ADS1115_RATE_8              0x00
#define ADS1115_RATE_16             0x01
#define ADS1115_RATE_32             0x02
#define ADS1115_RATE_64             0x03
#define ADS1115_RATE_128            0x04 // default
#define ADS1115_RATE_250            0x05
#define ADS1115_RATE_475            0x06
#define ADS1115_RATE_860            0x07

#define ADS1115_COMP_MODE_HYSTERESIS    0x00 // default
#define ADS1115_COMP_MODE_WINDOW        0x01

#define ADS1115_COMP_POL_ACTIVE_LOW     0x00 // default
#define ADS1115_COMP_POL_ACTIVE_HIGH    0x01

#define ADS1115_COMP_LAT_NON_LATCHING   0x00 // default
#define ADS1115_COMP_LAT_LATCHING       0x01

#define ADS1115_COMP_QUE_ASSERT1    0x00
#define ADS1115_COMP_QUE_ASSERT2    0x01
#define ADS1115_COMP_QUE_ASSERT4    0x02
#define ADS1115_COMP_QUE_DISABLE    0x03 // default

// -----------------------------------------------------------------------------
// Arduino-style "Serial.print" debug constant (uncomment to enable)
// -----------------------------------------------------------------------------
//#define ADS1115_SERIAL_DEBUG


class ADS1115 {
public:
	ADS1115();
	ADS1115(uint8_t address);
	~ADS1115();

	void initialize();
	bool testConnection();

	// SINGLE SHOT utilities
	bool pollConversion(uint16_t max_retries);
	void triggerConversion();

	// Read the current CONVERSION register
	int16_t getConversion(bool triggerAndPoll=true);

	// Differential
	int16_t getConversionP0N1();
	int16_t getConversionP0N3();
	int16_t getConversionP1N3();
	int16_t getConversionP2N3();

	// Single-ended
	int16_t getConversionP0GND();
	int16_t getConversionP1GND();
	int16_t getConversionP2GND();
	int16_t getConversionP3GND();

	// Utility
	float getMilliVolts(bool triggerAndPoll=true);
	float getMvPerCount();

	// CONFIG register
	bool    isConversionReady();
	uint8_t getMultiplexer();
	void    setMultiplexer(uint8_t mux);
	uint8_t getGain();
	void    setGain(uint8_t gain);
	bool    getMode();
	void    setMode(bool mode);
	uint8_t getRate();
	void    setRate(uint8_t rate);
	bool    getComparatorMode();
	void    setComparatorMode(bool mode);
	bool    getComparatorPolarity();
	void    setComparatorPolarity(bool polarity);
	bool    getComparatorLatchEnabled();
	void    setComparatorLatchEnabled(bool enabled);
	uint8_t getComparatorQueueMode();
	void    setComparatorQueueMode(uint8_t mode);
	void    setConversionReadyPinMode();

	// *_THRESH registers
	int16_t getLowThreshold();
	void    setLowThreshold(int16_t threshold);
	int16_t getHighThreshold();
	void    setHighThreshold(int16_t threshold);

	// DEBUG
	void showConfigRegister();

	// ------------------------------------------------------------------------
private:
	I2Cdev   *mI2Cport;

	uint8_t  devAddr;  // device's I2C address
	uint16_t buffer[2];
	bool     devMode;
	uint8_t  muxMode;
	uint8_t  pgaMode;

#ifdef ADS1115_SERIAL_DEBUG
	Serial   mSerial;
#endif // ADS1115_SERIAL_DEBUG

	// ------------------------------------------------------------------------
};

//******************************************************************************
#endif /* _ADS1115_H_ */
