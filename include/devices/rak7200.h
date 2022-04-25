/*!
 *  @file rak7200.h
 *
 *  BSD 3-Clause License
 *  Copyright (c) 2021, Giulio Berti
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef RAK7200_H
#define RAK7200_H

#include <deviceBase.h>
#include <Adafruit_LIS3DH.h>
#include <STM32LowPower.h>
#include <vector>

#define SX1276_RegVersion 0x42

// RAK7200 S76G GPIOs
#define RAK7200_S76G_BLUE_LED PA8	 // Blue LED (D2) active low
#define RAK7200_S76G_RED_LED PA11	 // Red LED (D3) active low
#define RAK7200_S76G_GREEN_LED PA12	 // Green LED (D4) active low
#define RAK7200_S76G_ADC_VBAT PB0	 // ADC connected to the battery (VBATT 1M PB0 1.5M GND) 1.5M / (1M + 1.5M) = 0.6
#define RAK7200_S76G_LIS3DH_INT1 PA0 // LIS3DH (U5) (I2C address 0x19) Interrupt INT1
#define RAK7200_S76G_LIS3DH_INT2 PB5 // LIS3DH (U5) (I2C address 0x19) Interrupt INT2
#define RAK7200_S76G_LPS_INT PA5	 // LPS22HB (U7) (I2C address 0x5C) Interrupt (mutually exclusive with SPI1_NSS)
#define RAK7200_S76G_MPU_INT PA5	 // MPU9250 (U8) (I2C address 0x68) Interrupt (mutually exclusive with SPI1_CLK)
#define RAK7200_S76G_TP4054_CHG1 PB1 // ADC TP4054 (U3)
#define RAK7200_S76G_TP4054_CHG2 PB8 // ADC TP4054 (U3)

// AcSiP S7xx UART1 (Console)
#define S7xx_CONSOLE_TX PA9	 // UART1 (CH340E U1)
#define S7xx_CONSOLE_RX PA10 // UART1 (CH340E U1)

// AcSiP S7xG SONY CXD5603GF GNSS
#define RAK7200_S76G_CXD5603_POWER_ENABLE PC4 // Enable 1V8 Power to GNSS (U2 TPS62740)
#define T_Motion_S76G_CXD5603_1PPS PB5		  // TTGO T-Motion 1PPS
#define S7xG_CXD5603_RESET PB2				  // Reset does not appear to work
#define S7xG_CXD5603_LEVEL_SHIFTER PC6
#define S7xG_CXD5603_UART_TX PC10 // UART4
#define S7xG_CXD5603_UART_RX PC11 // UART4
#define S7xG_CXD5603_BAUD_RATE 115200

// AcSiP S7xx Internal SPI2 STM32L073RZ(U|Y)x <--> SX127x
#define S7xx_SX127x_MOSI PB15 // SPI2
#define S7xx_SX127x_MISO PB14 // SPI2
#define S7xx_SX127x_SCK PB13  // SPI2
#define S7xx_SX127x_NSS PB12  // SPI2
#define S7xx_SX127x_NRESET PB10
#define S7xx_SX127x_DIO0 PB11
#define S7xx_SX127x_DIO1 PC13
#define S7xx_SX127x_DIO2 PB9
#define S7xx_SX127x_DIO3 PB4				// unused
#define S7xx_SX127x_DIO4 PB3				// unused
#define S7xx_SX127x_DIO5 PA15				// unused
#define S7xx_SX127x_ANTENNA_SWITCH_RXTX PA1 // Radio Antenna Switch 1:RX, 0:TX

// AcSiP S7xx I2C1
#define S7xx_I2C_SCL PB6 // I2C1
#define S7xx_I2C_SDA PB7 // I2C1

// Non Volatile (EEPROM) addresses
#define NV_DEVEUI 0x0B00
#define NV_APPEUI 0x0B08
#define NV_APPKEY 0x0B10

extern class Rak7200 dev;
uint64_t rtcmillis();

class Eeprom
{
private:
	const uint32_t EEPROM_BASE_ADDRESS = 0x08080000UL;
	const uint32_t EEPROM_SIZE = 0x1800UL;

public:
	/**
	 * writeEEPROM allows to write a byte(uint8_t) to the internal eeprom
	 * @param   address  starts at 0, the max size depends on the uc type
	 * @param   data     byte (uint8_t)
	 * @return  status   internal HAL_Status
	 */
	HAL_StatusTypeDef writeEEPROM(uint32_t address, uint8_t data);
	HAL_StatusTypeDef writeEEPROM(uint32_t address, uint16_t data);
	HAL_StatusTypeDef writeEEPROM(uint32_t address, uint32_t data);

	/**
	 * readEEPROMByte reads a byte from an internal eeprom
	 * @param   address  of the eeprom byte
	 * @return  data     as a byte (uint8_t)
	 */
	uint8_t readEEPROM8bit(uint32_t address);
	uint16_t readEEPROM16bit(uint32_t address);
	uint32_t readEEPROM32bit(uint32_t address);

	Eeprom(/* args */);
	~Eeprom();
};

class Rak7200 : public Device
{
public:
	Rak7200();
	void setConsole();
	void setGps();
	bool wakeGps();
	void sleepGps();
	gps_fix getGpsFix();
	void setLora();
	void setSensors();
	int8_t getTemperature();
	std::vector<float> getAcceleration();
	bool isMoving();
	bool isMotionJustStarted();
	void DumpEeprom();
	int8_t nvWrite(uint32_t address, uint8_t data);
	int8_t nvWrite(uint32_t address, uint16_t data);
	int8_t nvWrite(uint32_t address, uint32_t data);
	int8_t nvWrite(uint32_t address, uint64_t data);
	int8_t nvWrite(uint32_t address, uint8_t *data, uint16_t num);
	uint8_t nvRead8bit(uint32_t address);
	uint16_t nvRead16bit(uint32_t address);
	uint32_t nvRead32bit(uint32_t address);
	uint64_t nvRead64bit(uint32_t address);
	void nvRead(uint8_t *dest, uint32_t address, uint16_t num);
	void configLowPower();
	void configRtc();
	volatile bool wakeupSerial = false;
	volatile bool wakeupGps = false;
	volatile bool wakeupPin = false;
	volatile bool wakeupRtc = false;
	bool gpsDataAvailable();
	void updateMillis();
	void setRtcAlarmIn(uint8_t days, uint8_t hours, uint8_t minutes, uint8_t seconds);
	void setRtcAlarmIn(uint32_t seconds);
	void setRtcTimeFromGps();

private:
	bool GNSS_probe();
	HardwareSerial *_GNSS = NULL;
	bool _has_SX1276;

	Adafruit_LIS3DH *_lis = NULL;
	void setLis3dh();
	static void Lis3dhInt1_ISR();
	static void _rtcWakeup(void *data);
	static void _gpsWakeup();
	static void _serialWakeup();
	void deviceMoving();
	volatile bool Lis3dhInt1Flag = false;
	uint32_t _lastMotionMillis;
	uint32_t _motionWindowMs = 30000L;
	float _motionGpsSpeedThreshold = 5.0;
	STM32RTC &_rtc = STM32RTC::getInstance();
	bool gpsSleeping = false;


	Eeprom ee;
};

#endif