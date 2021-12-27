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

// RAK7200 S76G GPIOs
#define RAK7200_S76G_BLUE_LED             PA8  // Blue LED (D2) active low
#define RAK7200_S76G_RED_LED              PA11 // Red LED (D3) active low
#define RAK7200_S76G_GREEN_LED            PA12 // Green LED (D4) active low
#define RAK7200_S76G_ADC_VBAT             PB0  // ADC connected to the battery (VBATT 1M PB0 1.5M GND) 1.5M / (1M + 1.5M) = 0.6
#define RAK7200_S76G_LIS3DH_INT1          PA0  // LIS3DH (U5) (I2C address 0x19) Interrupt INT1
#define RAK7200_S76G_LIS3DH_INT2          PB5  // LIS3DH (U5) (I2C address 0x19) Interrupt INT2
#define RAK7200_S76G_LPS_INT              PA5  // LPS22HB (U7) (I2C address 0x5C) Interrupt (mutually exclusive with SPI1_NSS)
#define RAK7200_S76G_MPU_INT              PA5  // MPU9250 (U8) (I2C address 0x68) Interrupt (mutually exclusive with SPI1_CLK)
#define RAK7200_S76G_TP4054_CHG1          PB1  // ADC TP4054 (U3)
#define RAK7200_S76G_TP4054_CHG2          PB8  // ADC TP4054 (U3)

// AcSiP S7xx UART1 (Console)
#define S7xx_CONSOLE_TX                   PA9  // UART1 (CH340E U1)
#define S7xx_CONSOLE_RX                   PA10 // UART1 (CH340E U1)

// AcSiP S7xG SONY CXD5603GF GNSS
#define RAK7200_S76G_CXD5603_POWER_ENABLE PC4  // Enable 1V8 Power to GNSS (U2 TPS62740)
#define T_Motion_S76G_CXD5603_1PPS        PB5  // TTGO T-Motion 1PPS
#define S7xG_CXD5603_RESET                PB2  // Reset does not appear to work
#define S7xG_CXD5603_LEVEL_SHIFTER        PC6
#define S7xG_CXD5603_UART_TX              PC10 // UART4
#define S7xG_CXD5603_UART_RX              PC11 // UART4
#define S7xG_CXD5603_BAUD_RATE            115200


extern class Rak7200 dev;

class Rak7200: public Device {
    public:
        Rak7200();
        void setConsole();
        void setGps();

    private:
        bool GNSS_probe();
        HardwareSerial* _GNSS;
        

};

#endif