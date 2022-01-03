/*!
 *  @file rak7200.cpp
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

#include <Arduino.h>
#include <deviceBase.h>
#include <devices\rak7200.h>
#include <SPI.h>



Rak7200::Rak7200(){
    _GNSS = new HardwareSerial(S7xG_CXD5603_UART_RX, S7xG_CXD5603_UART_TX);
    if (_GNSS == NULL)
    {
        Serial.println("ERROR: Couldn't Configure GNSS Hardware Serial");
        while(1);
    }

    _lis = new Adafruit_LIS3DH(new TwoWire(S7xx_I2C_SDA, S7xx_I2C_SCL));
}

void Rak7200::setConsole(){
  #ifndef ESP8266
    while (!Serial) yield();     // will pause Zero, Leonardo, etc until serial console opens
  #endif

  // Configure AcSiP S7xx Serial1 to Arduino Serial
  Serial.setTx(S7xx_CONSOLE_TX);
  Serial.setRx(S7xx_CONSOLE_RX);
  Serial.begin(115200);
}

bool Rak7200::GNSS_probe() {
    unsigned long startTime = millis();
    char c1, c2;
    c1 = c2 = 0;

    Rak7200::_GNSS->flush();
    while (millis() - startTime < 3000) {
        if (Rak7200::_GNSS->available() > 0) {
            c1 = Rak7200::_GNSS->read();
            if ((c1 == '$') && (c2 == 0)) {
                c2 = c1;
                continue;
            }
            if ((c2 == '$') && (c1 == 'G')) {
                // got $G leave the function with GNSS port opened
                return true;
            }
            else {
                c2 = 0;
            }
        }
        delay(1);
    }
    return false;
}

void Rak7200::setGps(){
    // power on GNSS
    Serial.println("Powering On GNSS");
    pinMode(RAK7200_S76G_CXD5603_POWER_ENABLE, OUTPUT);
    digitalWrite(RAK7200_S76G_CXD5603_POWER_ENABLE, HIGH);
    delay(1200); // Delay 315µs to 800µs ramp up time

    _GNSS->begin(S7xG_CXD5603_BAUD_RATE);

    /* drive GNSS RST pin low */
    pinMode(S7xG_CXD5603_RESET, OUTPUT);
    digitalWrite(S7xG_CXD5603_RESET, LOW);

    /* activate 1.8V<->3.3V level shifters */
    pinMode(S7xG_CXD5603_LEVEL_SHIFTER, OUTPUT);
    digitalWrite(S7xG_CXD5603_LEVEL_SHIFTER, HIGH);

    /* keep RST low to ensure proper IC reset */
    delay(250);

    /* release */
    digitalWrite(S7xG_CXD5603_RESET, HIGH);

    /* give Sony GNSS few ms to warm up */
    delay(125);

    /* configure GNSS */
    // _GNSS.write("@GCD\r\n"); // Cold start
    // delay(500);
    //_GNSS.write("@GSW\r\n"); // Warm start
    //delay(500);
    Rak7200::_GNSS->write("@GSP\r\n"); // Hot start for position accuracy
    delay(500);
    //_GNSS.write("@GPPS 0x1\r\n"); // Enable PPS
    //delay(125);
    /*
     * @GNS Select the satellite systems to be used for positioning
     * bit 0 : GPS          0x01
     * bit 1 : GLONASS      0x02
     * bit 2 : SBAS         0x04
     * bit 3 : QZSS L1-CA   0x08
     *
     */
    Rak7200::_GNSS->write("@GNS 0x7\r\n"); // Configure GPS, GLONASS, SBAS
    //_GNSS.write("@GNS 0x5\r\n"); // Configure GPS, SBAS
    //_GNSS.write("@GNS 0x1\r\n"); // Configure GPS
    //_GNSS.write("@GNS 0x2\r\n"); // Configure GLONASS
    delay(125);
    /*
     *
     * @BSSL Select NMEA sentences to output
     * bit0 : GGA 0x01
     * bit1 : GLL 0x02
     * bit2 : GSA 0x04
     * bit3 : GSV 0x08
     * bit4 : GNS 0x10
     * bit5 : RMC 0x20
     * bit6 : VTG 0x40
     * bit7 : ZDA 0x80
     *
     */
    //_GNSS.write("@BSSL 0xFF\r\n"); // All NMEA sentences
    //_GNSS.write("@BSSL 0xFE\r\n"); // All NMEA sentences but GGA
    //_GNSS.write("@BSSL 0xB3\r\n"); // GGA, GLL, GNS, RMC, ZDA
    //_GNSS.write("@BSSL 0xA1\r\n"); // GGA, RMC, ZDA
    Rak7200::_GNSS->write("@BSSL 0x21\r\n"); // GGA and RMC
    delay(125);

    Serial.print("GNSS   - ");
    Serial.println(Rak7200::GNSS_probe() ? "PASS" : "FAIL");  

    _GNSS->begin(115200);
    while (!_GNSS);
    Serial.println("GNSS UART Initialized");
}

gps_fix Rak7200::getGpsFix(){
    while (Rak7200::gps.available(*_GNSS)) {
        Rak7200::fix = Rak7200::gps.read();
    }
    return Rak7200::fix;
}

void Rak7200::setLora(){
    SPI.setMISO(S7xx_SX127x_MISO);
    SPI.setMOSI(S7xx_SX127x_MOSI);
    SPI.setSCLK(S7xx_SX127x_SCK);
    SPI.setSSEL(S7xx_SX127x_NSS);

    SPI.begin();

    digitalWrite(S7xx_SX127x_NSS, HIGH);
    pinMode(S7xx_SX127x_NSS, OUTPUT);

    digitalWrite(S7xx_SX127x_NRESET, HIGH);
    pinMode(S7xx_SX127x_NRESET, OUTPUT);

    // manually reset radio
    digitalWrite(S7xx_SX127x_NRESET, LOW);
    delay(5);
    digitalWrite(S7xx_SX127x_NRESET, HIGH);
    delay(5);

    digitalWrite(S7xx_SX127x_NSS, LOW);

    SPI.transfer(SX1276_RegVersion & 0x7F);
    _has_SX1276 = (SPI.transfer(0x00) == 0x12 ? true : false);

    digitalWrite(S7xx_SX127x_NSS, HIGH);

    SPI.end();
    pinMode(S7xx_SX127x_NSS, INPUT);
    pinMode(S7xx_SX127x_NRESET, INPUT);

    Serial.println("Built-in components:");

    Serial.print("RADIO  - ");
    Serial.println(_has_SX1276 ? "PASS" : "FAIL");
}

// LMIC library expects pinmap as global constant
const lmic_pinmap lmic_pins = {
        .nss = S7xx_SX127x_NSS,
        .rxtx = S7xx_SX127x_ANTENNA_SWITCH_RXTX,
        .rst = S7xx_SX127x_NRESET,
        .dio = {S7xx_SX127x_DIO0, S7xx_SX127x_DIO1, S7xx_SX127x_DIO2},
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 1000000
};


void Rak7200::setSensors(){
    this->setLis3dh();
}

void Rak7200::setLis3dh(){
    if (this->_lis->begin(0x19) == false){
        Serial.println("Couldnt Start LIS3DH Accelerometer");
        while (1)
        {
            yield();
        }
    }
    Serial.println("LIS3DH found!");

    this->_lis->enableDRDY(false,1);    // Leave INT1 PIN free
    
    this->_lis->setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
    
    Serial.print("Range = "); Serial.print(2 << this->_lis->getRange());  
    Serial.println("G");

    // this->_lis->setDataRate(LIS3DH_DATARATE_50_HZ);
    Serial.print("Data rate set to: ");
    switch (this->_lis->getDataRate()) {
        case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
        case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
        case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
        case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
        case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
        case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
        case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

        case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
        case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
        case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
    }

    // 0 = turn off click detection & interrupt
    // 1 = single click only interrupt output
    // 2 = double click only interrupt output, detect single click
    // Adjust threshhold, higher numbers are less sensitive
    this->_lis->setClick(2, 40);
}

int8_t Rak7200::getTemperature(){
    return this->_lis->readTemperature(13);
}

std::vector<float> Rak7200::getAcceleration(){
    this->_lis->read();
    return std::vector<float>{ this->_lis->x_g, this->_lis->y_g, this->_lis->z_g };
}
