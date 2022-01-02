/*!
 *  @file main.cpp
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
#include <at.h>
#include <devices.h>
#include <lora.h>
#include <exception>

void setup() {
  // put your setup code here, to run once:
  pinMode(RAK7200_S76G_RED_LED, OUTPUT);
  pinMode(RAK7200_S76G_GREEN_LED, OUTPUT);
  pinMode(RAK7200_S76G_BLUE_LED, OUTPUT);

  dev.setConsole();
  Serial.println("\nHelium Mapper");
  setupAtCommands();
  dev.setGps();
  dev.setLora();
  LmicInit();

  digitalWrite(RAK7200_S76G_RED_LED, HIGH);
  digitalWrite(RAK7200_S76G_GREEN_LED, HIGH);
  digitalWrite(RAK7200_S76G_BLUE_LED, HIGH);
  Serial.println("Setup Complete.");
  Serial.flush();
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalToggle(RAK7200_S76G_RED_LED);
  readAtCommands();
  os_runloop_once();
  dev.getGpsFix();
  int32_t data = 0;

    if (dev.fix.valid.location) {
      dev.fix.valid.location = false;
      digitalToggle(RAK7200_S76G_GREEN_LED);

      // Prepare upstream data transmission at the next possible time.
        LoraParameter::gps location;

        location.lat = (int32_t)(dev.fix.latitudeL());
        Serial.print("Location: ");
        Serial.print(location.lat);

        location.lon = (int32_t)(dev.fix.longitudeL());
        Serial.print(", ");
        Serial.print(location.lon);

        location.alt = (int32_t)(dev.fix.altitude_cm());
        Serial.print(", Altitude: ");
        Serial.print(location.alt);

        lora.UpdateOrAppendParameter(LoraParameter(location, LoraParameter::Kind::gps));

        data = (uint8_t)(dev.fix.satellites);
        Serial.print(", Satellites: ");
        Serial.print(data);
        lora.UpdateOrAppendParameter(LoraParameter((uint8_t)data, LoraParameter::Kind::satellites));

        data = (uint16_t)(dev.fix.hdop);
        Serial.print(", HDOP: ");
        Serial.print(data);
        lora.UpdateOrAppendParameter(LoraParameter((uint16_t)data, LoraParameter::Kind::hdop));

        data = (int32_t)((dev.fix.speed_kph() * 100));
        Serial.print(", Speed: ");
        Serial.print(data);
        lora.UpdateOrAppendParameter(LoraParameter((uint16_t)data, LoraParameter::Kind::speed));
        Serial.println();

        Serial.print("Lora Packet: 0x");
        lora.PrintPacket();
    }
  float voltage = (float(analogRead(RAK7200_S76G_ADC_VBAT)) / 4096 * 3.30 / 0.6 * 10.0);
  // Serial.print(", V: ");
  // Serial.print(voltage);
  lora.UpdateOrAppendParameter(LoraParameter((uint16_t)(voltage * 1000.0), LoraParameter::Kind::voltage));

}
