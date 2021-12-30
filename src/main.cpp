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




void setup() {
  // put your setup code here, to run once:

  dev.setConsole();
  Serial.println("\nHelium Mapper");
  setupAtCommands();
  dev.setGps();
  dev.setLora();
}

void loop() {
  // put your main code here, to run repeatedly:
  readAtCommands();
  os_runloop_once();

  gps_fix fix = dev.getGpsFix();
  if (fix.valid.location) {
    fix.valid.location = false;
    digitalToggle(RAK7200_S76G_GREEN_LED);

    // Prepare upstream data transmission at the next possible time.
    Serial.print("Location: ");
    Serial.print(fix.latitudeL());

    Serial.print(", ");
    Serial.print(fix.longitudeL());

    Serial.print(", Altitude: ");
    Serial.print(fix.altitude_cm());

    Serial.print(", Satellites: ");
    Serial.print(fix.satellites);

    Serial.print(", HDOP: ");
    Serial.print(fix.hdop);

    Serial.print(", Speed: ");
    Serial.print(fix.speed_kph());
    
    Serial.print(", V: ");
    Serial.print(float(analogRead(RAK7200_S76G_ADC_VBAT)) / 4096 * 3.30 / 0.6 * 10.0);
    Serial.println();
  }

}
