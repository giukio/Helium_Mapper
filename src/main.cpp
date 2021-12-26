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
#include <AtParser.h>
#include <vector>

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


String inputString = "";         // a string to hold incoming data
bool stringComplete = false;    // whether the string is complete

void command_listener();

AtParser at = AtParser();

void setup() {
  // put your setup code here, to run once:

  at.AddCommand(new AtCommand("get_config=device", [](std::vector<String> p){
    Serial.println("Get Config Command");
  }));

  at.AddCommand(new AtCommand("set_config=lora", [](std::vector<String> p){
    Serial.print("Set Config Command: ");
    String var = p.at(0);
    if(var == "join_mode")
    {
      Serial.print("join_mode");
    }
    else if(var == "dev_eui")
    {
      Serial.print("dev_eui");
    }
    else if(var == "app_eui")
    {
      Serial.print("app_eui");
    }
    else if(var == "app_key")
    {
      Serial.print("app_key");
    }
    else if(var == "send_interval")
    {
      Serial.print("send_interval");
    }
  }));

  at.AddCommand(new AtCommand("send", [](std::vector<String> p){
    Serial.println("Send Command");
  }));

  at.AddCommand(new AtCommand("join", [](std::vector<String> p){
    Serial.println("Join Command");
  }));

  at.AddCommand(new AtCommand("doh", [](std::vector<String> p){
    pinMode(p.at(0).toInt(),OUTPUT);
    digitalWrite(p.at(0).toInt(),HIGH);
    Serial.println("OK");
  }));

  at.AddCommand(new AtCommand("dol", [](std::vector<String> p){
    pinMode(p.at(0).toInt(),OUTPUT);
    digitalWrite(p.at(0).toInt(),LOW);
    Serial.println("OK");
  }));

  at.AddCommand(new AtCommand("dot", [](std::vector<String> p){
    pinMode(p.at(0).toInt(),OUTPUT);
    digitalToggle(p.at(0).toInt());
    Serial.println("OK");
  }));



  #ifndef ESP8266
    while (!Serial) yield();     // will pause Zero, Leonardo, etc until serial console opens
  #endif

  // Configure AcSiP S7xx Serial1 to Arduino Serial
  Serial.setTx(S7xx_CONSOLE_TX);
  Serial.setRx(S7xx_CONSOLE_RX);
  Serial.begin(115200);
    
  inputString.reserve(200); // reserve 200 bytes for the inputString:
  Serial.println("# Arduino AT command Control 1.0. RUNNING #");
}

void loop() {
  // put your main code here, to run repeatedly:
  // print the string when a newline arrives:
  if (stringComplete) {

    if(at.Parse(inputString) != 0)
    {
      Serial.println("Parse Error.");
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

    
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}