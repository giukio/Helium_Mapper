/*!
 *  @file at.cpp
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

#include <AtParser.h>

AtParser at = AtParser();
String inputString = "";         // a string to hold incoming data

void setupAtCommands(){
    inputString.reserve(200); // reserve 200 bytes for the inputString:
    Serial.println("# Arduino AT command Control 1.0. RUNNING #");

    at.AddCommand(new AtCommand("version", [](std::vector<String> p){
        Serial.println("Helium Mapper Version 1.0.0");
    }));

    at.AddCommand(new AtCommand("get_config=device", [](std::vector<String> p){
        Serial.println("Get Config Command");
    }));

    at.AddCommand(new AtCommand("set_config=lora", [](std::vector<String> p){
        if (p.size() < 1) { Serial.println("ERROR: 2"); return;}
        for (auto &&i : p)
        {
            Serial.print(i);
        }
        
        String var = p.at(0);
        if(var == "join_mode")
        {
            if (p.size() < 2) { Serial.println("ERROR: 2"); return;}
            if (p.at(1) == "0"){
                Serial.println("join_mode:OTAA");
            }
            else if (p.at(1) == "1"){
                Serial.println("join_mode:ABP not supported");
            }
            else{
                Serial.print(p.at(1)); 
                Serial.println("ERROR: 2"); 
                return;
            }
        }
        else if(var == "region")
        {
            if (p.size() < 2) { Serial.println("ERROR: 2"); return;}
            if (p.at(1) == "EU868"){
                #ifdef CFG_eu868
                    Serial.println("Selected LoRaWAN 1.0.2 Region: EU868\r\nOK");
                #else
                    Serial.println("ERROR: Program needs to be recompiled using define CFG_eu868");
                #endif
            }
            else if (p.at(1) == "US915"){
                #ifdef CFG_us915
                    Serial.println("Selected LoRaWAN 1.0.2 Region: US915\r\nOK");
                #else
                    Serial.println("ERROR: Program needs to be recompiled using define CFG_us915");
                #endif
            }
            else{
                Serial.println("ERROR: 2"); 
                return;
            }
        }
        else if(var == "dev_eui")
        {
            if (p.size() < 2) { Serial.println("ERROR: 2"); return;}
            String devEui = p.at(1);
            if (devEui.length() != 16) { Serial.println("ERROR: 2"); return;}
            while (devEui.length() >= 2)
            {
                uint8_t b = (uint8_t)devEui.substring(0,2).toInt();
                Serial.println(b);
                devEui = devEui.substring(2);
            }
            
                    
            Serial.println("OK");
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

}

void readAtCommands(){
    while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
        if(at.Parse(inputString) != 0)
        {
            Serial.println("Parse Error.");
        }
        // clear the string:
        inputString = "";
    }
  }

}