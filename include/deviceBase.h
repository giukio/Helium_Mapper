/*!
 *  @file deviceBase.h
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
#ifndef DEVICEBASE_H
#define DEVICEBASE_H

#include <NMEAGPS.h>  //http://librarymanager/All#NeoGPS
#include <lmic.h>     //http://librarymanager/All#MCCI+LoRaWAN+LMIC+library
#include <hal/hal.h>

// Base class
class Device {
    public:
        virtual void setConsole() = 0;
        virtual void setGps() = 0;
        virtual gps_fix getGpsFix() = 0;
        virtual void setLora() = 0;
        virtual void setSensors() = 0;
        virtual bool isMoving() = 0;
        virtual bool isMotionJustStarted() = 0;

        virtual int8_t nvWrite(uint32_t address, uint8_t data) = 0;
        virtual int8_t nvWrite(uint32_t address, uint16_t data) = 0;
        virtual int8_t nvWrite(uint32_t address, uint32_t data) = 0;
        virtual int8_t nvWrite(uint32_t address, uint64_t data) = 0;
        virtual int8_t nvWrite(uint32_t address, uint8_t* data, uint16_t num) = 0;
        virtual uint8_t nvRead8bit(uint32_t address) = 0;
        virtual uint16_t nvRead16bit(uint32_t address) = 0;
        virtual uint32_t nvRead32bit(uint32_t address) = 0;
        virtual uint64_t nvRead64bit(uint32_t address) = 0;
        virtual void nvRead(uint8_t* dest, uint32_t address, uint16_t num) = 0;
        gps_fix fix;
    protected:
        NMEAGPS gps;
    
};

#endif