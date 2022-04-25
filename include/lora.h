/*!
 *  @file lora.h
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

#include <devices.h>
#include <vector>

extern class Lora lora;
extern u1_t PROGMEM DEVEUI[8];
extern u1_t PROGMEM APPEUI[8];
extern u1_t PROGMEM APPKEY[16];
extern bool txComplete;
extern uint64_t heartbeatTxInterval;
extern uint64_t mapTxInterval;

void printHex2(unsigned v);

class LoraParameter
{

public:
    enum class Kind
    {
        unspecified = 0x0000,
        gpsMinimal = 0x0187,
        gps = 0x0188,
        speed = 0x0189,
        satellites = 0x0190,
        hdop = 0x0191,
        temperature = 0x0267,
        acceleration = 0x0371,
        airResistance = 0x0402,
        gyroscope = 0x0586,
        pressure = 0x0673,
        humidity = 0x0768,
        voltage = 0x0802,
        magnetometerX = 0x0902,
        magnetometerY = 0x0a02,
        magnetometerZ = 0x0b02
    };

    struct gps
    {
        int32_t lat;
        int32_t lon;
        int32_t alt;
    };

    LoraParameter(uint8_t par, Kind kind = Kind::unspecified);
    LoraParameter(uint16_t par, Kind kind = Kind::unspecified);
    LoraParameter(uint32_t par, Kind kind = Kind::unspecified);
    LoraParameter(LoraParameter::gps par, Kind kind = Kind::unspecified);
    LoraParameter(std::vector<uint16_t> par, Kind kind = Kind::unspecified);
    Kind GetKind();
    std::vector<uint8_t> GetData();
    ~LoraParameter();

private:
    Kind _kind;
    std::vector<uint8_t> _data;
};

class Lora
{
private:
    std::vector<uint8_t> _packet;
    std::vector<LoraParameter> _parameters;
    osjob_t _sendjob;
    osjob_t _mapjob;

public:
    Lora(/* args */);

    void AppendParameter(LoraParameter p);
    void UpdateOrAppendParameter(LoraParameter p);
    LoraParameter *getParameter(LoraParameter::Kind k);
    bool removeParameter(LoraParameter::Kind k);
    osjob_t *getSendjob(uint8_t port = 1);
    void setTxData();
    void BuildPacket();
    void PrintPacket();
    bool isJoined();
    bool readyToTx();
    uint32_t readyToTxIn();
    bool dioTxComplete();
    ~Lora();
};

void printHex2(unsigned v);
void do_send(osjob_t *j);
void do_send_mapping(osjob_t *j);
void LmicInit();

// // ttn application function to decode uplink data.
// // Decode decodes an array of bytes into an object.
// //  - port contains the LoRaWAN fPort number
// //  - bytes is an array of bytes, e.g. [225, 230, 255, 0]
// // The function must return an object, e.g. {"temperature": 22.5}
// function Decoder(bytes, port) {
//   var decoded = {};
//   var hexString=bin2HexStr(bytes);
//   return rakSensorDataDecode(hexString);
// }

// // convert array of bytes to hex string.
// // e.g: 0188053797109D5900DC140802017A0768580673256D0267011D040214AF0371FFFFFFDDFC2E
// function bin2HexStr(bytesArr) {
//   var str = "";
//   for(var i=0; i<bytesArr.length; i++) {
//     var tmp = (bytesArr[i] & 0xff).toString(16);
//     if(tmp.length == 1) {
//       tmp = "0" + tmp;
//     }
//     str += tmp;
//   }
//   return str;
// }

// // convert string to short integer
// function parseShort(str, base) {
//   var n = parseInt(str, base);
//   return (n << 16) >> 16;
// }

// // convert string to triple bytes integer
// function parseTriple(str, base) {
//   var n = parseInt(str, base);
//   return (n << 8) >> 8;
// }

// // decode Hex sensor string data to object
// function rakSensorDataDecode(hexStr) {
//   var str = hexStr;
//   var myObj = {};

//   while (str.length > 4) {
//     var flag = parseInt(str.substring(0, 4), 16);
//     switch (flag) {
//       case 0x0768:// Humidity
//         myObj.humidity = parseFloat(((parseShort(str.substring(4, 6), 16) * 0.01 / 2) * 100).toFixed(1)) + "%RH";//unit:%RH
//         str = str.substring(6);
//         break;
//       case 0x0673:// Atmospheric pressure
//         myObj.barometer = parseFloat((parseShort(str.substring(4, 8), 16) * 0.1).toFixed(2)) + "hPa";//unit:hPa
//         str = str.substring(8);
//         break;
//       case 0x0267:// Temperature
//         myObj.temperature = parseFloat((parseShort(str.substring(4, 8), 16) * 0.1).toFixed(2)) + "°C";//unit: °C
//         str = str.substring(8);
//         break;
//       case 0x0188:// GPS
//         myObj.latitude = parseFloat((parseTriple(str.substring(4, 10), 16) * 0.0001).toFixed(4)) + "°";//unit:°
//         myObj.longitude = parseFloat((parseTriple(str.substring(10, 16), 16) * 0.0001).toFixed(4)) + "°";//unit:°
//         myObj.altitude = parseFloat((parseTriple(str.substring(16, 22), 16) * 0.01).toFixed(1)) + "m";//unit:m
//         str = str.substring(22);
//         break;
//       case 0x0371:// Triaxial acceleration
//         myObj.acceleration_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.001).toFixed(3)) + "g";//unit:g
//         myObj.acceleration_y = parseFloat((parseShort(str.substring(8, 12), 16) * 0.001).toFixed(3)) + "g";//unit:g
//         myObj.acceleration_z = parseFloat((parseShort(str.substring(12, 16), 16) * 0.001).toFixed(3)) + "g";//unit:g
//         str = str.substring(16);
//         break;
//       case 0x0402:// air resistance
//         myObj.gasResistance = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "KΩ";//unit:KΩ
//         str = str.substring(8);
//         break;
//       case 0x0802:// Battery Voltage
//         myObj.battery = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "V";//unit:V
//         str = str.substring(8);
//         break;
//       case 0x0586:// gyroscope
//         myObj.gyroscope_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "°/s";//unit:°/s
//         myObj.gyroscope_y = parseFloat((parseShort(str.substring(8, 12), 16) * 0.01).toFixed(2)) + "°/s";//unit:°/s
//         myObj.gyroscope_z = parseFloat((parseShort(str.substring(12, 16), 16) * 0.01).toFixed(2)) + "°/s";//unit:°/s
//         str = str.substring(16);
//         break;
//       case 0x0902:// magnetometer x
//         myObj.magnetometer_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "μT";//unit:μT
//         str = str.substring(8);
//         break;
//       case 0x0a02:// magnetometer y
//         myObj.magnetometer_y = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "μT";//unit:μT
//         str = str.substring(8);
//         break;
//       case 0x0b02:// magnetometer z
//         myObj.magnetometer_z = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "μT";//unit:μT
//         str = str.substring(8);
//         break;
//       default:
//         str = str.substring(7);
//         break;
//     }
//   }

//   return myObj;
// }

// // ttn application function to decode uplink data.
// // Decode decodes an array of bytes into an object.
// //  - port contains the LoRaWAN fPort number
// //  - bytes is an array of bytes, e.g. [225, 230, 255, 0]
// // The function must return an object, e.g. {"temperature": 22.5}
// function Decoder(bytes, port) {
//   var decoded = {};
//   var hexString=bin2HexStr(bytes);
//   return rakSensorDataDecode(hexString);
// }

// // convert array of bytes to hex string.
// // e.g: 01880500379708109D590000DC140802017A0768580673256D0267011D040214AF0371FFFFFFDDFC2E
// function bin2HexStr(bytesArr) {
//   var str = "";
//   for(var i=0; i<bytesArr.length; i++) {
//     var tmp = (bytesArr[i] & 0xff).toString(16);
//     if(tmp.length == 1) {
//       tmp = "0" + tmp;
//     }
//     str += tmp;
//   }
//   return str;
// }

// // convert string to short integer
// function parseShort(str, base) {
//   var n = parseInt(str, base);
//   return (n << 16) >> 16;
// }

// // convert string to Quadruple bytes integer
// function parseQuadruple(str, base) {
//   var n = parseInt(str, base);
//   return (n << 32) >> 32;
// }

// // decode Hex sensor string data to object
// function rakSensorDataDecode(hexStr) {
//   var str = hexStr;
//   var myObj = {};

//   while (str.length > 4) {
//     var flag = parseInt(str.substring(0, 4), 16);
//     switch (flag) {
//       case 0x0768:// Humidity
//         myObj.humidity = parseFloat(((parseShort(str.substring(4, 6), 16) * 0.01 / 2) * 100).toFixed(1)) + "%RH";//unit:%RH
//         str = str.substring(6);
//         break;
//       case 0x0673:// Atmospheric pressure
//         myObj.barometer = parseFloat((parseShort(str.substring(4, 8), 16) * 0.1).toFixed(2)) + "hPa";//unit:hPa
//         str = str.substring(8);
//         break;
//       case 0x0267:// Temperature
//         myObj.temperature = parseFloat((parseShort(str.substring(4, 8), 16) * 0.1).toFixed(2)) + "°C";//unit: °C
//         str = str.substring(8);
//         break;
//       case 0x0188:// GPS
//         myObj.latitude = parseFloat((parseQuadruple(str.substring(4, 12), 16) * 0.000001).toFixed(6)) + "°";//unit:°
//         myObj.longitude = parseFloat((parseQuadruple(str.substring(12, 20), 16) * 0.000001).toFixed(6)) + "°";//unit:°
//         myObj.altitude = parseFloat((parseQuadruple(str.substring(20, 28), 16) * 0.01).toFixed(1)) + "m";//unit:m
//         str = str.substring(28);
//         break;
//       case 0x0371:// Triaxial acceleration
//         myObj.acceleration_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.001).toFixed(3)) + "g";//unit:g
//         myObj.acceleration_y = parseFloat((parseShort(str.substring(8, 12), 16) * 0.001).toFixed(3)) + "g";//unit:g
//         myObj.acceleration_z = parseFloat((parseShort(str.substring(12, 16), 16) * 0.001).toFixed(3)) + "g";//unit:g
//         str = str.substring(16);
//         break;
//       case 0x0402:// air resistance
//         myObj.gasResistance = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "KΩ";//unit:KΩ
//         str = str.substring(8);
//         break;
//       case 0x0802:// Battery Voltage
//         myObj.battery = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "V";//unit:V
//         str = str.substring(8);
//         break;
//       case 0x0586:// gyroscope
//         myObj.gyroscope_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "°/s";//unit:°/s
//         myObj.gyroscope_y = parseFloat((parseShort(str.substring(8, 12), 16) * 0.01).toFixed(2)) + "°/s";//unit:°/s
//         myObj.gyroscope_z = parseFloat((parseShort(str.substring(12, 16), 16) * 0.01).toFixed(2)) + "°/s";//unit:°/s
//         str = str.substring(16);
//         break;
//       case 0x0902:// magnetometer x
//         myObj.magnetometer_x = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "μT";//unit:μT
//         str = str.substring(8);
//         break;
//       case 0x0a02:// magnetometer y
//         myObj.magnetometer_y = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "μT";//unit:μT
//         str = str.substring(8);
//         break;
//       case 0x0b02:// magnetometer z
//         myObj.magnetometer_z = parseFloat((parseShort(str.substring(4, 8), 16) * 0.01).toFixed(2)) + "μT";//unit:μT
//         str = str.substring(8);
//         break;
//       default:
//         str = str.substring(7);
//         break;
//     }
//   }

//   return myObj;
// }