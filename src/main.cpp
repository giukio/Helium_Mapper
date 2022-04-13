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
#include <STM32LowPower.h>

#define SIMULATE_LORA false

enum eDeviceState
{
	DEVICE_STATE_INIT,
	DEVICE_STATE_JOIN,
	DEVICE_STATE_WAIT_GPS_FIX,
	DEVICE_STATE_SEND_MINIMAL,
	DEVICE_STATE_SEND_HEARTBEAT,
	DEVICE_STATE_SEND_WAIT_TX,
	DEVICE_STATE_CYCLE,
	DEVICE_STATE_SLEEP,
	DEVICE_STATE_IDLE
};
enum eDeviceState deviceState;
static uint32_t lastWaitGpsFix;

void setup()
{
	// put your setup code here, to run once:
	pinMode(RAK7200_S76G_RED_LED, OUTPUT);
	pinMode(RAK7200_S76G_GREEN_LED, OUTPUT);
	pinMode(RAK7200_S76G_BLUE_LED, OUTPUT);

	dev.configRtc();
	dev.updateMillis();
	dev.setConsole();
	Serial.println("\nHelium Mapper");

	setupAtCommands();
	dev.setGps();
	dev.setLora();
	LmicInit();
	dev.setSensors();
	dev.configLowPower();

	dev.setRtcAlarmIn(0, 0, 1, 0);
	lastWaitGpsFix = millis();
	deviceState = DEVICE_STATE_INIT;

	digitalWrite(RAK7200_S76G_RED_LED, HIGH);
	digitalWrite(RAK7200_S76G_GREEN_LED, HIGH);
	digitalWrite(RAK7200_S76G_BLUE_LED, HIGH);
	Serial.println("Setup Complete.");
	Serial.flush();
}

void loop()
{
	// put your main code here, to run repeatedly:
	static uint32_t lastSesorLoop;
	static uint32_t lastHeartBeat;
	extern uint64_t heartbeatTxInterval;
	static uint32_t lastMap;
	extern uint64_t mapTxInterval;
	uint64_t WaitGpsFixInterval = 3600;
	static eDeviceState lastState;
	static bool isGpsValid;

	readAtCommands();
	os_runloop_once();

	// if (lastState != deviceState)
	// {
	// 	Serial.print(millis());
	// 	Serial.print(": SM: ");
	// 	Serial.println(deviceState);
	// 	lastState = deviceState;
	// }

	switch (deviceState)
	{
	case DEVICE_STATE_INIT:
	{
		digitalWrite(RAK7200_S76G_RED_LED, LOW);
		lastSesorLoop = millis();
		dev.getTemperature(); // Temp bug workaround
		if (dev.wakeupPin)
		{
			if (dev.wakeGps())
			{
				lastWaitGpsFix = millis();
				Serial.println("Wakeup from External Pin.");
			}
			else
			{
				deviceState = DEVICE_STATE_SLEEP;
				break;
			}
		}
		deviceState = DEVICE_STATE_JOIN;
		break;
	}
	case DEVICE_STATE_JOIN:
	{
		if (LMIC.devaddr != 0 || SIMULATE_LORA)
		{
			if (dev.wakeupRtc)
			{
				isGpsValid = false;
				deviceState = DEVICE_STATE_SEND_HEARTBEAT;
			}
			else if (dev.wakeupGps || dev.wakeupPin)
			{
				Serial.print(millis());
				Serial.println(": Waiting for GPS Fix");
				deviceState = DEVICE_STATE_WAIT_GPS_FIX;
			}
			else
			{
				deviceState = DEVICE_STATE_IDLE;
			}
		}
		break;
	}
	case DEVICE_STATE_WAIT_GPS_FIX:
	{
		dev.getGpsFix();

		// Serial.print("Fix status: ");
		// Serial.println(dev.fix.status);

		isGpsValid = (dev.fix.status == gps_fix::status_t::STATUS_STD) && dev.fix.valid.status && dev.fix.valid.location && dev.fix.valid.altitude;
		digitalWrite(RAK7200_S76G_GREEN_LED, isGpsValid ? LOW : HIGH);

		if (isGpsValid)
		{
			Serial.print(millis());
			Serial.print(": GPS Fix valid, #sats: ");
			Serial.println(dev.fix.satellites);

			lastWaitGpsFix = millis();
			deviceState = DEVICE_STATE_CYCLE;
		}
		else if ((millis() - lastWaitGpsFix) >= WaitGpsFixInterval * 1000)
		{
			Serial.print(millis());
			Serial.println(": GPS fix not found");
			deviceState = DEVICE_STATE_SLEEP;
		}
		else // if ((millis() - lastSesorLoop) >= 150) // Time only for debug, prevents spurious wakeups while receving all nmea sentences
		{
			deviceState = DEVICE_STATE_IDLE;
		}

		break;
	}
	case DEVICE_STATE_CYCLE:
	{

		if (isGpsValid && dev.isMoving() && ((millis() - lastMap) >= (mapTxInterval * 1000)))
		{
			deviceState = DEVICE_STATE_SEND_MINIMAL;
		}
		else if (dev.isMoving() == false)
		{
			deviceState = DEVICE_STATE_SLEEP;
		}
		else
		{
			deviceState = DEVICE_STATE_IDLE;
		}
		break;
	}
	case DEVICE_STATE_SEND_MINIMAL:
	{
		digitalWrite(RAK7200_S76G_BLUE_LED, LOW);
		lastMap = millis();
		Serial.print(millis());
		Serial.println(": Sending Map");
		LoraParameter::gps location;
		location.lat = (int32_t)(dev.fix.latitudeL());
		location.lon = (int32_t)(dev.fix.longitudeL());
		location.alt = (int32_t)(dev.fix.altitude_cm());
		lora.UpdateOrAppendParameter(LoraParameter(location, LoraParameter::Kind::gpsMinimal));
#if SIMULATE_LORA == false
		do_send_mapping(lora.getSendjob(2));
#endif
		dev.fix.valid.init();	// Require new fix for new message
		deviceState = DEVICE_STATE_SEND_WAIT_TX;
		break;
	}
	case DEVICE_STATE_SEND_HEARTBEAT:
	{
		digitalWrite(RAK7200_S76G_BLUE_LED, LOW);
		lastHeartBeat = millis();
		Serial.print(millis());
		Serial.println(": Sending Heartbeat");
		lora.UpdateOrAppendParameter(LoraParameter((uint16_t)(dev.getTemperature() * 10), LoraParameter::Kind::temperature));

		pinMode(RAK7200_S76G_ADC_VBAT, INPUT_ANALOG);
		uint32_t vBatAdc = analogRead(RAK7200_S76G_ADC_VBAT);
		float voltage = (float(vBatAdc) / 4096 * 3.30 / 0.6 * 10.0);
		lora.UpdateOrAppendParameter(LoraParameter((uint16_t)(voltage * 100.0), LoraParameter::Kind::voltage));

		lora.BuildPacket();
		lora.PrintPacket();

#if SIMULATE_LORA == false
		do_send(lora.getSendjob(1));
#endif
		deviceState = DEVICE_STATE_SEND_WAIT_TX;
		break;
	}
	case DEVICE_STATE_SEND_WAIT_TX:
	{
		if (txComplete || SIMULATE_LORA)
		{
			deviceState = DEVICE_STATE_IDLE;
		}
		break;
	}
	case DEVICE_STATE_SLEEP:
	{
		dev.sleepGps();
		deviceState = DEVICE_STATE_IDLE;
		break;
	}
	case DEVICE_STATE_IDLE:
	{
		Serial.print(millis());
		Serial.println(": Going to Sleep now.");
		Serial.flush();
		digitalWrite(RAK7200_S76G_RED_LED, HIGH);
		digitalWrite(RAK7200_S76G_GREEN_LED, HIGH);
		digitalWrite(RAK7200_S76G_BLUE_LED, HIGH);

		dev.wakeupGps = false;
		dev.wakeupPin = false;
		dev.wakeupRtc = false;
		dev.wakeupSerial = false;
		LowPower.deepSleep();

		Serial.println("\r\nWoke up from sleep!");
		deviceState = DEVICE_STATE_INIT;
		break;
	}
	default:
	{
		deviceState = DEVICE_STATE_INIT;
		break;
	}
	}
}
