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
	static uint32_t lastMap;
	uint64_t WaitGpsFixInterval = 20 * 60 * 1000;
	static eDeviceState lastState;
	static uint32_t lastStateMilli = millis();
	static bool isGpsValid;
	static NeoGPS::Location_t lastMapLocation;
	static bool forceMap = true;

	readAtCommands();
	os_runloop_once();

	if (lastState != deviceState)
	{
		Serial.print(millis());
		Serial.print(": SM: ");
		Serial.println(deviceState);
		lastState = deviceState;
		lastStateMilli = millis();
	}
	else if ((millis() - lastStateMilli) > 60000) // SM watchdog
	{
		Serial.println("SM Watchdog tripped, resetting...");
		deviceState = DEVICE_STATE_INIT;
		lastStateMilli = millis();
	}

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
				forceMap = true;
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
		if (lora.isJoined() || SIMULATE_LORA)
		{
			if (dev.wakeupRtc)
			{
				isGpsValid = false;
				deviceState = DEVICE_STATE_SEND_HEARTBEAT;
			}
			else if (dev.wakeupGps || dev.wakeupPin)
			{
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

		isGpsValid = (dev.fix.status == gps_fix::status_t::STATUS_STD) && dev.fix.valid.status && dev.fix.valid.location && dev.fix.valid.altitude;
		digitalWrite(RAK7200_S76G_GREEN_LED, isGpsValid ? LOW : HIGH);

		if (isGpsValid)
		{
			lastWaitGpsFix = millis();
			deviceState = DEVICE_STATE_CYCLE;
		}
		else if ((millis() - lastWaitGpsFix) >= WaitGpsFixInterval)
		{
			Serial.print(millis());
			Serial.print(": GPS fix not found, lastWaitGpsFix:");
			Serial.println(lastWaitGpsFix);
			deviceState = DEVICE_STATE_SLEEP;
		}
		else // if ((millis() - lastSesorLoop) >= 300) // Time only for debug, prevents spurious wakeups while receving all nmea sentences
		{
			deviceState = DEVICE_STATE_IDLE;
		}

		break;
	}
	case DEVICE_STATE_CYCLE:
	{
		bool mapTime = ((millis() - lastMap) >= (mapTxInterval));
		bool mapDistance = dev.fix.location.DistanceKm(lastMapLocation) > 0.15F;
		Serial.printf("readyToTx:%d isGpsValid:%d isMoving:%d forceMap:%d mapTime:%d mapDistance:%d\r\n", lora.readyToTx(), isGpsValid, dev.isMoving(), forceMap, mapTime, mapDistance);
		if (lora.readyToTx() && isGpsValid && (dev.isMoving() || forceMap) && (mapTime || mapDistance))
		{
			deviceState = DEVICE_STATE_SEND_MINIMAL;
		}
		else if (dev.isMoving() == false && forceMap == false)
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
		lastMapLocation = dev.fix.location;
		LoraParameter::gps location;
		location.lat = (int32_t)(dev.fix.latitudeL());
		location.lon = (int32_t)(dev.fix.longitudeL());
		location.alt = (int32_t)(dev.fix.altitude_cm());
		lora.UpdateOrAppendParameter(LoraParameter(location, LoraParameter::Kind::gpsMinimal));
#if SIMULATE_LORA == false
		do_send_mapping(lora.getSendjob(2));
#endif
		forceMap = false;
		dev.fix.valid.init(); // Require new fix for new message
		deviceState = DEVICE_STATE_SEND_WAIT_TX;
		break;
	}
	case DEVICE_STATE_SEND_HEARTBEAT:
	{
		lora.UpdateOrAppendParameter(LoraParameter((uint16_t)(dev.getTemperature() * 10), LoraParameter::Kind::temperature));

		pinMode(RAK7200_S76G_ADC_VBAT, INPUT_ANALOG);
		uint32_t vBatAdc = analogRead(RAK7200_S76G_ADC_VBAT);
		float voltage = (float(vBatAdc) / 4096 * 3.30 / 0.6 * 10.0);
		lora.UpdateOrAppendParameter(LoraParameter((uint16_t)(voltage * 100.0), LoraParameter::Kind::voltage));

		if (lora.readyToTx())
		{
			lastHeartBeat = millis();
			Serial.print(millis());
			Serial.println(": Sending Heartbeat");
			digitalWrite(RAK7200_S76G_BLUE_LED, LOW);
			lora.BuildPacket();
			lora.PrintPacket();

#if SIMULATE_LORA == false
			do_send(lora.getSendjob(1));
#endif
			dev.wakeupRtc = false;
			deviceState = DEVICE_STATE_SEND_WAIT_TX;
		}
		else if (lora.readyToTxIn() > 0 && LMIC_queryTxReady())
		{
			uint32_t waitToTx = lora.readyToTxIn();
			Serial.printf("DutyCycle busy, next slot in %d ms\r\n", waitToTx);
			if (waitToTx > heartbeatTxInterval)		// probably a rollover of os_getTime()
			{
				Serial.println("Clamping Duty cycle to heartbeatTxInterval");
				waitToTx = heartbeatTxInterval;
			}
			dev.setRtcAlarmIn((uint32_t)waitToTx + 1000); // give some buffer for lmic to recognize EV_TXCOMPLETE
			dev.wakeupRtc = false;
			deviceState = DEVICE_STATE_IDLE;
		}

		break;
	}
	case DEVICE_STATE_SEND_WAIT_TX:
	{
		if (lora.dioTxComplete() || txComplete || SIMULATE_LORA)
		{
			lora.removeParameter(LoraParameter::Kind::gpsMinimal);
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
