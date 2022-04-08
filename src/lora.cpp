/*!
 *  @file lora.cpp
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
#include <lora.h>

uint64_t heartbeatTxInterval = 600;
uint64_t mapTxInterval = 10;
bool txComplete = false;

Lora lora;

LoraParameter::LoraParameter(uint8_t par, Kind kind)
{
	_kind = kind;
	_data = std::vector<uint8_t>{par};
}

LoraParameter::LoraParameter(uint16_t par, Kind kind)
{
	_kind = kind;
	_data = std::vector<uint8_t>{
		static_cast<uint8_t>(par >> 8),
		static_cast<uint8_t>(par)};
}

LoraParameter::LoraParameter(uint32_t par, Kind kind)
{
	_kind = kind;
	_data = std::vector<uint8_t>{
		static_cast<uint8_t>(par >> 24),
		static_cast<uint8_t>(par >> 16),
		static_cast<uint8_t>(par >> 8),
		static_cast<uint8_t>(par)};
}

LoraParameter::LoraParameter(LoraParameter::gps par, Kind kind)
{
	_kind = kind;
	if (kind == LoraParameter::Kind::gpsMinimal)
	{
		int16_t altM = par.alt / 100;
		_data = std::vector<uint8_t>{
			static_cast<uint8_t>(par.lat >> 24),
			static_cast<uint8_t>(par.lat >> 16),
			static_cast<uint8_t>(par.lat >> 8),
			static_cast<uint8_t>(par.lon >> 24),
			static_cast<uint8_t>(par.lon >> 16),
			static_cast<uint8_t>(par.lon >> 8),
			static_cast<uint8_t>(altM >> 8),
			static_cast<uint8_t>(altM)};
	}
	else
	{
		_data = std::vector<uint8_t>{
			static_cast<uint8_t>(par.lat >> 24),
			static_cast<uint8_t>(par.lat >> 16),
			static_cast<uint8_t>(par.lat >> 8),
			static_cast<uint8_t>(par.lat),
			static_cast<uint8_t>(par.lon >> 24),
			static_cast<uint8_t>(par.lon >> 16),
			static_cast<uint8_t>(par.lon >> 8),
			static_cast<uint8_t>(par.lon),
			static_cast<uint8_t>(par.alt >> 24),
			static_cast<uint8_t>(par.alt >> 16),
			static_cast<uint8_t>(par.alt >> 8),
			static_cast<uint8_t>(par.alt)};
	}
}
LoraParameter::LoraParameter(std::vector<uint16_t> parVec, Kind kind)
{
	_kind = kind;
	_data.clear();
	for (auto &&par : parVec)
	{
		_data.push_back(static_cast<uint8_t>(par >> 8));
		_data.push_back(static_cast<uint8_t>(par));
	}
}

LoraParameter::Kind LoraParameter::GetKind()
{
	return this->_kind;
}

std::vector<uint8_t> LoraParameter::GetData()
{
	return this->_data;
}

LoraParameter::~LoraParameter()
{
}

Lora::Lora(/* args */)
{
}

void Lora::AppendParameter(LoraParameter p)
{
	this->_parameters.push_back(p);
	Serial.print("Added Lora Parameter: ");
	Serial.print((uint16_t)p.GetKind(), HEX);
	Serial.print(" of size: ");
	Serial.println(p.GetData().size());
}

void Lora::UpdateOrAppendParameter(LoraParameter p)
{
	for (auto &&i : this->_parameters)
	{
		if (i.GetKind() == p.GetKind())
		{
			i = p;
			return;
		}
	}
	// else, p kind was not present, append it.
	this->AppendParameter(p);
}

LoraParameter *Lora::getParameter(LoraParameter::Kind k)
{
	for (auto &&i : this->_parameters)
	{
		if (i.GetKind() == k)
		{
			return &i;
		}
	}
	return NULL;
}

osjob_t *Lora::getSendjob(uint8_t port)
{
	osjob_t *j;
	switch (port)
	{
	case 1:
		j = &this->_sendjob;
		break;
	case 2:
		j = &this->_mapjob;
		break;

	default:
		j = &this->_sendjob;
		break;
	}
	return j;
}

void Lora::setTxData()
{
	this->BuildPacket();
	LMIC_setTxData2(1, this->_packet.data(), this->_packet.size(), 0);
	this->PrintPacket();
}

void Lora::BuildPacket()
{
	this->_packet.clear();
	// Serial.print("BuildPacket: packet length "); Serial.print(this->_packet.size()); Serial.println(" bytes.");
	// Serial.print("BuildPacket: there are "); Serial.print(this->_parameters.size()); Serial.println(" parameters.");
	for (auto &&p : this->_parameters)
	{
		std::vector<uint8_t> kind = std::vector<uint8_t>{
			static_cast<uint8_t>((uint16_t)p.GetKind() >> 8),
			static_cast<uint8_t>((uint16_t)p.GetKind())};
		this->_packet.insert(this->_packet.end(), kind.begin(), kind.end());
		std::vector<uint8_t> data = p.GetData();
		this->_packet.insert(this->_packet.end(), data.begin(), data.end());
	}
	// Serial.print("BuildPacket: packet length "); Serial.print(this->_packet.size()); Serial.println(" bytes.");
}

void Lora::PrintPacket()
{
	Serial.print("Lora Packet: 0x");
	for (auto &&p : this->_packet)
	{
		printHex2(p);
	}
	Serial.println();
}

Lora::~Lora()
{
}

void onEvent(ev_t ev)
{
	Serial.print(os_getTime());
	Serial.print(": ");
	switch (ev)
	{
	case EV_SCAN_TIMEOUT:
		Serial.println(F("EV_SCAN_TIMEOUT"));
		break;
	case EV_BEACON_FOUND:
		Serial.println(F("EV_BEACON_FOUND"));
		break;
	case EV_BEACON_MISSED:
		Serial.println(F("EV_BEACON_MISSED"));
		break;
	case EV_BEACON_TRACKED:
		Serial.println(F("EV_BEACON_TRACKED"));
		break;
	case EV_JOINING:
		Serial.println(F("EV_JOINING"));
		break;
	case EV_JOINED:
		Serial.println(F("EV_JOINED"));
		{
			u4_t netid = 0;
			devaddr_t devaddr = 0;
			u1_t nwkKey[16];
			u1_t artKey[16];
			LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
			Serial.print("netid: ");
			Serial.println(netid, DEC);
			Serial.print("devaddr: ");
			Serial.println(__builtin_bswap32(devaddr), HEX);
			Serial.print("AppSKey: ");
			for (size_t i = 0; i < sizeof(artKey); ++i)
			{
				if (i != 0)
				{
					Serial.print("-");
				}
				printHex2(artKey[i]);
			}
			Serial.println("");
			Serial.print("NwkSKey: ");
			for (size_t i = 0; i < sizeof(nwkKey); ++i)
			{
				if (i != 0)
				{
					Serial.print("-");
				}
				printHex2(nwkKey[i]);
			}
			Serial.println();
		}
		// Disable link check validation (automatically enabled
		// during join, but because slow data rates change max TX
		// size, we don't use it in this example.
		LMIC_setLinkCheckMode(0);
		break;
		/*
		|| This event is defined but not used in the code. No
		|| point in wasting codespace on it.
		||
		|| case EV_RFU1:
		||     Serial.println(F("EV_RFU1"));
		||     break;
		*/
	case EV_JOIN_FAILED:
		Serial.println(F("EV_JOIN_FAILED"));
		break;
	case EV_REJOIN_FAILED:
		Serial.println(F("EV_REJOIN_FAILED"));
		break;
		break;
	case EV_TXCOMPLETE:
		txComplete = true;
		Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
		if (LMIC.txrxFlags & TXRX_ACK)
		{
			Serial.println(F("Received ack"));
		}
		if (LMIC.dataLen)
		{
			Serial.println(F("Received "));
			Serial.println(LMIC.dataLen);
			Serial.println(F(" bytes of payload"));
		}
		// {
		//     u1_t bPort = 0;
		//     Serial.print("EV_TXCOMPLETE: LMIC.txrxFlags 0b"); Serial.println(LMIC.txrxFlags, BIN);

		//     // if (LMIC.txrxFlags & TXRX_PORT){
		//         bPort = LMIC.frame[LMIC.dataBeg - 1];
		//     // }
		//     if ( bPort == 1){
		//         Serial.print("EV_TXCOMPLETE: port "); Serial.println(bPort);
		//         // os_setTimedCallback(lora.getSendjob(1), os_getTime() + sec2osticks(heartbeatTxInterval), do_send);
		//     }
		//     else if ( bPort == 2){
		//         Serial.print("EV_TXCOMPLETE: port "); Serial.println(bPort);
		//         if (dev.isMoving())
		//         {
		//             os_setTimedCallback(lora.getSendjob(2), os_getTime() + sec2osticks(mapTxInterval), do_send_mapping);
		//         }
		//     }
		//     else{
		//         Serial.print("EV_TXCOMPLETE Error: unknown port "); Serial.println(bPort);
		//     }
		// }
		break;
	case EV_LOST_TSYNC:
		Serial.println(F("EV_LOST_TSYNC"));
		break;
	case EV_RESET:
		Serial.println(F("EV_RESET"));
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		Serial.println(F("EV_RXCOMPLETE"));
		break;
	case EV_LINK_DEAD:
		Serial.println(F("EV_LINK_DEAD"));
		break;
	case EV_LINK_ALIVE:
		Serial.println(F("EV_LINK_ALIVE"));
		break;
		/*
		|| This event is defined but not used in the code. No
		|| point in wasting codespace on it.
		||
		|| case EV_SCAN_FOUND:
		||    Serial.println(F("EV_SCAN_FOUND"));
		||    break;
		*/
	case EV_TXSTART:
		Serial.println(F("EV_TXSTART"));
		break;
	case EV_TXCANCELED:
		Serial.println(F("EV_TXCANCELED"));
		break;
	case EV_RXSTART:
		/* do not print anything -- it wrecks timing */
		break;
	case EV_JOIN_TXCOMPLETE:
		Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
		break;

	default:
		Serial.print(F("Unknown event: "));
		Serial.println((unsigned)ev);
		break;
	}
}

void printHex2(unsigned v)
{
	v &= 0xff;
	if (v < 16)
	{
		Serial.print('0');
	}
	Serial.print(v, HEX);
}

void do_send(osjob_t *j)
{
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND)
	{
		Serial.println(F("OP_TXRXPEND, not sending"));
		txComplete = true;
	}
	else
	{
		lora.setTxData();
		Serial.println(F("Packet queued"));
		txComplete = false;
	}
	// Next TX is scheduled after TX_COMPLETE event.
}

void do_send_mapping(osjob_t *j)
{
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND)
	{
		Serial.println(F("OP_TXRXPEND, not sending"));
		txComplete = true;
	}
	else
	{
		LoraParameter *lp = lora.getParameter(LoraParameter::Kind::gpsMinimal);
		if (lp != NULL)
		{
			LMIC_setTxData2(2, lp->GetData().data(), lp->GetData().size(), 0);
			Serial.println(F("Packet queued"));
			txComplete = false;
		}
		else
		{
			Serial.println(F("Error: couldn't find gpsMinimal parameter"));
			txComplete = true;
		}
	}
	// Next TX is scheduled after TX_COMPLETE event.
}

void LmicInit()
{
	// LMIC init
	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	// allow much more clock error than the X/1000 default. See:
	// https://github.com/mcci-catena/arduino-lorawan/issues/74#issuecomment-462171974
	// https://github.com/mcci-catena/arduino-lmic/commit/42da75b56#diff-16d75524a9920f5d043fe731a27cf85aL633
	// the X/1000 means an error rate of 0.1%; the above issue discusses using values up to 10%.
	// so, values from 10 (10% error, the most lax) to 1000 (0.1% error, the most strict) can be used.
	// LMIC_setClockError(1 * MAX_CLOCK_ERROR / 40);
	LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

	LMIC_setLinkCheckMode(0);
	LMIC_setDrTxpow(DR_SF7, 14);
	// LMIC_setupBand(BAND_MILLI,13,10);
	// LMIC_selectSubBand(1);
	Serial.println("Radio Initialized");

	// Start job (sending automatically starts OTAA too)
	// do_send(lora.getSendjob(1));
	LMIC_startJoining();
}