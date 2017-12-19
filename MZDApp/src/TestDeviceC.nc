/*
 * Copyright (c) 2010, CISTER/ISEP - Polytechnic Institute of Porto
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * - Neither the name of the Technische Universitaet Berlin nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 * 
 * 
 * @author Ricardo Severino <rars@isep.ipp.pt>
 * @author Stefano Tennina <sota@isep.ipp.pt>
 * ========================================================================
 */
#include "app_profile.h"
#include <TKN154.h>
#include <GTS.h>

module TestDeviceC {
	uses {
		interface Boot;
		interface MCPS_DATA;
		interface MLME_RESET;
		interface MLME_SET;
		interface MLME_GET;
		interface MLME_SCAN;
		interface MLME_SYNC;
		interface MLME_ASSOCIATE;
		interface MLME_BEACON_NOTIFY;
		interface MLME_SYNC_LOSS;
		interface IEEE154Frame as Frame;
		interface IEEE154BeaconFrame as BeaconFrame;
		interface Leds;
		interface Packet;
		interface MLME_GTS;
	}
}
implementation {
	message_t m_frame;
	uint8_t m_payloadLen;
	ieee154_PANDescriptor_t m_PANDescriptor;
	bool m_ledCount;
	bool m_wasScanSuccessful;
	void startApp();
	uint8_t m_messageCount;
	task void packetSendTask();
	norace uint8_t m_step;
	uint8_t m_gtsCharacteristics;
	ieee154_CapabilityInformation_t m_capabilityInformation;

	typedef struct {
		unsigned char message_type : 4;
		uint16_t node_id;
		unsigned char message_payload : 6;
	} gts_message_st;

	uint8_t GtsSetCharacteristics(uint8_t gtsLength, uint8_t gtsDirection,
			uint8_t characteristicType) {
		return((gtsLength << 0) | (gtsDirection << 4) | (characteristicType << 5));
	}

	void GtsGetCharacteristics(uint8_t gtsCharacteristics, uint8_t * gtsLength,
			bool * gtsDirection, bool * characteristicType) {
		*gtsLength = gtsCharacteristics & 0b00001111;
		*gtsDirection = (gtsCharacteristics & 0b00010000) >> 4;
		*characteristicType = (gtsCharacteristics & 0b00100000) >> 5;
	}

	event void Boot.booted() {
		char payload[] = "GTS TEST!";
		uint8_t * payloadRegion;
		m_payloadLen = strlen(payload);
		payloadRegion = call Packet.getPayload(&m_frame, m_payloadLen);
		if(m_payloadLen <= call Packet.maxPayloadLength()) {
			memcpy(payloadRegion, payload, m_payloadLen);
			call MLME_RESET.request(TRUE);
		}
		m_capabilityInformation.AlternatePANCoordinator = 0;
		m_capabilityInformation.DeviceType = 0;
		m_capabilityInformation.PowerSource = 0;
		m_capabilityInformation.ReceiverOnWhenIdle = 0;
		m_capabilityInformation.Reserved = 0;
		m_capabilityInformation.SecurityCapability = 0;
		m_capabilityInformation.AllocateAddress = 1;
	}

	event void MLME_RESET.confirm(ieee154_status_t status) {
		if(status == IEEE154_SUCCESS) 
			startApp();
	}

	void startApp() {
		ieee154_phyChannelsSupported_t channelMask;
		uint8_t scanDuration = BEACON_ORDER;
		call MLME_SET.macShortAddress(TOS_NODE_ID);
		// scan only the channel where we expect the coordinator
		channelMask = ((uint32_t) 1) << RADIO_CHANNEL;
		// we want all received beacons to be signalled 
		// through the MLME_BEACON_NOTIFY interface, i.e.
		// we set the macAutoRequest attribute to FALSE
		call MLME_SET.macAutoRequest(FALSE);
		m_wasScanSuccessful = FALSE;
		call MLME_SCAN.request(PASSIVE_SCAN, // ScanType
		channelMask, // ScanChannels
		scanDuration, // ScanDuration
		0x00, // ChannelPage
		0, // EnergyDetectListNumEntries
		NULL, // EnergyDetectList
		0, // PANDescriptorListNumEntries
		NULL, // PANDescriptorListGtsSetCharacteristics
		0 // security
		);
	}

	event message_t * MLME_BEACON_NOTIFY.indication(message_t * frame) {
		void * beacon_payload = NULL;
		gts_message_st * gts_request;
		uint8_t gtsLength = 0;
		bool gtsDirection = 0;
		bool characteristicType = 0;
		uint8_t message_type;
		// received a beacon frame //
		ieee154_phyCurrentPage_t page = call MLME_GET.phyCurrentPage();
		ieee154_macBSN_t beaconSequenceNumber = call BeaconFrame.getBSN(frame);
				call Leds.led2On();
		if( ! m_wasScanSuccessful) {
			// received a beacon during channel scanning
			if(call BeaconFrame.parsePANDescriptor(frame, RADIO_CHANNEL, page, 
					&m_PANDescriptor) == SUCCESS) {
				// let's see if the beacon is from our coordinator...
				if(m_PANDescriptor.CoordAddrMode == ADDR_MODE_SHORT_ADDRESS & 
						& m_PANDescriptor.CoordPANId == PAN_ID && m_PANDescriptor.CoordAddress
						.shortAddress == COORDINATOR_ADDRESS) {
					// yes! wait until SCAN is finished, then syncronize to the beacons
					m_wasScanSuccessful = TRUE;

					printf("Beacon recebido: \n");

					beacon_payload_length = call BeaconFrame.getBeaconPayloadLength(frame);
					beacon_payload = call BeaconFrame.getBeaconPayload(frame);
					gts_request = (gts_message_st * ) beacon_payload;
					message_type = gts_request->message_type;
					m_gtsCharacteristics = (uint8_t) ((gts_request->message_payload && 11111100)>>2);
					if((message_type >> 4) == 4) {
						printf("Tipo de mensagem: Requisição de GTS (0100) \n");
						if(gts_request->node_id== TOS_NODE_ID){
						GtsGetCharacteristics(m_gtsCharacteristics, &gtsLength, &gtsDirection, 
								&characteristicType);
						printf("Nodo: %d\n"
						"Tamanho do GTS: %u \n"
						"Sentido: %d \n"
						"Característica: %d \n", TOS_NODE_ID, gtsLength, gtsDirection, characteristicType);
						printf("ID do nodo atual é igual a ID da solicitação de GTS \n");
						printf("Enviado requisição de GTS para o Coordenador PAN \n");
						call MLME_GTS.request(m_gtsCharacteristics, 0); //send the gts slot request
						}
					}
				}
			}
		}
		call Leds.led2Off();
		return frame;
	}

	event void MLME_SCAN.confirm(ieee154_status_t status, uint8_t ScanType,
			uint8_t ChannelPage, uint32_t UnscannedChannels,
			uint8_t EnergyDetectListNumEntries, int8_t * EnergyDetectList,
			uint8_t PANDescriptorListNumEntries, ieee154_PANDescriptor_t 
			* PANDescriptorList) {
		if(m_wasScanSuccessful) {
			// we received a beacon from the coordinator before
			call MLME_SET.macCoordShortAddress(m_PANDescriptor.CoordAddress
					.shortAddress);
			call MLME_SET.macPANId(m_PANDescriptor.CoordPANId);
			call MLME_SYNC.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor
					.ChannelPage, TRUE);
			call Frame.setAddressingFields(&m_frame, ADDR_MODE_SHORT_ADDRESS, // SrcAddrMode,
			ADDR_MODE_SHORT_ADDRESS, // DstAddrMode,
			m_PANDescriptor.CoordPANId, // DstPANId,
			&m_PANDescriptor.CoordAddress, // DstAddr,
			NULL // security
			);
			call MLME_ASSOCIATE.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor
					.ChannelPage, m_PANDescriptor.CoordAddrMode, m_PANDescriptor.CoordPANId,
					m_PANDescriptor.CoordAddress, m_capabilityInformation, NULL // security
			);
		}
		else 
			startApp();
	}

	task void packetSendTask() {
		if( ! m_wasScanSuccessful) 
			return;
		if(m_step == 1) {
			call MCPS_DATA.request(&m_frame, // frame,
			m_payloadLen, // payloadLength,
			0, // msduHandle,
			TX_OPTIONS_ACK | TX_OPTIONS_GTS // TxOptions,
			);
		} else {
			call MCPS_DATA.request(&m_frame, // frame,
			m_payloadLen, // payloadLength,
			0, // msduHandle,
			TX_OPTIONS_ACK // TxOptions,
			);
		}
	}

	event void MCPS_DATA.confirm(message_t * msg, uint8_t msduHandle,
			ieee154_status_t status, uint32_t timestamp) {
	}

	event void MLME_SYNC_LOSS.indication(ieee154_status_t lossReason,
			uint16_t PANId, uint8_t LogicalChannel, uint8_t ChannelPage,
			ieee154_security_t * security) {
		m_wasScanSuccessful = FALSE;
		call Leds.led1Off();
		call Leds.led2Off();
		call Leds.led0On();

		m_messageCount = 0;
		m_step = 0;
		startApp();
	}

	event message_t * MCPS_DATA.indication(message_t * frame) {

		return frame;
	}

	event void MLME_GTS.indication(uint16_t DeviceAddress,
			uint8_t GtsCharacteristics, ieee154_security_t * security) {
	}

	event void MLME_GTS.confirm(uint8_t GtsCharacteristics,
			ieee154_status_t status) {
		m_step = 1;
		if(status == IEEE154_SUCCESS && m_step == 0) {
			call Leds.led0On();
		}
	}
	event void MLME_ASSOCIATE.indication(uint64_t DeviceAddress,
			ieee154_CapabilityInformation_t CapabilityInformation, ieee154_security_t 
			* security) {
	}
	event void MLME_ASSOCIATE.confirm(uint16_t AssocShortAddress, uint8_t status,
			ieee154_security_t * security) {
		if(status == IEEE154_SUCCESS) {
			// we are now associated -
			call Leds.led1On();
		}
		else {
			call MLME_ASSOCIATE.request(m_PANDescriptor.LogicalChannel, m_PANDescriptor
					.ChannelPage, m_PANDescriptor.CoordAddrMode, m_PANDescriptor.CoordPANId,
					m_PANDescriptor.CoordAddress, m_capabilityInformation, NULL // security
			);
		}
	}
}