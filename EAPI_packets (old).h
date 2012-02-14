#pragma once

#define MAX_PAYLOAD				256

/* Packet Identifiers (uint16_t values) */
#define EAPI_INPKT_HOME			(0x0040)
#define EAPI_INPKT_STATUS		(0x0041)
#define EAPI_INPKT_DISABLE		(0x0042)	
#define EAPI_INPKT_ENABLE_FC	(0x0043)
#define EAPI_INPKT_DATA_FC		(0x0044)
#define EAPI_INPKT_GET_DATA		(0x0045)
#define EAPI_INPKT_SET_FREQ		(0x0046)
#define EAPI_INPKT_DISCOVER		(0x0047)

#define EAPI_OUTPKT_DATA_FC		(0x0080)
#define EAPI_OUTPKT_NACK		(0x0081)
#define EAPI_OUTPKT_STATUS		(0x0082)
#define EAPI_OUTPKT_DATA		(0x0083)
#define EAPI_OUTPKT_DISCOVER	(0x0084)

/* NACK codes (uint8_t values */
#define NACK_NULL_PACKET		(0x00)
#define NACK_LENGTH_ERROR		(0x01)
#define NACK_STATE_CHANGE_REQ	(0x02)
#define NACK_UNKNOWN_PACKET		(0x03)

#pragma pack(push,1)
typedef struct {
    uint16_t cmd;
    uint16_t len;
    uint8_t  payload[MAX_PAYLOAD];
} udp_reply_pkt_t;
#pragma pack(pop)

