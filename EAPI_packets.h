// --------------------------------------------------------------------------------------
// Module     : EAPI_PACKETS
// Author     : R. Leslie
// --------------------------------------------------------------------------------------
// Entact Robotics inc.
// --------------------------------------------------------------------------------------

#ifndef EAPI_PACKETS_H
#define EAPI_PACKETS_H

#include <stdint.h>

#define DEV_NAME_LEN	(32)

#pragma pack(push)
#pragma pack(1)

// general packet structure
typedef struct
{
    uint16_t cmd;
    uint16_t len;
    uint8_t payload[256];
} udp_pkt_t;

// EAPI_INPKT_DATA_FC packet structure
typedef struct
{
    uint16_t cmd;
    uint16_t len;
    float task_force[8];
} udp_datafc_pkt_t;

// EAPI_INPKT_DATA_PC packet structure
typedef struct
{
    uint16_t cmd;
    uint16_t len;
    float task_position[12];
} udp_datapc_pkt_t;

// EAPI_OUTPKT_STATUS packet structure
typedef struct
{
    uint16_t cmd;
    uint16_t len;
    uint8_t device_type[DEV_NAME_LEN];
    uint8_t device_name[DEV_NAME_LEN];
    uint32_t serial_no;
    uint32_t state;
    uint32_t homed;
    uint32_t faultvector;
} udp_status_pkt_t;

// EAPI_OUTPKT_DATA packet structure
typedef struct
{
    uint16_t cmd;
    uint16_t len;
    float pos[3];
    float orr[9];
    float posdot[3];
    float omega[3];
} udp_outdata_pkt_t;

#pragma pack(pop)

/* packet cmd valid fields (uint16_t values) */
#define EAPI_INPKT_HOME			(0x0040)
#define EAPI_INPKT_STATUS		(0x0041)
#define EAPI_INPKT_DISABLE		(0x0042)	
#define EAPI_INPKT_ENABLE_FC	(0x0043)
#define EAPI_INPKT_DATA_FC		(0x0044)
#define EAPI_INPKT_GET_DATA		(0x0045)
#define EAPI_INPKT_SET_FREQ		(0x0046)
#define EAPI_INPKT_DISCOVER		(0x0047)
#define EAPI_INPKT_ENABLE_TC	(0x0048)
#define EAPI_INPKT_DATA_TC		(0x0049)
#define EAPI_INPKT_DISPLAY_ENC	(0x004A)
#define EAPI_INPKT_SET_DAMPING	(0x004B)
#define EAPI_INPKT_ENABLE_PC	(0x004C)
#define EAPI_INPKT_DATA_PC		(0x004D)

#define EAPI_OUTPKT_DATA		(0x0080)
#define EAPI_OUTPKT_NACK		(0x0081)
#define EAPI_OUTPKT_STATUS		(0x0082)
#define EAPI_OUTPKT_DISCOVER	(0x0083)
#define EAPI_OUTPKT_ACK			(0x0084)

/* NACK codes (uint8_t values */
#define NACK_NULL_PACKET		(0x00)
#define NACK_LENGTH_ERROR		(0x01)
#define NACK_STATE_CHANGE_REQ	(0x02)
#define NACK_UNKNOWN_PACKET		(0x03)

/* ACK codes (uint8_t values */
#define ACK_STATE_CHANGED		(0x00)
#define ACK_HOMED				(0x01)
#define ACK_FREQ_CHANGED		(0x02)

#endif // EAPI_PACKETS_H