// EntactAPI.cpp : Defines the exported functions for the DLL application.
//

#ifdef UNICODE
	#define	Seeifdef	Unicode
#else
	#define Seeifdef	Ansi
#endif

#if defined(_MSC_VER)	// Microsoft compiler
	#define WIN32_LEAN_AND_MEAN
  #include <windows.h>
	#include <winsock2.h>
	#include <ws2tcpip.h>
	#pragma comment(lib,"ws2_32.lib")
#elif defined(__GNUC__) // GNU compiler
  #include <dlfcn.h>
	#include <sys/socket.h>
	#include <arpa/inet.h>
#else
	#error define your compiler
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <iostream>

#include "EntactAPI.h"
#include "EAPI_packets.h"
#include "EAPI_kinematics.h"

/******************************************************************************* 
Defines
********************************************************************************/
#define REMOTE_PORT		(55556)
#define LOCAL_PORT		(55555)
#define RECV_TIMOUT		(100)			// milli-seconds (different in linux???)
#define MAX_N_DEVICES	(16)
#define MAX_Q_ELEMENTS	(8)

/******************************************************************************* 
Global Data 
********************************************************************************/
typedef struct {
	int serialno;							// The Serial Number of the device
	char productname[256];					// The devices product name
	char name[256];							// The devices user defined name
} device_info_t;

typedef struct {
	device_info_t device_info;									// the devices information
	struct sockaddr_in addr;									// server information for device
	double q[MAX_Q_ELEMENTS];									// joint angles in radians
	double qd[MAX_Q_ELEMENTS];									// joint velocity in radians/s
	double pos[3];														// task position (X,Y,Z)
	double or[9];															// task orientation (Or Matrix)
	double posdot[3];													// task velocity (Xd,Yd,Zd)
	double omega[3];													// rotational velocity
	int (*pt2forwardKin)(double*,double*);						// forward kinematics function pointer
	void (*pt2eventCB)(unsigned char);							// device disabled event, callback function pointer
} EAPI_device;

// Device info structures; fixed to MAX_N_DEVICES at the moment (figure out how to do this dynamically in the future)
EAPI_device deviceList[MAX_N_DEVICES]; 

// function prototypes
static int init_udp();
static int uninit_udp();
static int discover_devices(int *n_devices);

// UDP sockets
#if defined(_MSC_VER)	// Microsoft compiler
	WSADATA w;
	SOCKET send_sd, recv_sd;
#elif defined(__GNUC__) // GNU compiler
    int send_sd, recv_sd;
#else
	#error define your compiler
#endif

sockaddr_in local_addr;

// API state information
int api_init = 0;		// API had been initialized
int n_devices = 0;		// Number of devices discovered during initialization

/******************************************************************************* 
DLL (or SO) Entry Point 
********************************************************************************/
#if defined(_MSC_VER)
	#pragma unmanaged
     bool APIENTRY DllMain( HANDLE hModule, 
							DWORD  ul_reason_for_call, 
							LPVOID lpReserved
						)
 #elif defined(__GNUC__)
       bool DllMain(	void * hModule, 
						unsigned long  ul_reason_for_call, 
                        void *lpReserved
                  )
 #endif 
{
	switch (ul_reason_for_call)
	{
	case 1:	// DLL_PROCESS_ATTACH:
	case 2:	// DLL_THREAD_ATTACH:
	case 3:	// DLL_THREAD_DETACH:
	case 0: // DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

/******************************************************************************* 
API Public Functions 
********************************************************************************/

ENTACTAPI_API int openEAPI(eapi_device_handle handles[], int size)
{
	int i;

	// if the api has already been initialized from another thread
	if (api_init)
	{
		// provide users handle array with pointers to already discovered devices
		i = 0;
		while (i < n_devices)
		{
			if (i < size) handles[i] = (void *) &deviceList[i];
			i++;
		}
		return n_devices;
	}

	// if we are here the api had not been initialized
	// we will carry out the process of discovering entact
	// devices on the network

	// clear the deviceList[] structure of data
	memset(deviceList, 0, sizeof(deviceList));

	// initialize the api's udp socket
	if (!init_udp()) 
	{
		// udp socket initialization failed
		return EAPI_ERR; 
	}
	
	// broadcast onto the local network in search of entact devices
	// fill the deviceList[] array with IP/port and device information
	if (!discover_devices(&n_devices))
	{
		// failed to discover devices
		return EAPI_ERR; 
	}

	// provide users handle array with pointers to newly discovered devices
	i = 0;
	while (i < n_devices)
	{
		if (i < size) handles[i] = (void *) &deviceList[i];
		i++;
	}

	// enable access to all other API functions
	api_init = 1;

	return n_devices;
}

ENTACTAPI_API int closeEAPI()
{
	// has the api first been opened?
	if (!api_init) return EAPI_ERR;

	api_init = 0;

	// clear the deviceList[] structure of data
	memset(deviceList, 0, sizeof(deviceList));

	// close UDP sockets
	if (!uninit_udp()) return EAPI_ERR; 

	return EAPI_OK;
}

ENTACTAPI_API int setSampleRateEAPI(eapi_device_handle handle, int frequency)
{
	udp_pkt_t packet, ackpacket;
	uint32_t freq = frequency;
	int pkt_size, rec_len;

	if (!api_init) return EAPI_ERR;

	packet.cmd = EAPI_INPKT_SET_FREQ;
	packet.len = sizeof(freq);
	memcpy((void*) packet.payload, (void*) &freq, sizeof(frequency));
	pkt_size = sizeof(packet) - sizeof(packet.payload) + packet.len;
	if(sendto(send_sd, (const char*) &packet, pkt_size, 0, (struct sockaddr *) &((EAPI_device *) handle)->addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return EAPI_ERR;

	// acknowledgement packet here to verify 
	rec_len = recvfrom(recv_sd, (char*) &ackpacket, sizeof(ackpacket), 0, NULL, NULL);
	if (rec_len == SOCKET_ERROR) return EAPI_ERR;

	if ((ackpacket.cmd == EAPI_OUTPKT_ACK)&&(ackpacket.payload[0] == ACK_FREQ_CHANGED))
	{
		return EAPI_OK;
	}
	else
	{
		return EAPI_ERR;
	}

	return EAPI_ERR;
}

ENTACTAPI_API int setEventCallbackEAPI(eapi_device_handle handle, void (*cbFunc) (unsigned char eventArg))
{
	if (!api_init) return EAPI_ERR;

	((EAPI_device*)handle)->pt2eventCB = cbFunc;

	// NOTE: Not implemented at the moment.
	// would like to add a monitor thread which requests device status at a low frequency interval
	// if faults or state changes occur the users callback function is executed so the user can be aware in real-time

	return 1;
}

ENTACTAPI_API int setModeEAPI(eapi_device_handle handle, int mode)
{
	udp_pkt_t packet, ackpacket;
	int pkt_size, rec_len;

	if (!api_init) return EAPI_ERR;

	packet.cmd = EAPI_INPKT_DISABLE; // default to this mode (if mode parameter is invalid)
	if (mode == EAPI_DISABLED_MODE) packet.cmd = EAPI_INPKT_DISABLE;	
	if (mode == EAPI_FORCECONTROL_MODE)	packet.cmd = EAPI_INPKT_ENABLE_FC;
	if (mode == EAPI_TORQUECONTROL_MODE) packet.cmd = EAPI_INPKT_ENABLE_TC;	

	packet.len = 0;
	pkt_size = sizeof(packet) - sizeof(packet.payload) + packet.len;
	if(sendto(send_sd, (const char*) &packet, pkt_size, 0, (struct sockaddr *) &((EAPI_device *) handle)->addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return EAPI_ERR;
	
	rec_len = recvfrom(recv_sd, (char*) &ackpacket, sizeof(ackpacket), 0, NULL, NULL);
	if (rec_len == SOCKET_ERROR) return EAPI_ERR;

	if ((ackpacket.cmd == EAPI_OUTPKT_ACK)&&(ackpacket.payload[0] == ACK_STATE_CHANGED))
	{
		return EAPI_OK;
	}
	else
	{
		printf("here %d %d\n",ackpacket.cmd,ackpacket.payload[0]);
		return EAPI_ERR;
	}

	return EAPI_ERR;
}

ENTACTAPI_API int getModeEAPI(eapi_device_handle handle)
{
	udp_pkt_t outpkt;
	udp_status_pkt_t statuspkt;
	int pkt_size, rec_len;

	if (!api_init) return EAPI_ERR;

	// send a status request packet
	outpkt.cmd = EAPI_INPKT_STATUS;
	outpkt.len = 0;
	pkt_size = sizeof(outpkt) - sizeof(outpkt.payload) + outpkt.len;
	if(sendto(send_sd, (const char*) &outpkt, pkt_size, 0, (struct sockaddr *) &((EAPI_device *) handle)->addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return EAPI_ERR;

	// recvfrom() status information for the device
	rec_len = recvfrom(recv_sd, (char*) &statuspkt, sizeof(statuspkt), 0, NULL, NULL);
	if (rec_len == SOCKET_ERROR) return EAPI_ERR;
	if (statuspkt.cmd != EAPI_OUTPKT_STATUS) return EAPI_ERR;
	if (rec_len != sizeof(statuspkt)) return EAPI_ERR;

	if (statuspkt.state == 0) return EAPI_DISABLED_MODE; 
	if (statuspkt.state == 2) return EAPI_FORCECONTROL_MODE;
	if (statuspkt.state == 4) return EAPI_TORQUECONTROL_MODE;

	return EAPI_ERR;
}

ENTACTAPI_API int homeDeviceEAPI(eapi_device_handle handle)
{
	udp_pkt_t packet, ackpacket;
	udp_outdata_pkt_t datapacket;
	int pkt_size, rec_len;

	if (!api_init) return EAPI_ERR;

	packet.cmd = EAPI_INPKT_HOME;
	packet.len = 0;
	pkt_size = sizeof(packet) - sizeof(packet.payload) + packet.len;
	if(sendto(send_sd, (const char*) &packet, pkt_size, 0, (struct sockaddr *) &((EAPI_device *) handle)->addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return EAPI_ERR;

	// acknowledgement packet here to verify 
	rec_len = recvfrom(recv_sd, (char*) &ackpacket, sizeof(ackpacket), 0, NULL, NULL);
	if (rec_len == SOCKET_ERROR) return EAPI_ERR;

	if ((ackpacket.cmd == EAPI_OUTPKT_ACK)&&(ackpacket.payload[0] == ACK_HOMED))
	{
		// calibration has been successfull, now updating the joint values for the device
		packet.cmd = EAPI_INPKT_GET_DATA;
		packet.len = 0;
		pkt_size = sizeof(packet) - sizeof(packet.payload) + packet.len;
		if(sendto(send_sd, (const char*) &packet, pkt_size, 0, (struct sockaddr *) &((EAPI_device *) handle)->addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return EAPI_ERR;

		// recvfrom() data packet with joint values
		rec_len = recvfrom(recv_sd, (char*) &datapacket, sizeof(datapacket), 0, NULL, NULL);
		if (rec_len == SOCKET_ERROR) return EAPI_ERR;
		if (datapacket.cmd != EAPI_OUTPKT_DATA) return EAPI_ERR;

		//((EAPI_device *) handle)->q[0] = datapacket.q[0];
		//((EAPI_device *) handle)->q[1] = datapacket.q[1];
		//((EAPI_device *) handle)->q[2] = datapacket.q[2];
		//((EAPI_device *) handle)->q[3] = datapacket.q[3];
		//((EAPI_device *) handle)->q[4] = datapacket.q[4];
		//((EAPI_device *) handle)->q[5] = datapacket.q[5];
		//((EAPI_device *) handle)->q[6] = datapacket.q[6];
		//((EAPI_device *) handle)->q[7] = datapacket.q[7];

		//((EAPI_device *) handle)->qd[0] = datapacket.qd[0];
		//((EAPI_device *) handle)->qd[1] = datapacket.qd[1];
		//((EAPI_device *) handle)->qd[2] = datapacket.qd[2];
		//((EAPI_device *) handle)->qd[3] = datapacket.qd[3];
		//((EAPI_device *) handle)->qd[4] = datapacket.qd[4];
		//((EAPI_device *) handle)->qd[5] = datapacket.qd[5];
		//((EAPI_device *) handle)->qd[6] = datapacket.qd[6];
		//((EAPI_device *) handle)->qd[7] = datapacket.qd[7];
		
		return EAPI_OK;
	}
	else
	{
		return EAPI_ERR;
	} 

	return EAPI_ERR;
}

ENTACTAPI_API int isHomedEAPI(eapi_device_handle handle)
{
	int homed = 0;
	udp_pkt_t outpkt;
	udp_status_pkt_t statuspkt;
	int pkt_size, rec_len;

	if (!api_init) return EAPI_ERR;

	// send a status request packet
	outpkt.cmd = EAPI_INPKT_STATUS;
	outpkt.len = 0;
	pkt_size = sizeof(outpkt) - sizeof(outpkt.payload) + outpkt.len;
	if(sendto(send_sd, (const char*) &outpkt, pkt_size, 0, (struct sockaddr *) &((EAPI_device *) handle)->addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return EAPI_ERR;

	// recvfrom() status information for the device
	rec_len = recvfrom(recv_sd, (char*) &statuspkt, sizeof(statuspkt), 0, NULL, NULL);
	if (rec_len == SOCKET_ERROR) return EAPI_ERR;
	if (statuspkt.cmd != EAPI_OUTPKT_STATUS) return EAPI_ERR;
	if (rec_len != sizeof(statuspkt)) return EAPI_ERR;

	if(statuspkt.homed == 1)
	{
		homed = 1;
	}

	return homed;
}

ENTACTAPI_API int readJointsEAPI(eapi_device_handle handle, double *pos, double *vel, int size)
{
	static udp_outdata_pkt_t inpkt;
	static udp_pkt_t outpkt;
	int pkt_size, rec_len;
	
	if (!api_init) return EAPI_ERR;

	// send a status request packet
	outpkt.cmd = EAPI_INPKT_GET_DATA;
	outpkt.len = 0;
	pkt_size = sizeof(outpkt) - sizeof(outpkt.payload) + outpkt.len;
	if(sendto(send_sd, (const char*) &outpkt, pkt_size, 0, (struct sockaddr *) &((EAPI_device *) handle)->addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return EAPI_ERR;

	
	// Receive back a reply packet (should be with encoder data)
	rec_len = recvfrom(recv_sd, (char*) &inpkt, sizeof(inpkt), 0, NULL, NULL);
	if (rec_len == SOCKET_ERROR) return EAPI_ERR;
	if (inpkt.cmd != EAPI_OUTPKT_DATA) return EAPI_ERR;
	if (rec_len != sizeof(udp_outdata_pkt_t)) return EAPI_ERR;

	//((EAPI_device *) handle)->q[0] = inpkt.q[0]; pos[0] = inpkt.q[0];
	//((EAPI_device *) handle)->q[1] = inpkt.q[1]; pos[1] = inpkt.q[1];
	//((EAPI_device *) handle)->q[2] = inpkt.q[2]; pos[2] = inpkt.q[2];
	//((EAPI_device *) handle)->q[3] = inpkt.q[3]; pos[3] = inpkt.q[3];
	//((EAPI_device *) handle)->q[4] = inpkt.q[4]; pos[4] = inpkt.q[4];
	//((EAPI_device *) handle)->q[5] = inpkt.q[5]; pos[5] = inpkt.q[5];
	//((EAPI_device *) handle)->q[6] = inpkt.q[6]; pos[6] = inpkt.q[6];
	//((EAPI_device *) handle)->q[7] = inpkt.q[7]; pos[7] = inpkt.q[7];

	//((EAPI_device *) handle)->qd[0] = inpkt.qd[0]; vel[0] = inpkt.qd[0];
	//((EAPI_device *) handle)->qd[1] = inpkt.qd[1]; vel[1] = inpkt.qd[1];
	//((EAPI_device *) handle)->qd[2] = inpkt.qd[2]; vel[2] = inpkt.qd[2];
	//((EAPI_device *) handle)->qd[3] = inpkt.qd[3]; vel[3] = inpkt.qd[3];
	//((EAPI_device *) handle)->qd[4] = inpkt.qd[4]; vel[4] = inpkt.qd[4];
	//((EAPI_device *) handle)->qd[5] = inpkt.qd[5]; vel[5] = inpkt.qd[5];
	//((EAPI_device *) handle)->qd[6] = inpkt.qd[6]; vel[6] = inpkt.qd[6];
	//((EAPI_device *) handle)->qd[7] = inpkt.qd[7]; vel[7] = inpkt.qd[7];

	return EAPI_OK;
}

ENTACTAPI_API int readTaskPositionEAPI(eapi_device_handle handle, double *taskpos, int size)
{
	//double task_temp[12];
	int ret_val;

	if (!api_init) return EAPI_ERR;
	
	// if in a real-time state 
	// this is calculated using the encoder positions returned at the time of the previous force/torque command

	//printf("joint 7 %3.3f /n", ((EAPI_device *) handle)->q[6]);
	//printf("here");

	// commented out for T3 testing (sept 29th)
	ret_val = ((EAPI_device *) handle)->pt2forwardKin(((EAPI_device *) handle)->q, taskpos);

	//ret_val = forwardKinematics_t3left(((EAPI_device *) handle)->q, task_temp);


		if (size >=12)
		{
			taskpos[0] = ((EAPI_device *) handle)->pos[0]; // X
			taskpos[1] = ((EAPI_device *) handle)->pos[1]; // Y
			taskpos[2] = ((EAPI_device *) handle)->pos[2]; // Z

			taskpos[3] = ((EAPI_device *) handle)->or[0]; // R11
			taskpos[4] = ((EAPI_device *) handle)->or[1]; // R12
			taskpos[5] = ((EAPI_device *) handle)->or[2]; // R13
			taskpos[6] = ((EAPI_device *) handle)->or[3]; // R21
			taskpos[7] = ((EAPI_device *) handle)->or[4]; // R22
			taskpos[8] = ((EAPI_device *) handle)->or[5]; // R23
			taskpos[9] = ((EAPI_device *) handle)->or[6]; // R31
			taskpos[10] = ((EAPI_device *) handle)->or[7]; // R32
			taskpos[11] = ((EAPI_device *) handle)->or[8]; // R33

			ret_val = 1;
		}
		else
		{
			ret_val = EAPI_ERR;
		}
	// if in a non-real-time state
	// this is calculated by querying the device at this moment for its encoder positions

	// - Not checking for this case at the moment

	return ret_val;
}

ENTACTAPI_API int readTaskVelocityEAPI(eapi_device_handle handle, double *taskvel, int size)
{
	int ret_val;

	if (!api_init) return EAPI_ERR;
	
	if (size >=6)
		{
			taskvel[0] = ((EAPI_device *) handle)->posdot[0]; // X dot
			taskvel[1] = ((EAPI_device *) handle)->posdot[1]; // Y dot
			taskvel[2] = ((EAPI_device *) handle)->posdot[2]; // Z dot

			taskvel[3] = ((EAPI_device *) handle)->omega[0]; // omega x
			taskvel[4] = ((EAPI_device *) handle)->omega[1]; // omega y
			taskvel[5] = ((EAPI_device *) handle)->omega[2]; // omega z

			ret_val = 1;
		}
		else
		{
			ret_val = EAPI_ERR;
		}

	return ret_val;
}

ENTACTAPI_API int writeForceEAPI(eapi_device_handle handle, double *f, int size)
{
	static udp_datafc_pkt_t packet;
	static udp_outdata_pkt_t inpkt;
	int pkt_size, i, rec_len;

	if (!api_init) return EAPI_ERR;
	if ((size<0)||(size>sizeof(packet.task_force))) return EAPI_ERR;

	packet.cmd = EAPI_INPKT_DATA_FC;
	packet.len = size*sizeof(float);
	for (i=0; i<size; i++)
	{
		packet.task_force[i] = (float) f[i];
	}

	pkt_size = sizeof(packet) - sizeof(packet.task_force) + packet.len;
	if(sendto(send_sd, (const char*) &packet, pkt_size, 0, (struct sockaddr *) &((EAPI_device *) handle)->addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return EAPI_ERR;

	// Receive back a reply packet (should be with encoder data)
	rec_len = recvfrom(recv_sd, (char*) &inpkt, sizeof(inpkt), 0, NULL, NULL);
	if (rec_len == SOCKET_ERROR) return EAPI_ERR;
	if (inpkt.cmd != EAPI_OUTPKT_DATA) return EAPI_ERR;
	if (rec_len != sizeof(udp_outdata_pkt_t)) return EAPI_ERR;

	((EAPI_device *) handle)->pos[0] = inpkt.pos[0];
	((EAPI_device *) handle)->pos[1] = inpkt.pos[1];
	((EAPI_device *) handle)->pos[2] = inpkt.pos[2];

	((EAPI_device *) handle)->posdot[0] = inpkt.posdot[0];
	((EAPI_device *) handle)->posdot[1] = inpkt.posdot[1];
	((EAPI_device *) handle)->posdot[2] = inpkt.posdot[2];

	((EAPI_device *) handle)->or[0] = inpkt.orr[0];
	((EAPI_device *) handle)->or[1] = inpkt.orr[1];
	((EAPI_device *) handle)->or[2] = inpkt.orr[2];
	((EAPI_device *) handle)->or[3] = inpkt.orr[3];
	((EAPI_device *) handle)->or[4] = inpkt.orr[4];
	((EAPI_device *) handle)->or[5] = inpkt.orr[5];
	((EAPI_device *) handle)->or[6] = inpkt.orr[6];
	((EAPI_device *) handle)->or[7] = inpkt.orr[7];
	((EAPI_device *) handle)->or[8] = inpkt.orr[8];

	((EAPI_device *) handle)->omega[0] = inpkt.omega[0];
	((EAPI_device *) handle)->omega[1] = inpkt.omega[1];
	((EAPI_device *) handle)->omega[2] = inpkt.omega[2];

	return EAPI_OK;
}


/******************************************************************************* 
API Private Functions 
********************************************************************************/

// returns 1 on success, returns 0 on failure
static int init_udp()
{
	#if defined(_MSC_VER)	// Microsoft compiler
	if (WSAStartup(0x0101, &w) != NO_ERROR) return 0;
	#endif

	send_sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (send_sd == INVALID_SOCKET) return 0;
	
	
	recv_sd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (recv_sd == INVALID_SOCKET) return 0;
	
	local_addr.sin_family = AF_INET;
	local_addr.sin_port = htons(LOCAL_PORT);
	local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(recv_sd, (const sockaddr *) &local_addr, sizeof(sockaddr_in)) == SOCKET_ERROR) return 0;
	
	return 1;
}

// returns 1 on success, returns 0 on failure
static int uninit_udp()
{
	#if defined(_MSC_VER)	// Microsoft compiler
	if (WSACleanup() == SOCKET_ERROR) return 0;
	#endif

	return 1;
}

// returns 1 on success, returns 0 on failure
static int discover_devices(int *n_devices)
{
    sockaddr_in device_address, broadcast_address;
	BOOL bOptVal;

	DWORD recv_timout = RECV_TIMOUT; // milli-seconds
	
	udp_pkt_t outpkt;
	udp_status_pkt_t statuspkt;
	int pkt_size, rec_len, sockaddr_size, valid_msg, dev_found;

	// temporarily setting the send socket to broadcast mode
	bOptVal = TRUE;
	if (setsockopt(send_sd, SOL_SOCKET, SO_BROADCAST, (char *) &bOptVal, sizeof(BOOL)) ==  SOCKET_ERROR) return 0;

	// setting the recv socket with a timeout
	if (setsockopt(recv_sd, SOL_SOCKET, SO_RCVTIMEO, (char *) &recv_timout, sizeof(DWORD)) ==  SOCKET_ERROR) return 0;
	
	// broadcast sendto()
	memset(&broadcast_address, '\0', sizeof(struct sockaddr_in));
    broadcast_address.sin_family = AF_INET;
    broadcast_address.sin_port = htons(REMOTE_PORT);
    broadcast_address.sin_addr.s_addr = htonl(INADDR_BROADCAST); 
	// send a status request packet
	outpkt.cmd = EAPI_INPKT_STATUS;
	outpkt.len = 0;
	pkt_size = sizeof(outpkt) - sizeof(outpkt.payload) + outpkt.len;
    if(sendto(send_sd, (const char*) &outpkt, pkt_size, 0, (sockaddr *) &broadcast_address, sizeof(sockaddr_in)) == SOCKET_ERROR) return 0;
	
	// loop through all the packets we've received back
	valid_msg = 1;
	dev_found = 0;
	while (valid_msg)
	{
		//printf("here \n");
		sockaddr_size = sizeof(struct sockaddr);
		rec_len = recvfrom(recv_sd, (char*) &statuspkt, sizeof(statuspkt), 0, (struct sockaddr *) &device_address, &sockaddr_size);
		if (rec_len != SOCKET_ERROR)
		{
			// recording the IP address and port information for the device
			deviceList[dev_found].addr.sin_port = device_address.sin_port;
			deviceList[dev_found].addr.sin_addr.S_un.S_addr = device_address.sin_addr.S_un.S_addr;
			deviceList[dev_found].addr.sin_family = AF_INET;

			// recording the product/name/serialno for the device
			memcpy(deviceList[dev_found].device_info.productname, statuspkt.device_type, sizeof(statuspkt.device_type));
			memcpy(deviceList[dev_found].device_info.name, statuspkt.device_name, sizeof(statuspkt.device_name));
			deviceList[dev_found].device_info.serialno = statuspkt.serial_no;

			std::cout << "forwardKinematics" << std::endl;
			printf("forwardKinematics\n");
			// map the proper kinematics functions for the device
			// - at the moment we only have W5D's, so these are automatically mapped
			deviceList[dev_found].pt2forwardKin = forwardKinematics_w5d;
			deviceList[dev_found].pt2eventCB = NULL;

			dev_found++;
		}
		else
		{
			valid_msg = 0;
		}
	} 
	
	*n_devices = dev_found;

	// disabling broadcast mode on the send socket
	bOptVal = FALSE;
	if (setsockopt(send_sd, SOL_SOCKET, SO_BROADCAST, (char *) &bOptVal, sizeof(BOOL)) ==  SOCKET_ERROR) return 0;
	
	return 1;
}
