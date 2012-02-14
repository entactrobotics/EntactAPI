/*******************************************************************************   
	EntactAPI.h

	This header defines all the library functions available for interfacing with
	Entact haptic devices using ethernet.

	Currently this API is limited to Windows XP, Vista and 7

*******************************************************************************/

#ifndef _EntactAPI
#define _EntactAPI

#if defined(_MSC_VER)	// Microsoft compiler
	#ifdef ENTACTAPI_EXPORTS
		#define ENTACTAPI_API __declspec(dllexport)
	#else
		#define ENTACTAPI_API __declspec(dllimport)
	#endif
#elif defined(__GNUC__) // GNU compiler
	#define ENTACTAPI_API
#else
	#error define your compiler
#endif

/*************************************************************************** 
	Defines 
***************************************************************************/
#define EAPI_DISABLED_MODE			(0x0000)
#define EAPI_FORCECONTROL_MODE		(0x0001)
#define EAPI_TORQUECONTROL_MODE		(0x0002)

#define EAPI_ERR					(-1)
#define EAPI_OK						(1)

typedef void* eapi_device_handle;

extern "C" 
{
	/*************************************************************************** 
		API Initialization Functions
	***************************************************************************/
	
	/*
	openEAPI()
		eapi_device_handle handles[],  
			an array of device handles, NULL if no devices are attached
		int size,
			size of the handles[] array which is passed in
	returns:
		n_devices,	
			if the function call is successfull n_devices contains the number of attached
			devices found. n_devices will be 0 of no devices are found but the API is 
			intialized succesfully
		EAPI_ERR,	
			if an error occurs initializing the API 
	*/
	ENTACTAPI_API int openEAPI(eapi_device_handle handles[], int size);
	/*
	closeEAPI()

	returns:
		EAPI_OK,
			upon successfull API close
		EAPI_ERR,
			if an error occurs during de-initialization, or the API
			was never initialized in the first place
	*/
	ENTACTAPI_API int closeEAPI();

	/*************************************************************************** 
		Device Management Functions
	***************************************************************************/
	
	ENTACTAPI_API int setSampleRateEAPI(eapi_device_handle handle, int frequency);
	ENTACTAPI_API int setModeEAPI(eapi_device_handle handle, int mode);
	ENTACTAPI_API int getModeEAPI(eapi_device_handle handle);
	ENTACTAPI_API int setEventCallbackEAPI(eapi_device_handle handle, void (*cbFunc) (unsigned char eventArg));

	/*************************************************************************** 
		Calibration and Joint Angles
	***************************************************************************/
	ENTACTAPI_API int homeDeviceEAPI(eapi_device_handle handle);
	ENTACTAPI_API int isHomedEAPI(eapi_device_handle handle);
	ENTACTAPI_API int readJointsEAPI(eapi_device_handle handle, double *pos, double *vel, int size);

	/*************************************************************************** 
		Task Position and Velocity and Jacobian
	***************************************************************************/

	ENTACTAPI_API int readTaskPositionEAPI(eapi_device_handle handle, double *taskpos, int size);
	ENTACTAPI_API int readTaskVelocityEAPI(eapi_device_handle handle, double *taskvel, int size);
	
	/*************************************************************************** 
		Force and Joint Torque
	***************************************************************************/
	ENTACTAPI_API int writeForceEAPI(eapi_device_handle handle, double *f, int size);

};

#endif // _EntactAPI