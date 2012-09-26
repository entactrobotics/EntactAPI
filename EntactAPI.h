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
	/*
	setSampleRateEAPI()
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device
		int frequency,
			frequency in Hz to set the real-time loop running on the device (by default 1000)
			it is recommended to keep the frequency below 5000 to avoid real-time over-runs in the device firmware
	returns:
		EAPI_OK,	
			if the function call is successfull 
		EAPI_ERR,	
			if the frequency update is unsuccessfull 
	*/
	ENTACTAPI_API int setSampleRateEAPI(eapi_device_handle handle, int frequency);
	/*
	setModeEAPI()
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device
		int mode,
			use the masks above to set the various modes of operation
			EAPI_FORCECONTROL_MODE, in this mode forces are commanded to the device, and world space positions are returned to the API
			EAPI_TORQUECONTROL_MODE, in this mode motor torques are commanded to the device, and motor shaft angles are returned to the API
			EAPI_DISABLED_MODE, devices start up mode, in this mode the motor drivers are disabled such that the device cannot produce forces/torques
	returns:
		EAPI_OK,	
			if the function call is successfull at changing the devices mode 
		EAPI_ERR,	
			if the device mode change is unsuccessfull 
	*/
	ENTACTAPI_API int setModeEAPI(eapi_device_handle handle, int mode);
	/*
	getModeEAPI()
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device
	returns:
		EAPI_FORCECONTROL_MODE / EAPI_TORQUECONTROL_MODE / EAPI_DISABLED_MODE,	
			depending on which mode the device is currently set in
		EAPI_ERR,	
			if an error occurs polling the device for its mode
	*/
	ENTACTAPI_API int getModeEAPI(eapi_device_handle handle);
	// Currently un-implemented
	// Will be used for communicating safety fault conditions back to the API in real-time
	ENTACTAPI_API int setEventCallbackEAPI(eapi_device_handle handle, void (*cbFunc) (unsigned char eventArg));

	/*************************************************************************** 
		Calibration and Joint Angles
	***************************************************************************/
	/*
	homeDeviceEAPI()
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device

			This function is used to calibrate the haptic device such that the world positon can
			be relayed back to the API. The haptic device must be moved to its calibration position
			when this function is called. Entry into the force control mode (EAPI_FORCECONTROL_MODE
			is restricted if the device is not first homed.

	returns:
		EAPI_OK,	
			if the function call is successfull at calibrating the device 
		EAPI_ERR,	
			if the device fails to home (probably a communication error in UDP)
	*/
	ENTACTAPI_API int homeDeviceEAPI(eapi_device_handle handle);
	/*
	isHomedEAPI()
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device
	returns:
		,	
			if the function call is successfull n_devices contains the number of attached
			devices found. n_devices will be 0 of no devices are found but the API is 
			intialized succesfully
		EAPI_ERR,	
			if an error occurs 
	*/
	ENTACTAPI_API int isHomedEAPI(eapi_device_handle handle);
	/*
	NOTE: this function still needs some small additions to work properly

	readJointsEAPI()  
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device
		double *pos,
			an array to store the position values of each encoder of the haptic device.
			for the w5d there are 8 encoders which can be checked for position
		double *vel,
			an array to store the velocity values of each encoder of the haptic device.
		int size,
			the size of the pos and vel arrays. Note: these should both be the same length.
			for the w5d the pos and vel arrays should be 8 elements long

			Call this function to poll the device for its encoder position and velocity values.
			These are the position of each of the joints, and do NOT correspond to the world position/velocity.

	returns:
		EAPI_OK,	
			if the function call is successfull 
		EAPI_ERR,	
			if the device fails to return the latest encoder values (probably a communication error in UDP)
	*/
	ENTACTAPI_API int readJointsEAPI(eapi_device_handle handle, double *pos, double *vel, int size);

	/*************************************************************************** 
		Task Position and Velocity and Jacobian
	***************************************************************************/
	/*
	readTaskPositionEAPI()
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device
		double *taskpos,
			each element of this array will be filled with position/orientation information (in world space)
			for the w5d:
			taskpos[0] = X position (mm)
			taskpos[1] = Y position (mm)
			taskpos[2] = Z position (mm)
			taskpos[3] = Rotation Matrix Element 1,1
			taskpos[4] = Rotation Matrix Element 1,2
			taskpos[5] = Rotation Matrix Element 1,3
			taskpos[6] = Rotation Matrix Element 2,1
			taskpos[7] = Rotation Matrix Element 2,2
			taskpos[8] = Rotation Matrix Element 2,3
			taskpos[9] = Rotation Matrix Element 3,1
			taskpos[10] = Rotation Matrix Element 3,2
			taskpos[11] = Rotation Matrix Element 3,3
		int size,
			size of the taskpos array which is passed in to the function. for the w5d this would be 12

		This function is called to obtain the current position/orientation of the haptic devices handle.
		The device must be calibrated, and in force control mode.
		The values for position/orientation are updated every time a force is commanded to the device.

	returns:
		EAPI_OK,	
			if the function call is successfull 
		EAPI_ERR,	
			if the device fails to retrieve the current positions
	*/
	ENTACTAPI_API int readTaskPositionEAPI(eapi_device_handle handle, double *taskpos, int size);
	/*
	readTaskVelocityEAPI()
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device
		double *taskvel,
			each element of this array will be filled with velocity information (in world space)
			for the w5d:
			taskvel[0] = X velocity (mm/s)
			taskvel[1] = Y velocity (mm/s)
			taskvel[2] = Z velocity (mm/s)
			taskvel[3] = X angular velocity (rad/s)
			taskvel[4] = Y angular velocity (rad/s)
			taskvel[5] = Z angular velocity (rad/s)
		int size,
			size of the taskvel array which is passed in to the function.
			for the w5d this would be 6.

		The usage of this function is analogous to the Position function above.

	returns:
		EAPI_OK,	
			if the function call is successfull 
		EAPI_ERR,	
			if the device fails to retrieve the current velocities
	*/
	ENTACTAPI_API int readTaskVelocityEAPI(eapi_device_handle handle, double *taskvel, int size);
	
	/*************************************************************************** 
		Force and Joint Torque
	***************************************************************************/
	/*
	writeForceEAPI()
		eapi_device_handle handle,  
			an element of the device handles array, example: handle[0] for the first attached device
		double *f,
			force command for the device.
			for a w5d:
			f[0] = X force (N)
			f[1] = Y force (N)
			f[2] = Z force (N)
			f[3] = X torque (N.mm)
			f[4] = Y torque (N.mm)
			f[5] = Z torque (N.mm)
		int size,
			size of the f array which is passed in to the function. for the w5d this would be 6 elements

			Call this function to command a force/torque vector to the handle of the device. 
			Note: this function may only be called in force control mode (after the device is calibrated)
			Also Note: In force control mode calling this function causes the API to refresh the current
			position/velocity of the device.

	returns:
		EAPI_OK,	
			if the function call is successfull 
		EAPI_ERR,	
			if the device fails to update the devices force/torque setpoint
	*/
	ENTACTAPI_API int writeForceEAPI(eapi_device_handle handle, double *f, int size);

};

#endif // _EntactAPI