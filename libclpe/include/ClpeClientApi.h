/********************************************************************
  This is the ClpeClientApi Class Header of CanLab CLPE Client API.  
*********************************************************************/

#ifndef ClpeClientApi_class
#define ClpeClientApi_class
 
#include "ClpeSocket.h"
#include "ClpeStreamApi.h"

#define HOST_MASTER "192.168.7.7"			// Xavier network address
#define HOST_SLAVE  "192.168.8.7"
#define PORT 30000							// Xavier network port

#define SUCCESSED		 			 0		// successed with no error

#define ERROR_GETSET_COMMAND		-1		// error - invalid command
#define ERROR_GETSET_FPS			-2		// error - invalid fps value
#define ERROR_GETSET_COMMUNICATION  -3		// error - communication fail
#define ERROR_GETSET_CHECKSUM		-4		// error - data checksum
#define ERROR_GETSET_TIMESYNC		-5		// error - time sync failed
#define ERROR_GETSET_START			-6		// error - start cam
#define ERROR_GETSET_STOP			-7		// error - stop cam
#define ERROR_INVALID_MCU_ID		-8		// error - invalid MCU ID
#define ERROR_RESYNC_TIME		    -9		// error - time sync failed
#define ERROR_INVALID_CAM_ID        -10     // error - invalid Camera ID

#define ERROR_CONNECT_DRIVER		-1		// error - can not probe driver
#define ERROR_CONNECT_NETWORK		-2		// error - can not find network
#define ERROR_CONNECT_ADDRESS		-3		// error - can not set address
#define ERROR_CONNECT_PING			-4		// error - can not ping
#define ERROR_CONNECT_CREATE		-5		// error - can not create socket
#define ERROR_CONNECT_CONNECT		-6		// error - can not connect socket

#define ERROR_CHECK_CHRONY			-1		// error - check chrony sync

#define ERROR_CHECK_CONNECT			-1		// error - check connection

#define ERROR_START_STREAM			-1		// error - start stream
#define ERROR_STOP_STREAM			-1		// error - stop stream

#define PLAY_OFF					 0
#define PLAY_ON						 1

class ClpeClientApi : private ClpeSocket
{
public:
	ClpeClientApi();
	virtual ~ClpeClientApi();
    
	//---------------- clpe api func to use -------------------//
	int Clpe_Connection(string password);

	int Clpe_CheckPci();
	int Clpe_CheckNetwork();
	int Clpe_CheckPing();
	int Clpe_CheckTimeSyncStatus();
    int Clpe_ReqResyncTime();

	int Clpe_GetMicomVersion(unsigned char* version_master);
	int Clpe_GetXavierVersion(unsigned char* version_master);
    int Clpe_GetSDKVersion(unsigned char* version);
	int Clpe_GetCamStatus(int *status);
    int Clpe_GetEepromData(int camId, unsigned char* eepromData);

	int Clpe_SetXavierPowerOff();

	int Clpe_StartStream(T_CB_APP cb_app, int use_cam_0, int use_cam_1, int use_cam_2, int use_cam_3, int display_on);
	int Clpe_StopStream();
	int Clpe_GetFrameAllCam(int *p_camera_id, unsigned char **p_buffer, unsigned int *p_size, struct timeval *pt_camera_timeStamp);
	int Clpe_GetFrameOneCam(int camera_id, unsigned char **p_buffer, unsigned int *p_size, struct timeval *pt_camera_timeStamp);
	//---------------- clpe api func to use -------------------//
private:	
   	bool Clpe_Send(unsigned char *s, int mcu_id);
	bool Clpe_Recv(unsigned char *s, int mcu_id);
	
	int Clpe_TimeSync();
	int Clpe_StartCam(char use_cam_0, char use_cam_1, char use_cam_2, char use_cam_3, int mcu_id);
	int Clpe_StopCam();
    int Clpe_CheckConnect(string password, int settingValue);
    int m_isAttachedSlave = 0;;
};

#endif
