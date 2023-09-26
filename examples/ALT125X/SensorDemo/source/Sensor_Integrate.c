#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "newlibPort.h"
#include "platform.h"

#include "Sensor_Integrate.h"
#include "ATCommands/ATPacketDomain.h"
#include "ATCommands/ATMQTT.h"
#include "ATCommands/ATProprietary.h"

// Define UART variables for communicating

serial_handle *userUartHandle = NULL;
serial_handle *modemUartHandle = NULL;

TaskHandle_t gSensorDemoHandle = NULL;
TaskHandle_t gDebugUartHandle = NULL;

static char cmdBuffer[2048];
static int startSensorTask(void *pvParameters);

/*
char kitID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char scopeID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char modelID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char deviceID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char iotHubAddress[MAX_URL_LEN] = {0};
char dpsServerAddress[MAX_URL_LEN] = {0};
*/

// Define where to send the sensor data, with either IOTCENTRAL or MQTTDASHBOARD
#define MQTTDASHBOARD

// Create a task to initialize and setup communication protocols
int Init_Setup()
{
  xTaskCreate(startSensorTask, /* The function that implements the task. */
		  "SensorDemo", /* The text name assigned to the task - for debug only as it is not used by
						the kernel. */
		  configMINIMAL_STACK_SIZE * 2, /* The size of the stack to allocate to the task. */
		  (void *)NULL,          /* The parameter passed to the task */
		  configMAX_PRIORITIES - 1, /* The priority assigned to the task - minimal priority. */
		  &gSensorDemoHandle);

	return WE_SUCCESS;
}

static int startSensorTask(void *pvParameters)
{

	bool ret = false;
	unsigned int status = 0;

	// Setup UART and initialize the sensors and read their ID
	if(WE_SUCCESS == Init_Uart())
		{
		printf("init UART process is done\r\n");
		status |= adrastea_UART_init;
		}


	WE_Delay(300);

	// Check network connectivity and internet connectivity
	if(WE_SUCCESS == ATPacketDomain_SetNetworkRegistrationResultCode(ATPacketDomain_Network_Registration_Result_Code_Enable_with_Location_Info))
		{
			printf("Connected to the base station\r\n");
		}
	WE_Delay(1000);

	ATProprietary_Ping(ATProprietary_IP_Addr_Format_IPv4, "8.8.8.8", ATProprietary_Ping_Packet_Count_Invalid, ATProprietary_Ping_Packet_Size_Invalid, ATProprietary_Ping_Timeout_Invalid);
	WE_Delay(2000);
	//ATProprietary_ResolveDomainName(1, "www.google.com", ATProprietary_IP_Addr_Format_IPv4);


	if(WE_FAIL == connect_mqttdashboard())
	{
		printf("Could not connect to MQTT broker\r\n");
		return WE_FAIL;
	}


	for(;;)
	{


		if(WE_FAIL == mqttdashboard_publish())
		{
			printf("Could not publish to MQTT broker\r\n");
			return WE_FAIL;
		}

	}
	return WE_SUCCESS;

}
