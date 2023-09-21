#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "newlibPort.h"
#include "platform.h"
#include "WSEN_TIDS.h"
#include "WSEN_ITDS.h"
#include "WSEN_PADS.h"
#include "WSEN_HIDS.h"
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
static int read_PADS_SensorData(float *temperature, float *pressure);
static int initialize_pads();
static int read_TIDS_SensorData(float *temperature);
struct ITDS_SensorData read_ITDS_SensorData();

/*
char kitID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char scopeID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char modelID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char deviceID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char iotHubAddress[MAX_URL_LEN] = {0};
char dpsServerAddress[MAX_URL_LEN] = {0};
*/

// Define where to send the sensor data, with either IOTCENTRAL or MQTTDASHBOARD
#define IOTCENTRAL

struct ITDS_SensorData {
    float temperature;
    int16_t Xaxis;
    int16_t Yaxis;
    int16_t Zaxis;
	float Xaxis_float;
    float Yaxis_float;
    float Zaxis_float;
};

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
	float temperature_PADS, pressure_PADS, temperature_TIDS,temperature_ITDS, Yaxis_ITDS, Zaxis_ITDS, temperature_HIDS, humidity_HIDS;
	//Adrastea_status_t status;
	bool ret = false;
	unsigned int status = 0;
	// bool uartInitialized, tidsInitialized, itdsInitialized, padsInitialized, hidsInitialized = false;

#ifdef DEBUG_OUTPUT
	printf("Start init process\r\n");
#endif

	// Setup UART and initialize the sensors and read their ID
	if(WE_SUCCESS == Init_Uart())
		{
		printf("init UART process is done\r\n");
		status |= adrastea_UART_init;
		}

	if(!(WE_FAIL == Init_TIDS()))
	{
		printf("init TIDS process is done\r\n");
		status |= adrastea_TIDS_init;
	}

	if(!(WE_FAIL == Init_ITDS()))
		status |= adrastea_ITDS_init;

	if(!(WE_FAIL == Init_PADS()))
		status |= adrastea_PADS_init;

	if(!(WE_FAIL == Init_HIDS()))
		status |= adrastea_HIDS_init;

	if (status == (adrastea_UART_init | adrastea_TIDS_init | adrastea_ITDS_init | adrastea_PADS_init | adrastea_HIDS_init)) {
	    // Proceed with the next steps of the state machine
	    printf("All init processes are done\r\n");
	} else {
	    // Handle error case or other state transitions
	    printf("Init processes are incomplete\n");
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

#ifdef IOTCENTRAL
	if(WE_FAIL == connect_IoTCentral())
	{
		printf("Could not connect to DPS of IoT Centrl\r\n");
		return WE_FAIL;
	}

#elif defined(MQTTDASHBOARD)
	if(WE_FAIL == connect_mqttdashboard())
	{
		printf("Could not connect to MQTT broker\r\n");
		return WE_FAIL;
	}
#endif
	//printf("Connected to DPS o IoT Centrl\r\n");

	for(;;)
	{
		read_PADS_SensorData(&temperature_PADS, &pressure_PADS);
		read_TIDS_SensorData(&temperature_TIDS);
		struct ITDS_SensorData ITDSsensorData = read_ITDS_SensorData();

		  // Round temp to 2 decimal places
		//temperature_PADS = roundf(temperature_PADS * 100) / 100;
		//pressure_PADS = roundf(pressure_PADS * 100) / 100;
		//ITDSsensorData.temperature = roundf(ITDSsensorData.temperature * 100) / 100;

		// Round ITDS value to 4 decimal places
		//ITDSsensorData.Xaxis_float = roundf(ITDSsensorData.Xaxis_float * 10000) / 10000;
		//ITDSsensorData.Yaxis_float = roundf(ITDSsensorData.Yaxis_float * 10000) / 10000;
		//ITDSsensorData.Zaxis_float = roundf(ITDSsensorData.Zaxis_float * 10000) / 10000;

#ifdef DEBUG_OUTPUT
		printf("\nAll Sensor Data\r\n");
		printf("Sensor_PADS data = Temp : %f C    Press %f kPa\r\n", temperature_PADS, pressure_PADS);
		printf("Sensor_TIDS data = Temp : %f C \r\n", temperature_TIDS);
		printf("Sensor_ITDS data = Temp : %f C \r\n", ITDSsensorData.temperature);

		//printf("X-axis: %d\n", ITDSsensorData.Xaxis);
		printf("X-axis : %f\r\n", (ITDSsensorData.Xaxis_float));
		//printf("Y-axis: %d\n", ITDSsensorData.Yaxis);
		printf("Y-axis : %f\r\n", ITDSsensorData.Yaxis_float);
		//printf("Z-axis: %d\n", ITDSsensorData.Zaxis);
		printf("Z-axis : %f\r\n", ITDSsensorData.Zaxis_float);
#endif

#ifdef IOTCENTRAL
		if(WE_FAIL == IoTCentral_publish(temperature_PADS, pressure_PADS, temperature_TIDS, ITDSsensorData.temperature, ITDSsensorData.Xaxis_float, ITDSsensorData.Yaxis_float, ITDSsensorData.Zaxis_float))
		{
			printf("Could not publish to IoT Central\r\n");
			return WE_FAIL;
		}

#elif defined(MQTTDASHBOARD)
		if(WE_FAIL == mqttdashboard_publish(temperature_PADS, pressure_PADS, temperature_TIDS, ITDSsensorData.temperature, ITDSsensorData.Xaxis_float, ITDSsensorData.Yaxis_float, ITDSsensorData.Zaxis_float))
		{
			printf("Could not publish to MQTT broker\r\n");
			return WE_FAIL;
		}
#endif
	}
	return WE_SUCCESS;

}

// Functions for Sensors
/*
 ------------------------------------------------------------------------------------------------------------------------------------------
 */

//            PADS

static int pads_inizialized = 0;

int Init_PADS(){
	if(WE_SUCCESS == I2CInit(PADS_ADDRESS_I2C_1))
		{
			printf("I2C PADS initialized\n\r");
		}

		else
		{
			printf("Could not initialized I2C\n\r");
			return -1;
		}

		I2C_SetAddress(PADS_ADDRESS_I2C_1);
		uint8_t id = 0;

		if (WE_SUCCESS == PADS_getDeviceID(&id))
		{
			if(id == PADS_DEVICE_ID_VALUE)
				printf("PADS ID verified: %02x\n\r\n", id);
			else
			{
				printf("Could not read out device id  of PADS\n\r\n");
				return WE_FAIL;
			}
		}
}

int read_PADS_SensorData(float *temperature, float *pressure_PADS)
{
	if(WE_SUCCESS != initialize_pads())
	   	 {
	   	  printf("Could not initalize pads sensor\n\r");
	   	  return -1;
	   	 }
	   	 I2C_SetAddress(PADS_ADDRESS_I2C_1);
		 PADS_setSingleConvMode(PADS_enable);
	   	 vTaskDelay(15);
	if (WE_FAIL == PADS_setSingleConvMode(PADS_enable)) return WE_FAIL;
	vTaskDelay(15);

   	if (WE_FAIL == PADS_getTemperature(temperature)) return WE_FAIL;
   	WE_Delay(15);

   	if (WE_FAIL == PADS_getPressure(pressure_PADS)) return WE_FAIL;

   	return WE_SUCCESS;
}

int initialize_pads()
{
	if (1 == pads_inizialized)
	{
		return WE_SUCCESS;
	}

	int retVal = WE_SUCCESS;

	/*set ODR to Power down*/
	retVal &= PADS_setOutputDataRate(pads_outputDataRatePowerDown);

	/*set low pass filter to BW=ODR/20*/
	if(retVal) retVal &= PADS_setLowPassFilterConf(pads_lPfilterBW2);

	/*enable low pass filter */
	if (retVal) retVal &= PADS_setLowPassFilter(PADS_enable);

	/*enable Block Data Update*/
	if (retVal) retVal &= PADS_setBlockDataUpdate(PADS_enable);

	/*select Power Mode [0:low Current Mode; 1:low noise Mode]*/
	if (retVal) retVal &= PADS_setPowerMode(pads_lowNoise);

	/*enable the Auto Increment */
	if (retVal) retVal &= PADS_setAutoIncrement(PADS_enable);
	pads_inizialized = retVal;
	return retVal;
}


//            TIDS

int Init_TIDS(){
	//#define TIDS_DEVICE_ID_VALUE 0xA0
	if(WE_SUCCESS == I2CInit(TIDS_ADDRESS_I2C_1))
	{
		printf("I2C TIDS initialized\n\r");
	}
	else
	{
		printf("Could not initialized I2C\n\r");
		return WE_FAIL;
	}

	I2C_SetAddress(TIDS_ADDRESS_I2C_1);
	uint8_t id = 0;

	if (WE_SUCCESS == TIDS_getDeviceID(&id))
	{
		if(id == TIDS_DEVICE_ID_VALUE)
			printf("TIDS ID verified: %02x\n\r\n", id);
		else
		{
			printf("Could not read out device id \n\r\n");
			return WE_FAIL;
		}
	}

}

static int read_TIDS_SensorData(float *temperature_TIDS)
{
	I2C_SetAddress(TIDS_ADDRESS_I2C_1);
	if(WE_FAIL == TIDS_setSingleConvMode(TIDS_enable))
	{
		printf("Could not set single conv\n\r");
		return WE_FAIL;
	}

	if (WE_FAIL == TIDS_getTemperature(temperature_TIDS))
		return WE_FAIL;
}

//            ITDS

int Init_ITDS(){
	if(WE_SUCCESS == I2CInit(ITDS_ADDRESS_I2C_1))
	{
		printf("I2C ITDS initialized\n\r");
	}

	else
	{
		printf("Could not initialized I2C\n\r");
		return -1;
	}
	I2C_SetAddress(ITDS_ADDRESS_I2C_1);
	uint8_t id = 0;

	if (WE_SUCCESS == ITDS_getDeviceID(&id))
	{
		if(id == ITDS_DEVICE_ID_VALUE)
			printf("ITDS ID verified: %02x\n\r\n", id);
		else
		{
			printf("Could not read out device id \n\r\n");
			return WE_FAIL;
		}
	}

	// Sampling rate of 200 Hz
	ITDS_setOutputDataRate(odr6);
	// Enable normal mode
	ITDS_setOperatingMode(normalOrLowPower);
	ITDS_setpowerMode(normalMode);
	// Enable block data update
	ITDS_setBlockDataUpdate(ITDS_enable);
	// Enable address auto increment
	ITDS_setAutoIncrement(ITDS_enable);
	// Full scale +-16g
	ITDS_setFullScale(sixteenG);
	// Filter bandwidth = ODR/2
	ITDS_setFilteringCutoff(outputDataRate_2);
}

struct ITDS_SensorData read_ITDS_SensorData()
{
	struct ITDS_SensorData sensorData;
	I2C_SetAddress(ITDS_ADDRESS_I2C_1);
	ITDS_getTemperature12bit(&(sensorData.temperature));

	I2C_SetAddress(ITDS_ADDRESS_I2C_1);
	ITDS_state_t DRDY = ITDS_disable;
	int16_t XRawAcc = 0, YRawAcc = 0, ZRawAcc = 0;



	uint8_t retries = 0;
	do
	{
		ITDS_getdataReadyState(&DRDY);
		retries++;
	} while ((DRDY == ITDS_disable) && (retries < 10));

	if(retries < 10)
	{
		WE_Delay(400);
		ITDS_getRawAccelerationX(&(sensorData.Xaxis));
		XRawAcc = sensorData.Xaxis;
		//printf("In line 376 XRawAcc is %d\n\r\n", XRawAcc);
		XRawAcc = XRawAcc >> 2; // shifted by 2 as 14bit resolution is used in normal mode
		//printf("In line 378 XRawAcc after shift is %d\n\r\n", XRawAcc);
		float XAcceleration = (float) (XRawAcc);
		XAcceleration = XAcceleration / 1000; // mg to g
		XAcceleration = XAcceleration * 1.952; // Multiply with sensitivity 1.952 in normal mode, 14bit, and full scale +-16g
		sensorData.Xaxis_float = XAcceleration;

		ITDS_getRawAccelerationY(&(sensorData.Yaxis));
		YRawAcc = sensorData.Yaxis;
		//printf("In line 394 YRawAcc is %d\n\r\n", YRawAcc);
		YRawAcc = YRawAcc >> 2; // shifted by 2 as 14bit resolution is used in normal mode
		//printf("In line 396 YRawAcc shift is %d\n\r\n", YRawAcc);
		float YAcceleration = (float) (YRawAcc);
		//printf("In line 399 YAcc is %f\n\r\n", YAcceleration);
		YAcceleration = YAcceleration / 1000; // mg to g
		YAcceleration = YAcceleration * 1.952; // Multiply with sensitivity 1.952 in normal mode, 14bit, and full scale +-16g
		sensorData.Yaxis_float = YAcceleration;

		ITDS_getRawAccelerationZ(&(sensorData.Zaxis));
		ZRawAcc = sensorData.Zaxis;
		ZRawAcc = ZRawAcc >> 2; // shifted by 2 as 14bit resolution is used in normal mode
		float ZAcceleration = (float) (ZRawAcc);
		ZAcceleration = ZAcceleration / 1000; // mg to g
		ZAcceleration = ZAcceleration * 1.952; // Multiply with sensitivity 1.952 in normal mode, 14bit, and full scale +-16g
		sensorData.Zaxis_float = ZAcceleration;

	}
	return sensorData;
}

//                    HIDS

int Init_HIDS(){
	//The Output Data Rate in Hz
	int ODR = 0;

	if(WE_SUCCESS == I2CInit(HIDS_ADDRESS_I2C_0))
	{
		printf("I2C HIDS initialized\n\r");

		if (WE_FAIL == HIDS_setOdr(ODR))
		{
			printf("Could not initialized output data rate\n\r\n");
			return WE_FAIL;
		}

		if (WE_FAIL == HIDS_setPowerMode(activeMode))
		{
			printf("Could not initialized power mode\n\r");
			return WE_FAIL;

		}

	}
	else
	{
		printf("Could not initialized I2C\n\r\n");
		return -1;
	}

	uint8_t id = 0;
	I2C_SetAddress(HIDS_ADDRESS_I2C_0);
	if(WE_SUCCESS == HIDS_getDeviceID(&id))
	{
		printf("HIDS ID verified %02x\n\r\n", id);
	}
	else
	{
		printf("Could not read out device id of HIDS \n\r");
		return -1;
	}

}
