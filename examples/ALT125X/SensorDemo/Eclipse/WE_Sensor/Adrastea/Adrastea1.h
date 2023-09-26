/*
 ***************************************************************************************************
 * This file is part of WIRELESS CONNECTIVITY SDK for STM32:
 *
 *
 * THE SOFTWARE INCLUDING THE SOURCE CODE IS PROVIDED “AS IS”. YOU ACKNOWLEDGE THAT WÜRTH ELEKTRONIK
 * EISOS MAKES NO REPRESENTATIONS AND WARRANTIES OF ANY KIND RELATED TO, BUT NOT LIMITED
 * TO THE NON-INFRINGEMENT OF THIRD PARTIES’ INTELLECTUAL PROPERTY RIGHTS OR THE
 * MERCHANTABILITY OR FITNESS FOR YOUR INTENDED PURPOSE OR USAGE. WÜRTH ELEKTRONIK EISOS DOES NOT
 * WARRANT OR REPRESENT THAT ANY LICENSE, EITHER EXPRESS OR IMPLIED, IS GRANTED UNDER ANY PATENT
 * RIGHT, COPYRIGHT, MASK WORK RIGHT, OR OTHER INTELLECTUAL PROPERTY RIGHT RELATING TO ANY
 * COMBINATION, MACHINE, OR PROCESS IN WHICH THE PRODUCT IS USED. INFORMATION PUBLISHED BY
 * WÜRTH ELEKTRONIK EISOS REGARDING THIRD-PARTY PRODUCTS OR SERVICES DOES NOT CONSTITUTE A LICENSE
 * FROM WÜRTH ELEKTRONIK EISOS TO USE SUCH PRODUCTS OR SERVICES OR A WARRANTY OR ENDORSEMENT
 * THEREOF
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS DRIVER PACKAGE.
 *
 * COPYRIGHT (c) 2023 Würth Elektronik eiSos GmbH & Co. KG
 *
 ***************************************************************************************************
 */

/**
 * @file
 * @brief Adrastea driver header file.
 */

#ifndef ADRASTEA_H_INCLUDED
#define ADRASTEA_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

//#include "../global/global.h"
#include "../global/global_adrastea.h"
#include "../../../../../../devices/ALT125X/include/serial_container.h"
 #include "../../../source/Sensor_Integrate.h"

typedef uint32_t serial_handle;



/**
 * @brief Max recommended payload size is 1460 bytes.
 */
#define ADRASTEA_MAX_PAYLOAD_SIZE 1460

/**
 * @brief Default receive buffer size (used when receiving data e.g. via sockets or HTTP requests).
 */
#define ADRASTEA_RECEIVE_BUFFER_SIZE 2048

/**
 * @brief Max. length of sent commands and responses from Adrastea.
 */
#define ADRASTEA_LINE_MAX_SIZE 2048
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */

/**
 * @brief Max. length of response text (size of buffer storing responses received from ADRASTEA).
 * @see Adrastea_currentResponseText
 */
#define ADRASTEA_MAX_RESPONSE_TEXT_LENGTH ADRASTEA_LINE_MAX_SIZE

#define ADRASTEA_RESPONSE_OK     "OK"                       /**< String sent by module if AT command was successful */
#define ADRASTEA_RESPONSE_ERROR  "ERROR"                    /**< String sent by module if AT command failed */
#define ADRASTEA_SMS_ERROR  "+CMS ERROR"                    /**< String sent by module if SMS AT command failed */

#define ADRASTEA_MCU_EVENT     "MCU menu -- PowerManager"	/**< String sent by module on boot up before access to at commands */
#define ADRASTEA_MAPCLIOPEN_EVENT  "Open MAP CLI"			/**< String sent by module after module is ready to receive at commands */
#define ADRASTEA_POWERMODECHANGE_EVENT  "Configured to"		/**< String sent by module after mcu changed power mode */
#define ADRASTEA_SLEEPSET_EVENT  "Sleep -"					/**< String sent by module after mcu changed sleep */
#define ADRASTEA_MAPCLICLOSE_EVENT     "MAP CLI Closed"		/**< String sent by module when map cli is closed*/

//Define timeout
#define ADRASTEA_TIMEOUT_GENERAL     1000
#define ADRASTEA_TIMEOUT_DEVICE      1000
#define ADRASTEA_TIMEOUT_GNSS        10000
#define ADRASTEA_TIMEOUT_HTTP        3000
#define ADRASTEA_TIMEOUT_MQTT        3000
#define ADRASTEA_TIMEOUT_NETSERVICE  10000
#define ADRASTEA_TIMEOUT_PACKETDOMAIN 3000
#define ADRASTEA_TIMEOUT_PROPRIETARY 10000
#define ADRASTEA_TIMEOUT_SIM         3000
#define ADRASTEA_TIMEOUT_SMS         20000
#define ADRASTEA_TIMEOUT_SOCKET      3000
#define ADRASTEA_TIMEOUT_POWER       3000

/**
 * @brief AT command confirmation status.
 */
typedef enum Adrastea_CNFStatus_t
{
	Adrastea_CNFStatus_Success,
	Adrastea_CNFStatus_Failed,
	Adrastea_CNFStatus_Invalid,
	Adrastea_CNFStatus_NumberOfValues
} Adrastea_CNFStatus_t;

/**
 * @brief Timeout categories (for responses to AT commands).
 * @see Adrastea_SetTimeout(), Adrastea_GetTimeout()
 */
/*typedef enum Adrastea_Timeout_t
{
	Adrastea_Timeout_General=1000,
	Adrastea_Timeout_Device=1000,
	Adrastea_Timeout_GNSS=10000,
	Adrastea_Timeout_HTTP= 3000,
	Adrastea_Timeout_MQTT = 3000,
	Adrastea_Timeout_NetService = 10000,
	Adrastea_Timeout_PacketDomain = 3000,
	Adrastea_Timeout_Proprietary = 10000,
	Adrastea_Timeout_SIM = 3000,
	Adrastea_Timeout_SMS  = 20000,
	Adrastea_Timeout_Socket = 3000 ,
	Adrastea_Timeout_Power = 3000,
	Adrastea_Timeout_NumberOfValues
} Adrastea_Timeout_t;*/

typedef enum Adrastea_Timeout_t
{
	Adrastea_Timeout_General,
	Adrastea_Timeout_Device,
	Adrastea_Timeout_GNSS,
	Adrastea_Timeout_HTTP,
	Adrastea_Timeout_MQTT,
	Adrastea_Timeout_NetService,
	Adrastea_Timeout_PacketDomain,
	Adrastea_Timeout_Proprietary,
	Adrastea_Timeout_SIM,
	Adrastea_Timeout_SMS,
	Adrastea_Timeout_Socket,
	Adrastea_Timeout_Power,
	Adrastea_Timeout_NumberOfValues
} Adrastea_Timeout_t;

typedef enum {
    adrastea_unknown = 0,
    adrastea_started = 1,
    adrastea_UART_init = 2,
    adrastea_TIDS_init = 4,
    adrastea_ITDS_init = 8,
    adrastea_PADS_init = 16,
    adrastea_HIDS_init = 32,
    adrastea_provisioning = 64,
    adrastea_provisioned = 128,
    adrastea_network_disconnected = 256,
    adrastea_network_connected = 512,
    adrastea_MQTT_connected = 1024,
    adrastea_MQTT_disconnected = 2048,
    adrastea_MQTT_wrong_root_ca = 4096,
    adrastea_error = 8192
} Adrastea_status_t;

/**
 * @brief Pins used by this driver.
 */
typedef enum Adrastea_Pin_t
{
	Adrastea_Pin_Reset,
	Adrastea_Pin_WakeUp,
	Adrastea_Pin_Count
} Adrastea_Pin_t;

/**
 * @brief Hold how many lines if the incoming response.
 */
typedef struct Adrastea_Response_Complete_t
{
	uint8_t lineskip;
	char delim;
} Adrastea_Response_Complete_t;

/**
 * @brief Adrastea event callback.
 *
 * Arguments: Event text
 */
typedef void (*Adrastea_EventCallback_t)(char*);

/**
 * @brief Adrastea byte received callback.
 *
 * Is used to handle responses (byte per byte) from Adrastea.
 *
 * Arguments: Received byte
 *
 * @see Adrastea_SetByteRxCallback()
 */
typedef void (*Adrastea_ByteRxCallback_t)(uint8_t);

/**
 * @brief Adrastea line received callback.
 *
 * Can be used to intercept responses from Adrastea.
 *
 * Arguments: Line text, length of text
 * Returns: true if the line should not be processed by the driver,
 *          false if the driver should process the line as usual
 *
 * @see Adrastea_SetLineRxCallback()
 */
typedef bool (*Adrastea_LineRxCallback_t)(char*, uint16_t);

extern uint8_t Adrastea_optionalParamsDelimCount;

//extern bool Adrastea_Init(uint32_t baudrate, WE_FlowControl_t flowControl, WE_Parity_t parity, Adrastea_EventCallback_t eventCallback, WE_Pin_t *pins);
extern bool Adrastea_Deinit(void);

extern bool Adrastea_PinReset(void);
extern bool Adrastea_PinWakeUp(void);
//extern bool Adrastea_SetPin(Adrastea_Pin_t pin, WE_Pin_Level_t level);
//extern WE_Pin_Level_t Adrastea_GetPinLevel(Adrastea_Pin_t pin);

extern bool Adrastea_SendRequest(char *data);
extern bool Adrastea_WaitForConfirm(uint32_t maxTimeMs, Adrastea_CNFStatus_t expectedStatus, char *pOutResponse);

 int Adrastea_CountArgs(char *stringP);
extern void Adrastea_HandleRxByte(uint8_t receivedByte);
/**
 * @brief Is called when a complete line has been received.
 *
 * @param[in] rxPacket Received text
 * @param[in] rxLength Received text length
 */
 extern void Adrastea_HandleRxLine(char *rxPacket, uint16_t rxLength);

extern bool Adrastea_SetTimingParameters(uint32_t waitTimeStepMicroseconds, uint32_t minCommandIntervalMicroseconds);
extern void Adrastea_SetTimeout(Adrastea_Timeout_t type, uint32_t timeout);
extern uint32_t Adrastea_GetTimeout(Adrastea_Timeout_t type);
//extern void Adrastea_GetDefaultPinConfig(WE_Pin_t *pins);

extern void Adrastea_Transmit(serial_handle *handle, const char *buf, uint32_t len);
extern void Adrastea_SetByteRxCallback(Adrastea_ByteRxCallback_t callback);
extern void Adrastea_SetLineRxCallback(Adrastea_LineRxCallback_t callback);
extern void Adrastea_SetEolCharacters(uint8_t eol1, uint8_t eol2, bool twoEolCharacters);
extern bool Adrastea_CheckATMode();

extern int Init_Uart();

extern void rxUartTask(void *pvParameters);
extern int connect_IoTCentral();
extern int connect_mqttdashboard();
extern int mqttdashboard_publish();
extern int IoTCentral_publish(float temperature_PADS, float pressure_PADS, float temperature_TIDS, float ITDSsensorData_temperature, float ITDSsensorData_Xaxis_float, float ITDSsensorData_Yaxis_float, float ITDSsensorData_Zaxis_float);
extern int azureconfig_input();
//extern void extractOperationId(char *inputString);
extern char* extractOperationId(char *inputString, char *operationId, char *assignedHub);
#endif // ADRASTEA_H_INCLUDED
