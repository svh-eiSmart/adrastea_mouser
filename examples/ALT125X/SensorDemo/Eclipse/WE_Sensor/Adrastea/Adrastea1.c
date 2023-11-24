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
 * @brief Adrastea driver source file.
 */

#include "Adrastea1.h"
#include "WSEN_TIDS.h"
#include "WSEN_ITDS.h"
#include "WSEN_PADS.h"
#include "WSEN_HIDS.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

 #include "ATCommands/ATEvent.h"
#include "ATCommands/ATDevice.h"
#include "../global/ATCommands.h"
#include "ATCommands/ATMQTT.h"
#include "ATCommands/ATProprietary.h"
#include "json.h"
#include "FreeRTOS.h"
#include "task.h"

// UART config parameter

#include "../../../source/FreeRTOSConfig.h"
typedef void * TaskHandle_t;
static TaskHandle_t gSensorDemoHandle = NULL;
static TaskHandle_t gDebugUartHandle = NULL;

 static serial_handle *userUartHandle = NULL;
 static serial_handle *modemUartHandle = NULL;

#define RESPONSE_BUFFER_SIZE 1024
static char cmdBuffer[2048];
char responseBuffer[RESPONSE_BUFFER_SIZE];
uint32_t responseIndex = 0;
static bool OkResponseReceived = false;
static bool ErrorResponseReceived = false;
static bool prevResponse = false;
static bool JsonFile = false;
bool SIM_Registered = false;
bool operationID_flag = false;

// Define a structure to represent a JSON-like object
typedef struct {
    char *scopeID;
    char *devID;
    int keepalivetime;
} JsonConfig;

size_t test_str_len= 0;
#define MINICONSOLE_UART_INSTANCE ACTIVE_UARTF0
#define WE_SUCCESS   0
#define WE_FAIL     -1

// azureconfig input parameter
char c;
char *scopeID = NULL;
char *devID = NULL;
int keepalivetime = 0;
char test_str[256]; // Assuming a maximum of 255 characters
int test_str_index = 0; // Index to keep track of the current position in the array
char read_file[1024];
// char *operationId;
#define MAX_OPERATION_ID_LENGTH 256
//char operationId[MAX_OPERATION_ID_LENGTH] = {0};
char operationId[256];
    char assignedHub[256];


const char *CONFIGURATION_DATA = "{\n\
    \"configuration\": {\n\
        \"version\":\"1\",\n\
        \"deviceId\":\"hortikit\",\n\
        \"scopeId\":\"0ne006E0511\",\n\
        \"DPSServer\":\"global.azure-devices-provisioning.net\",\n\
        \"modelId\":\"dtmi:wurthelektronik:designkit:adrastea;1\",\n\
        \"SNTPServer\":\"0.de.pool.ntp.org\",\n\
        \"timezone\":\"60\"\n\
    }\n\
}";


/**
 * @brief Timeouts for responses to AT commands (milliseconds).
 * Initialization is done in Adrastea_Init().
 */
static uint32_t Adrastea_timeouts[Adrastea_Timeout_NumberOfValues] = {
		0 };

/**
 * @brief Is set to true when sending an AT command and is reset to false when the response has been received.
 */
static bool Adrastea_requestPending = false;

/**
 * @brief Struct to hold current command response expected lines to skip
 */
static Adrastea_Response_Complete_t Adrastea_responseSkip = {
		.delim = '\0',
		.lineskip = 0 };

/**
 * @brief Is set to false when a command is received and response lines checked.
 */
static bool Adrastea_shouldCheckResponseSkip = true;

/**
 * @brief Name of the command last sent using Adrastea_SendRequest() (without prefix "AT+").
 */
static char Adrastea_pendingCommandName[64] = {
		0 };

/**
 * @brief Length of Adrastea_pendingCommandName.
 */
static size_t Adrastea_pendingCommandNameLength = 0;

/**
 * @brief Buffer used for current response text.
 */
static char Adrastea_currentResponseText[ADRASTEA_MAX_RESPONSE_TEXT_LENGTH];

/**
 * @brief Length of text in current response text buffer.
 * @see Adrastea_currentResponseText
 */
static size_t Adrastea_currentResponseLength = 0;

/**
 * @brief Confirmation status of the current (last issued) command.
 */
static Adrastea_CNFStatus_t Adrastea_cmdConfirmStatus;

/**
 * @brief Data buffer for received data.
 */
static char Adrastea_rxBuffer[ADRASTEA_LINE_MAX_SIZE];

/**
 * @brief Number of bytes in receive buffer.
 * @see Adrastea_rxBuffer
 */
static uint16_t Adrastea_rxByteCounter = 0;

/**
 * @brief Is set to true when the first EOL character (default: carriage return)
 * has been found in the current response text.
 */
static bool Adrastea_eolChar1Found = false;

/**
 * @brief First EOL character expected for responses received from Adrastea.
 * @see Adrastea_eolChar2, Adrastea_twoEolCharacters, Adrastea_SetEolCharacters()
 */
static uint8_t Adrastea_eolChar1 = '\r';

/**
 * @brief Second EOL character expected for responses received from Adrastea.
 * Only applicable if Adrastea_twoEolCharacters is true.
 * @see Adrastea_eolChar1, Adrastea_twoEolCharacters, Adrastea_SetEolCharacters()
 */
static uint8_t Adrastea_eolChar2 = '\n';

/**
 * @brief Controls whether a line of text received from Adrastea is considered complete after one or two EOL characters.
 * @see Adrastea_eolChar1, Adrastea_eolChar2, Adrastea_SetEolCharacters()
 */
static bool Adrastea_twoEolCharacters = true;

/**
 * @brief Is set to true after the echoed command is received and set to false when the response is received.
 */
static bool Adrastea_responseincoming = false;

/**
 * @brief Is set to true when module is in at mode and set to false otherwise.
 */
static volatile bool Adrastea_ATMode = false;

/**
 * @brief Pin configuration array.
 * The values in Adrastea_Pin_t are used to access the entries in this array.
 */
//static WE_Pin_t Adrastea_pins[Adrastea_Pin_Count] = { 0 };

/**
 * @brief Time step (microseconds) when waiting for responses from Adrastea.
 *
 * Note that WE_MICROSECOND_TICK needs to be defined to enable microsecond timer resolution.
 *
 * @see Adrastea_SetTimingParameters
 */
static uint32_t Adrastea_waitTimeStepUsec = 2 * 1000;

/**
 * @brief Minimum interval (microseconds) between subsequent commands sent to Adrastea
 * (more precisely the minimum interval between the confirmation of the previous command
 * and the sending of the following command).
 *
 * Note that WE_MICROSECOND_TICK needs to be defined to enable microsecond timer resolution.
 *
 * @see Adrastea_SetTimingParameters
 */
static uint32_t Adrastea_minCommandIntervalUsec = 3 * 1000;

/**
 * @brief Time (microseconds) of last confirmation received from Adrastea (if any).
 *
 * Note that WE_MICROSECOND_TICK needs to be defined to enable microsecond timer resolution.
 */
static uint32_t Adrastea_lastConfirmTimeUsec = 0;

/**
 * @brief Callback function for events.
 */
Adrastea_EventCallback_t Adrastea_eventCallback;

/**
 * @bief Is set to true if currently executing a (custom) event handler.
 */
static bool Adrastea_executingEventCallback = false;

/**
 * @brief Callback function which is executed if a single byte has been received from Adrastea.
 * The default callback is Adrastea_HandleRxByte().
 * @see Adrastea_SetByteRxCallback()
 */
Adrastea_ByteRxCallback_t Adrastea_byteRxCallback = NULL;

/**
 * @brief Optional callback function which is executed if a line is received from Adrastea.
 * Can be used to intercept responses from Adrastea.
 * @see Adrastea_SetLineRxCallback()
 */
Adrastea_LineRxCallback_t Adrastea_lineRxCallback = NULL;

/**
 * @brief Number of optional parameters delimiters so far (used by at commands).
 *
 */
uint8_t Adrastea_optionalParamsDelimCount = 0;

/**
 * @brief bool to indicate if the MCU is currently in the process of waking up ffrom sleep.
 *
 */
static bool Adrastea_wakingUp = false;

/**
 * @brief Major firmware version of the driver.
 */
uint8_t Adrastea_firmwareVersionMajor = 6;

/**
 * @brief Minor firmware version of the driver.
 */
uint16_t Adrastea_firmwareVersionMinor = 6;

/**
 * @brief Initializes the serial communication with the module
 *
 * @param[in] baudrate       Baud rate of the serial communication
 * @param[in] flowControl    Flow control setting for the serial communication
 * @param[in] parity         Parity bit configuration for the serial communication
 * @param[in] eventCallback  Function pointer to event handler (optional)
 * @param[in] pins           Pin configuration, pointer to array with Adrastea_Pin_Count elements
 *                           (optional, default configuration is used if NULL, see Adrastea_GetDefaultPinConfig())

 * @return true if successful, false otherwise
 */
//bool Adrastea_Init(uint32_t baudrate, WE_FlowControl_t flowControl, WE_Parity_t parity, Adrastea_EventCallback_t eventCallback, WE_Pin_t *pins){}

/**
 * @brief Deinitializes the serial communication with the module.
 *
 * @return true if successful, false otherwise
 */
bool Adrastea_Deinit(void)
{}

/**
 * @brief Performs a reset of the module using the reset pin.
 *
 * @return true if successful, false otherwise
 */
bool Adrastea_PinReset(void)
{}

/**
 * @brief Wakes the module up from power save mode using the wake up pin.
 *
 * @return true if successful, false otherwise
 */
bool Adrastea_PinWakeUp(void)
{}

/**
 * @brief Sets pin level to high or low.
 *
 * @param[in] pin Output pin to be set
 * @param[in] level Output level to be set
 *
 * @return true if successful, false otherwise
 */
//bool Adrastea_SetPin(Adrastea_Pin_t pin, WE_Pin_Level_t level){}

/**
 * @brief Read current pin level.
 *
 * @param[in] pin Pin to be read
 *
 * @return Current level of pin
 */
//WE_Pin_Level_t Adrastea_GetPinLevel(Adrastea_Pin_t pin){}

/**
 * @brief Sends the supplied AT command to the module
 *
 * @param[in] data AT command to send. Note that the command has to end with "\r\n\0".
 *
 * @return true if successful, false otherwise
 */
bool Adrastea_SendRequest(char *data)
{
	//printf("Inside adrastea_sendreq 2 function \n");

	size_t dataLength = strlen(data);

	 if (Adrastea_executingEventCallback)
	{
		// Don't allow sending AT commands from event handlers, as this will mess up send/receive states and buffers.
		return false;
	}

	Adrastea_requestPending = true;
	Adrastea_currentResponseLength = 0;

	// Make sure that the time between the last confirmation received from the module and the next command sent to the module is not shorter than Adrastea_minCommandIntervalUsec
	uint32_t t = WE_GetTickMicroseconds() - Adrastea_lastConfirmTimeUsec;
	if (t < Adrastea_minCommandIntervalUsec)
	{
		WE_DelayMicroseconds(Adrastea_minCommandIntervalUsec - t);
	}



	char delimiters[] = {
			ATCOMMAND_COMMAND_DELIM,
			'?',
			'\r' };

	// Get command name from request string (remove prefix "AT+" and parameters)
	Adrastea_pendingCommandName[0] = '\0';
	Adrastea_pendingCommandNameLength = 0;
	if (dataLength > 2 && data[0] == 'A' && data[1] == 'T')
	{
		char *pData;
		if (dataLength > 3 && (data[2] == '+' || data[2] == '%'))
		{
			pData = data + 3;
		}
		else
		{
			pData = data + 2;
		}
		if (ATCommand_GetCmdName(&pData, Adrastea_pendingCommandName, delimiters, sizeof(delimiters)))
		{
			Adrastea_pendingCommandNameLength = strlen(Adrastea_pendingCommandName);
		}
	}

#ifdef WE_DEBUG
	fprintf(stdout, "> %s", data);
#endif

	 Adrastea_Transmit(modemUartHandle, data, dataLength);

	return true;
}

/**
 * @brief Sends raw data to Adrastea via UART.
 *
 * This function sends data immediately without any processing and is used
 * internally for sending AT commands to Adrastea.
 *
 * @param[in] data Pointer to data buffer (data to be sent)
 * @param[in] dataLength Number of bytes to be sent
 */

void Adrastea_Transmit(serial_handle *handle, const char *buf, uint32_t len)
{
	 WE_UART_Transmit(handle,buf, len);
}


/**
 * @brief Waits for the response from the module after a request.
 *
 * @param[in] maxTimeMs Maximum wait time in milliseconds
 * @param[in] expectedStatus Status to wait for
 * @param[out] pOutResponse Received response text (if any) will be written to this buffer (optional)
 *
 * @return true if successful, false otherwise
 */

bool Adrastea_WaitForConfirm(uint32_t maxTimeMs, Adrastea_CNFStatus_t expectedStatus, char *pOutResponse)
{
	Adrastea_cmdConfirmStatus = Adrastea_CNFStatus_Invalid;

	uint32_t t0 = WE_GetTick();

	while (1)
	{
		if (Adrastea_CNFStatus_Invalid != Adrastea_cmdConfirmStatus)
		{
			/* Store current time to enable check for min. time between received confirm and next command. */
			Adrastea_lastConfirmTimeUsec = WE_GetTickMicroseconds();
			Adrastea_requestPending = false;
			Adrastea_responseincoming = false;
			if (Adrastea_cmdConfirmStatus == expectedStatus)
			{
				if (NULL != pOutResponse)
				{
					/* Copy response for further processing */
					memcpy(pOutResponse, Adrastea_currentResponseText, Adrastea_currentResponseLength);
				}

				return true;
			}
			else
			{
				//printf("Write nok 7.\r\n");
				return false;
			}
		}

		uint32_t now = WE_GetTick();
		// Check if "OK" response was received
		if (OkResponseReceived) {
			OkResponseReceived = false; // Reset the flag
			prevResponse = true;
			//printf("OK RECEIVED IN THE FUNCTION\n");
			break;
		}

		if (now - t0 > maxTimeMs)
		{
			/* Timeout */
			break;
		}

		if (Adrastea_waitTimeStepUsec > 0)
		{

			WE_DelayMicroseconds(Adrastea_waitTimeStepUsec);
		}
	}

	Adrastea_responseincoming = false;
	Adrastea_requestPending = false;
	return true;
	//return false;
}


/**
 * @brief Count the number of arguments in argument string.
 *
 * @param[in] Argument String to check
 *
 * @return number of arguments in argument string
 */
int Adrastea_CountArgs(char *stringP)
{}

/**
 * @brief Check if the response of the command is expected to be more the one line and fill Adrastea_responseSkip accordingly.
 */
void Adrastea_CheckResponseComplete()
{}

/**
 * @brief Set timing parameters used by the Adrastea driver.
 *
 * Note that WE_MICROSECOND_TICK needs to be defined to enable microsecond timer resolution.
 *
 * @param[in] waitTimeStepUsec Time step (microseconds) when waiting for responses from Adrastea.
 * @param[in] minCommandIntervalUsec Minimum interval (microseconds) between subsequent commands sent to Adrastea.
 *
 * @return true if successful, false otherwise
 */
bool Adrastea_SetTimingParameters(uint32_t waitTimeStepUsec, uint32_t minCommandIntervalUsec)
{}

/**
 * @brief Sets the timeout for responses to AT commands of the given type.
 *
 * @param[in] type Timeout (i.e. command) type
 * @param[in] timeout Timeout in milliseconds
 */
void Adrastea_SetTimeout(Adrastea_Timeout_t type, uint32_t timeout)
{
	Adrastea_timeouts[type] = timeout;
}

/**
 * @brief Gets the timeout for responses to AT commands of the given type.
 *
 * @param[in] type Timeout (i.e. command) type
 *
 * @return Timeout in milliseconds
 */
uint32_t Adrastea_GetTimeout(Adrastea_Timeout_t type)
{
	return Adrastea_timeouts[type];
}

/**
 * @brief Returns the default pin configuration.
 *
 * @param[out] pins Pin configuration array of length Adrastea_Pin_Count
 */


// void Adrastea_GetDefaultPinConfig(WE_Pin_t *pins){}

void WE_UART_HandleRxByte(uint8_t receivedByte)
{
	//Adrastea_byteRxCallback(receivedByte);
}

/**
 * @brief Default byte received callback.
 *
 * Is called when a single byte has been received.
 *
 * @param[in] receivedByte The received byte.
 */

void Adrastea_HandleRxByte(uint8_t receivedByte){
	 if (receivedByte == Adrastea_eolChar1 && Adrastea_rxByteCounter == 0)
	 	{
	 		return;
	 	}

	 	if (receivedByte == Adrastea_eolChar2 && !Adrastea_eolChar1Found)
	 	{
	 		return;
	 	}

	 	if (Adrastea_rxByteCounter >= ADRASTEA_LINE_MAX_SIZE)
	 	{
	 		Adrastea_rxByteCounter = 0;
	 		Adrastea_eolChar1Found = false;
	 		return;
	 	}

	 	if (receivedByte == Adrastea_eolChar1)
	 	{

	 		Adrastea_eolChar1Found = true;

	 		if (Adrastea_shouldCheckResponseSkip)
	 		{
	 			Adrastea_CheckResponseComplete();
	 			Adrastea_shouldCheckResponseSkip = false;
	 		}

	 		if (!Adrastea_twoEolCharacters)
	 		{
	 			if (Adrastea_responseSkip.lineskip == 0)
	 			{
	 				Adrastea_rxBuffer[Adrastea_rxByteCounter] = '\0';
	 				Adrastea_rxByteCounter++;
	 				Adrastea_HandleRxLine(Adrastea_rxBuffer, Adrastea_rxByteCounter);
	 				Adrastea_eolChar1Found = false;
	 				Adrastea_rxByteCounter = 0;
	 				Adrastea_shouldCheckResponseSkip = true;
	 			}
	 			else
	 			{
	 				Adrastea_responseSkip.lineskip -= 1;
	 				Adrastea_eolChar1Found = false;
	 				Adrastea_rxBuffer[Adrastea_rxByteCounter] = Adrastea_responseSkip.delim;
	 				Adrastea_rxByteCounter++;
	 			}
	 		}
	 	}
	 	else if (Adrastea_eolChar1Found)
	 	{
	 		if (receivedByte == Adrastea_eolChar2)
	 		{

	 			if (Adrastea_responseSkip.lineskip == 0)
	 			{
	 				/* Interpret it now */
	 				Adrastea_rxBuffer[Adrastea_rxByteCounter] = '\0';
	 				Adrastea_rxByteCounter++;
	 				Adrastea_HandleRxLine(Adrastea_rxBuffer, Adrastea_rxByteCounter);
	 				Adrastea_eolChar1Found = false;
	 				Adrastea_rxByteCounter = 0;
	 				Adrastea_shouldCheckResponseSkip = true;
	 			}
	 			else
	 			{
	 				Adrastea_responseSkip.lineskip -= 1;
	 				Adrastea_eolChar1Found = false;
	 				Adrastea_rxBuffer[Adrastea_rxByteCounter] = Adrastea_responseSkip.delim;
	 				Adrastea_rxByteCounter++;
	 			}
	 		}
	 	}
	 	else
	 	{
	 		Adrastea_rxBuffer[Adrastea_rxByteCounter++] = receivedByte;
	 	}
 }

/**
 * @brief Is called when a complete line has been received.
 *
 * @param[in] rxPacket Received text
 * @param[in] rxLength Received text length
 */
void Adrastea_HandleRxLine(char *rxPacket, uint16_t rxLength){

	printf("Rx byte < %s\r\n", rxPacket);
	/* Check if a custom line rx callback is specified and call it if so */
	if (NULL != Adrastea_lineRxCallback)
	{
		/* Custom callback returns true if the line has been handled */
		bool handled = Adrastea_lineRxCallback(rxPacket, rxLength);
		if (handled)
		{
			return;
		}
	}
	if (Adrastea_requestPending)
	{
		if (0 == strncmp(&rxPacket[0], ADRASTEA_RESPONSE_OK, strlen(ADRASTEA_RESPONSE_OK)))
		{
			Adrastea_responseincoming = false;
			Adrastea_cmdConfirmStatus = Adrastea_CNFStatus_Success;
		}
		else if (0 == strncmp(&rxPacket[0], ADRASTEA_RESPONSE_ERROR, strlen(ADRASTEA_RESPONSE_ERROR)))
		{
			Adrastea_responseincoming = false;
			Adrastea_cmdConfirmStatus = Adrastea_CNFStatus_Failed;
		}
		else if (0 == strncmp(&rxPacket[0], ADRASTEA_SMS_ERROR, strlen(ADRASTEA_SMS_ERROR)))
		{
			Adrastea_responseincoming = false;
			Adrastea_cmdConfirmStatus = Adrastea_CNFStatus_Failed;
		}
		else if (0 == strncmp(&rxPacket[0], ADRASTEA_POWERMODECHANGE_EVENT, strlen(ADRASTEA_POWERMODECHANGE_EVENT)))
		{
			Adrastea_responseincoming = false;
			Adrastea_cmdConfirmStatus = Adrastea_CNFStatus_Success;
		}
		else if (0 == strncmp(&rxPacket[0], ADRASTEA_SLEEPSET_EVENT, strlen(ADRASTEA_SLEEPSET_EVENT)))
		{
			Adrastea_responseincoming = false;
			Adrastea_cmdConfirmStatus = Adrastea_CNFStatus_Success;
		}
		else
		{
			if (Adrastea_responseincoming)
			{
				/* Copy to response text buffer, taking care not to exceed buffer size */
				uint16_t chunkLength = rxLength;
				bool isevent = false;

				if (Adrastea_currentResponseLength == 0 && rxLength > (Adrastea_pendingCommandNameLength + 2) && (0 == strncmp(Adrastea_pendingCommandName, rxPacket + 1, Adrastea_pendingCommandNameLength)))
				{
					isevent = true;
					chunkLength -= (Adrastea_pendingCommandNameLength + 2);
				}

				if (Adrastea_currentResponseLength + chunkLength >= ADRASTEA_MAX_RESPONSE_TEXT_LENGTH)
				{
					chunkLength = ADRASTEA_MAX_RESPONSE_TEXT_LENGTH - Adrastea_currentResponseLength;
				}

				if (Adrastea_currentResponseLength == 0)
				{
					if (isevent)
					{
						memcpy(&Adrastea_currentResponseText[Adrastea_currentResponseLength], Adrastea_rxBuffer + Adrastea_pendingCommandNameLength + 2, chunkLength);
					}
					else
					{
						memcpy(&Adrastea_currentResponseText[Adrastea_currentResponseLength], Adrastea_rxBuffer, chunkLength);
					}
				}
				else
				{
					memcpy(&Adrastea_currentResponseText[Adrastea_currentResponseLength], Adrastea_rxBuffer, chunkLength);
				}
				Adrastea_currentResponseLength += chunkLength;
			}
			else if (rxLength < ADRASTEA_LINE_MAX_SIZE && rxLength > 2 && Adrastea_pendingCommandName[0] != '\0' && 'A' == rxPacket[0] && 'T' == rxPacket[1] && ((('+' == rxPacket[2] || '%' == rxPacket[2]) && (0 == strncmp(Adrastea_pendingCommandName, Adrastea_rxBuffer + 3, Adrastea_pendingCommandNameLength))) || (0 == strncmp(Adrastea_pendingCommandName, Adrastea_rxBuffer + 2, Adrastea_pendingCommandNameLength))))
			{
				//if true command echo detected store response for lines after
				Adrastea_responseincoming = true;
			}
		}
	}

	if (0 == strncmp(&rxPacket[0], ADRASTEA_MCU_EVENT, strlen(ADRASTEA_MCU_EVENT)))
	{
		Adrastea_ATMode = false;
		if (!Adrastea_wakingUp)
		{
			Adrastea_Transmit(modemUartHandle,"map\r\n", 5);
		}
	}
	else if (0 == strncmp(&rxPacket[0], ADRASTEA_MAPCLICLOSE_EVENT, strlen(ADRASTEA_MAPCLICLOSE_EVENT)))
	{
		Adrastea_ATMode = false;
	}
	else if (0 == strncmp(&rxPacket[0], ADRASTEA_MAPCLIOPEN_EVENT, strlen(ADRASTEA_MAPCLIOPEN_EVENT)))
	{
		Adrastea_ATMode = true;
	}
	else if (('+' == rxPacket[0]) || ('%' == rxPacket[0]))
	{
		if (NULL != Adrastea_eventCallback)
		{
			/* Execute callback (if specified). */
			Adrastea_executingEventCallback = true;
			Adrastea_eventCallback(Adrastea_rxBuffer);
			Adrastea_executingEventCallback = false;
		}
	}

 }

/**
 * @brief Sets the callback function which is executed if a byte has been received from Adrastea.
 *
 * The default callback is Adrastea_HandleRxByte().
 *
 * @param[in] callback Pointer to byte received callback function (default callback is used if NULL)
 */
void Adrastea_SetByteRxCallback(Adrastea_ByteRxCallback_t callback)
{
	Adrastea_byteRxCallback = (callback == NULL) ? Adrastea_HandleRxByte : callback;
}

/**
 * @brief Sets an optional callback function which is executed if a line has been received from Adrastea.
 * Can be used to intercept responses from Adrastea.
 *
 * The callback function must return true if the line should not be processed by the driver
 * and false if the driver should process the line as usual.
 *
 * @param[in] callback Pointer to line received callback function
 */
void Adrastea_SetLineRxCallback(Adrastea_LineRxCallback_t callback)
{}

/**
 * @brief Sets EOL character(s) used for interpreting responses from Adrastea.
 *
 * @param[in] eol1 First EOL character
 * @param[in] eol2 Second EOL character (is only used if twoEolCharacters is true)
 * @param[in] twoEolCharacters Controls whether the two EOL characters eol1 and eol2 (true) or only eol1 (false) is used
 */
void Adrastea_SetEolCharacters(uint8_t eol1, uint8_t eol2, bool twoEolCharacters)
{}

/**
 * @brief Check if module is in at command mode
 *
 * @return boolean to indicate mode.
 */
bool Adrastea_CheckATMode()
{
	printf("test");
}

// ========================================================== NEW IoTCentral PnP functions  ===================================================================================

/* extern void rxUartTask(void *pvParameters) {

	int currentChar = 0;
	while (1) {
		char c;
		if (serial_read(modemUartHandle, &c, 1) > 0) {
			// Print received character to user UART
			serial_write(userUartHandle, &c, 1);

			// Check for newline character carriage return, or space character to process response
			 if (c == '\n' || c == '\r' ) {
				// Null-terminate the response
				responseBuffer[responseIndex] = '\0';
				// printf("Here is the response buffer: %s\n", responseBuffer);
				// Check if the response is "OK" or "ERROR"
				if (strcmp(responseBuffer, "OK") == 0) {
					// Handle "OK" response
					//printf("RECEIVED OK ");
					OkResponseReceived = true;
					prevResponse = true;
				} else if (strcmp(responseBuffer, "ERROR") == 0) {
					// Handle "ERROR" response
					//printf("RECEIVED ERR ");
					ErrorResponseReceived = true;
					prevResponse = false;
				}
				else if (strncmp(responseBuffer, "+CEREG: 5", strlen("+CEREG: 5")) == 0) {
					// Check if connected to base station, registered and roaming
					printf("SIM is Registered - +CEREG: 5,\r\n");
					SIM_Registered = true;
				}

				else  if (strncmp(responseBuffer, "%MQTTEVU:\"CONCONF\"", strlen("%MQTTEVU:\"CONCONF\"")) == 0) {
					printf("Connected to MQTT Broker\r\n");

				}


			}
		}

	}
} */

extern void rxUartTask(void *pvParameters) {
	int currentChar = 0;
	char c,s;
	while (1) {


		if (serial_read(modemUartHandle, &c, 1) > 0) {
					// Print received character to user UART
					serial_write(userUartHandle, &c, 1);
					//printf("data received modem: %c",c);
		}
	}
}


int Init_Uart()
{
	// ACTIVE_UARTIO is connection of internal UART between modem and CortexM4. MINICONSOLE_UART_INSTANCE (ACTIVE_UARTF0) is connect between Cortex M4 and host PC i.e external UART
	sHuart uartInit;
	if (serial_load_defconfig(MINICONSOLE_UART_INSTANCE, &uartInit) != 0) {
		/*UART default configuration error*/
		return WE_FAIL;
	}

	// Open serial and set it as stdout for printf
	userUartHandle = serial_open(&uartInit);
	newlib_set_stdout_port(MINICONSOLE_UART_INSTANCE);

	if (serial_load_defconfig(ACTIVE_UARTI0, &uartInit) != 0) {
		/*UART default configuration error*/
		return WE_FAIL;
	}

	else{
		serial_write(userUartHandle, "UART Connection between Application MCU to Host MCU is setup\r\n", strlen("UART Connection between Application MCU to Host MCU is setup\r\n"));
	}

	modemUartHandle = serial_open(&uartInit);

	// Create task for actively checking data between Internal UART and sending to external UART
	xTaskCreate(rxUartTask, /* The function that implements the task. */
			"debugUart", /* The text name assigned to the task - for debug only as it is not used by
						the kernel. */
			configMINIMAL_STACK_SIZE * 2, /* The size of the stack to allocate to the task. */
			(void *)NULL,          /* The parameter passed to the task */
			configMAX_PRIORITIES - 1, /* The priority assigned to the task - minimal priority. */
			&gDebugUartHandle);

	return WE_SUCCESS;
}

int connect_mqttdashboard(){
	strcpy(cmdBuffer,"AT\r\n");
	serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
	WE_Delay(2000);

	// Set MQTT Event to All
	bool ret = false;
	ret = ATMQTT_SetMQTTUnsolicitedNotificationEvents(ATMQTT_Event_All, 1);
	WE_Delay(4000);

	//sprintf(cmdBuffer, "AT%%MQTTCFG=\"NODES\",1,\"adrastea-test-dev-1\",\"broker.hivemq.com\"\r\n");
	//serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
	ret =  ATMQTT_ConfigureNodes(1,"adrastea-test-dev-1","broker.hivemq.com",NULL,NULL);
	//AdrasteaExamplesPrint("Configure Node", ret);
	WE_Delay(4000);

	ret = false;
	ret = ATMQTT_ConfigureProtocol(1, 1200, 1);
	//AdrasteaExamplesPrint("Configure Protocol", ret);
	WE_Delay(4000);

	ret = ATMQTT_Connect(1);
	//AdrasteaExamplesPrint("Connect", ret);
	WE_Delay(4000);
}

// Publish to IoTCentral
int IoTCentral_publish(float temperature_PADS, float pressure_PADS, float temperature_TIDS, float ITDSsensorData_temperature, float ITDSsensorData_Xaxis_float, float ITDSsensorData_Yaxis_float, float ITDSsensorData_Zaxis_float){
	sprintf(cmdBuffer, "AT%%MQTTCMD=\"PUBLISH\",1,0,0,\"devices/adrastea-test-dev-1/messages/events\",27\r\n");
	serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
	WE_Delay(2400);
	sprintf(cmdBuffer, "{Temperature: \"%f\"}\r\n",temperature_PADS);
	serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
	WE_Delay(PERIOD_IN_MS);
}

// Publish to mqttdashboard public broker
/*int mqttdashboard_publish(){
	memset(test_str, 0, sizeof(test_str));
	test_str_index = 0;
	printf(" At mqttpublish func\r\n");
	// to read from MiniConsole of Host MCU
	do {
		serial_read(userUartHandle, &c, 1);
		if (c != 10) {
			//serial_write(modemUartHandle, &c, 1);
			serial_write(userUartHandle, &c, 1);
			printf("%c",c);
			//printf("%c",c);
			test_str[test_str_index++] = c; // Store the character in test_str array
		}
	} while (c != 10);


	test_str[test_str_index] = '\0'; // Null-terminate the string
	printf("MAP CLI Closed.\r\nUser input:\r\n");
	printf("%s\r\n",test_str); // Print the collected input
	size_t len = strlen(test_str);
	if (len >= 4) {
		// Remove the first 4 characters
		memmove(test_str, test_str + 4, len - 3);}

	printf("Modified string: %s\r\n", test_str);
	test_str_len = strlen(test_str)+1;

	char* substr = strstr(test_str, "detected");

	if (substr != NULL) {
		printf("Bug found.\r\n");
	} else {
		printf("Bug not available.\r\n");
	}

	sprintf(cmdBuffer, "AT%%MQTTCMD=\"PUBLISH\",1,0,0,\"Adra\",%d\r\n", test_str_len);
	serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
	printf("Publishing Data\r\n");
	WE_Delay(2400);
	sprintf(cmdBuffer, "%s\r\n", test_str);
	serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
} */

/*int mqttdashboard_publish(){
	memset(test_str, 0, sizeof(test_str));
	test_str_index = 0;
	printf(" At mqttpublish func\r\n");
	 int flag_prev_char_cr = 0;
	// to read from MiniConsole of Host MCU
	    do {
	        serial_read(userUartHandle, &c, 1);

	        if (c == '\r') {
	            flag_prev_char_cr = 1;  // Set the flag if the character is '\r'
	        } else if (c == '\n' && flag_prev_char_cr == 1) {
	            // Break the loop if current character is '\n' and previous was '\r'
	            break;
	        } else {
	            flag_prev_char_cr = 0;  // Reset the flag if the character is not '\r'
	            //serial_write(modemUartHandle, &c, 1);
	            serial_write(userUartHandle, &c, 1);
	            //printf("%c", c);
	            test_str[test_str_index++] = c;
	        }
	    } while (1);


	test_str[test_str_index] = '\0'; // Null-terminate the string
	printf("User input:\r\n");
	printf("%s\r\n",test_str); // Print the collected input
	size_t len = strlen(test_str);
	if (len >= 4) {
		// Remove the first 4 characters
		memmove(test_str, test_str + 4, len - 3);}

	printf("Modified string: %s\r\n", test_str);
	test_str_len = strlen(test_str)+1;

	char* substr = strstr(test_str, "detected");

	if (substr != NULL) {
		printf("Bug found.\r\n");
	} else {
		printf("Bug not available.\r\n");
	}
	//memset(cmdBuffer, 0, sizeof(cmdBuffer));
	sprintf(cmdBuffer, "AT%%MQTTCMD=\"PUBLISH\",1,0,0,\"Adra\",%d\r\n", test_str_len);
	serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
	//printf("Publishing Data\r\n");
	WE_Delay(2400);
	sprintf(cmdBuffer, "%s\r\n", test_str);
	serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
}
*/

int mqttdashboard_publish(){
	memset(test_str, 0, sizeof(test_str));
	test_str_index = 0;
	printf(" At mqttpublish func\r\n");
	 int flag_prev_char_cr = 0;
	// to read from MiniConsole of Host MCU
	    do {
	        serial_read(userUartHandle, &c, 1);

	        if (c == '\r') {
	            flag_prev_char_cr = 1;  // Set the flag if the character is '\r'
	        } else if (c == '\n' && flag_prev_char_cr == 1) {
	            // Break the loop if current character is '\n' and previous was '\r'
	            break;
	        } else {
	            flag_prev_char_cr = 0;  // Reset the flag if the character is not '\r'
	            //serial_write(modemUartHandle, &c, 1);
	            serial_write(userUartHandle, &c, 1);
	            //printf("%c", c);
	            test_str[test_str_index++] = c;
	        }
	    } while (1);


	test_str[test_str_index] = '\0'; // Null-terminate the string
	printf("User input:\r\n");
	printf("%s\r\n",test_str); // Print the collected input
	size_t len = strlen(test_str);
	if (len >= 4) {
		// Remove the first 4 characters
		memmove(test_str, test_str + 4, len - 3);}

	printf("Modified string: %s\r\n", test_str);
	test_str_len = strlen(test_str)+1;

	char* substr = strstr(test_str, "detected");

	if (substr != NULL) {
		printf("Bug found.\r\n");
		sprintf(cmdBuffer, "AT%%MQTTCMD=\"PUBLISH\",1,0,0,\"Adra\",%d\r\n", test_str_len);
		serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
		//printf("Publishing Data\r\n");
		WE_Delay(2400);
		sprintf(cmdBuffer, "%s\r\n", test_str);
		serial_write(modemUartHandle, cmdBuffer, strlen(cmdBuffer));
	} else {
		printf("Bug not available.\r\n");
	}
	//memset(cmdBuffer, 0, sizeof(cmdBuffer));

}


