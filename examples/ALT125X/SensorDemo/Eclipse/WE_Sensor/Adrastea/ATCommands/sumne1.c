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
 * @brief AT commands for Packet Domain functionality.
 */

// #include <stdio.h>
//#include "../global/ATCommands.h"
// #include "ATPacketDomain.h"
#include "Adrastea1.h"
#include "sumne1.h"
char AT_commandBuffer[2000];

_Bool ATPacketDomain_SetNetworkRegistrationResultCode4(resultcode)
{
	char *pRequestCommand = AT_commandBuffer;
	char strnu[3] = {0};
	strcpy(pRequestCommand, "AT+CEREG=");
	//printf("Inside ATPacket function \n");
	//sprintf(strnu,"%d",resultcode);
	//printf("strnu %s \n",strnu);
	//strcpy(pRequestCommand, strnu);


	printf("We adrastea transmit :%s \n",pRequestCommand);
	if (!Adrastea_SendRequest1(pRequestCommand))
	{
		return 0;
	}

	// Adrastea_SendRequest1()

	printf(" after adrastea sendcmd \n");


	return 1;
}

_Bool Adrastea_SendRequest1()
{
	printf("Inside adrastea_sendreq 2 function \n");
}

