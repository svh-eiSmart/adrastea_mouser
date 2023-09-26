#ifndef SENSOR_INTEGRATE_H_
#define SENSOR_INTEGRATE_H_

#define DEBUG_OUTPUT
//  #include "../Eclipse/WE_Sensor/Adrastea/ATCommands/ATPacketDomain.h"

#define PERIOD_IN_MS 10000

//MQTT Settings
#define DEVICE_SUB_TOPIC "$dps/registrations/res/#"
#define DEVICE_PUB_DPS_TOPIC "$dps/registrations/PUT/iotdps-register/?$rid=1"
#define DEVICE_PAYLOAD_DATA "{\"registrationId\":\"adrastea-test-dev-1\",\"payload\":{\"modelId\":\"dtmi:calypso:Adrastea_fa;1\"}}"

/*
char kitID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char scopeID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char modelID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char deviceID[DEVICE_CREDENTIALS_MAX_LEN] = {0};
char iotHubAddress[MAX_URL_LEN] = {0};
char dpsServerAddress[MAX_URL_LEN] = {0};
*/

#define DEFAULT_TELEMETRY_SEND_INTEVAL 30 // seconds
#define MAX_TELEMETRY_SEND_INTERVAL 600   // seconds
#define MIN_TELEMETRY_SEND_INTERVAL 3     // seconds

#endif
