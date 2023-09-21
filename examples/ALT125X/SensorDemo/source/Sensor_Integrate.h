#ifndef SENSOR_INTEGRATE_H_
#define SENSOR_INTEGRATE_H_

#define DEBUG_OUTPUT
//  #include "../Eclipse/WE_Sensor/Adrastea/ATCommands/ATPacketDomain.h"

#define PERIOD_IN_MS 10000

#define azrootca "-----BEGIN CERTIFICATE-----\n\
MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\n\
RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\n\
VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\n\
DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\n\
ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\n\
VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\n\
mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\n\
IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\n\
mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\n\
XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\n\
dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\n\
jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\n\
BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\n\
DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\n\
9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\n\
jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\n\
Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\n\
ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\n\
R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\n\
-----END CERTIFICATE-----"

 #define azdevcert "-----BEGIN CERTIFICATE-----\n\
MIIBxTCCAWygAwIBAgIEZLUHxTAKBggqhkjOPQQDAjAfMQswCQYDVQQGEwJERTEQ\n\
MA4GA1UEAxMHUm9vdCBDQTAeFw0yMzA3MTYwOTIwMDVaFw0yNDAxMTcwOTIwMDVa\n\
MB4xHDAaBgNVBAMTE2FkcmFzdGVhLXRlc3QtZGV2LTEwWTATBgcqhkjOPQIBBggq\n\
hkjOPQMBBwNCAASPTsD8iDZ0mc3SfsodD5x93iaYKjb8NHrNJsgJS34Q9W6OuKBu\n\
aD7Ly7ueY+LGGvJCaZzg80Xs8w0cR+JFCACTo4GWMIGTMAwGA1UdEwEB/wQCMAAw\n\
DgYDVR0PAQH/BAQDAgWgMB8GA1UdIwQYMBaAFBpgE4wItCWKL94gvAUmG540BQyV\n\
MBQGA1UdEQQNMAuCCWxvY2FsaG9zdDAdBgNVHSUEFjAUBggrBgEFBQcDAgYIKwYB\n\
BQUHAwEwHQYDVR0OBBYEFBec0TwfOnHmvCsLTUXbhQWgnZgbMAoGCCqGSM49BAMC\n\
A0cAMEQCIEtbSV4T9vjm1R5q8R1LqHPXPO47G9hulf5jpV5v7G2iAiAcvAWNh5ch\n\
JESwdLeTz0p9aUDDb95NN8lpyRzCE8bVFg==\n\
-----END CERTIFICATE-----\n\
-----BEGIN CERTIFICATE-----\n\
MIIBrjCCAVWgAwIBAgIJAPwzOwtwik4VMAoGCCqGSM49BAMCMB8xCzAJBgNVBAYT\n\
AkRFMRAwDgYDVQQDEwdSb290IENBMB4XDTIzMDcxNjA5MjAwMVoXDTI0MDcxNzA5\n\
MjAwMVowHzELMAkGA1UEBhMCREUxEDAOBgNVBAMTB1Jvb3QgQ0EwWTATBgcqhkjO\n\
PQIBBggqhkjOPQMBBwNCAATcI9wOQr3PyXrqrvSOXkBoLE2Kc4tneBV59xVP7ns0\n\
kyzK5UEX+kxyeQkk8HFsIDNNTzS994uNGHedRKLaMfmso3oweDASBgNVHRMBAf8E\n\
CDAGAQH/AgEDMA4GA1UdDwEB/wQEAwICBDAUBgNVHREEDTALgglsb2NhbGhvc3Qw\n\
HQYDVR0lBBYwFAYIKwYBBQUHAwIGCCsGAQUFBwMBMB0GA1UdDgQWBBQaYBOMCLQl\n\
ii/eILwFJhueNAUMlTAKBggqhkjOPQQDAgNHADBEAiBhiubDTVmO0t48cMo6/uWG\n\
fRf22NfOE8nwEafdjyBtfQIgXnCQaqz7M7b8xBXI2QwbJfMQZ1FWM9E1EGVx3wFi\n\
ZeI=\n\
-----END CERTIFICATE-----"

#define azdevkey "-----BEGIN EC PRIVATE KEY-----\n\
MHcCAQEEIG4z8mQN3ZKo9oxbvp3/L9d+X8c8yoVZLNeEObCaphPZoAoGCCqGSM49AwEHoUQDQgAE\n\
j07A/Ig2dJnN0n7KHQ+cfd4mmCo2/DR6zSbICUt+EPVujrigbmg+y8u7nmPixhryQmmc4PNF7PMN\n\
HEfiRQgAkw==\n\
-----END EC PRIVATE KEY-----"



/*   #define azdevcert "-----BEGIN CERTIFICATE-----\n\
MIIBwDCCAWWgAwIBAgIEZQwmnzAKBggqhkjOPQQDAjAfMQswCQYDVQQGEwJERTEQ\n\
MA4GA1UEAxMHUm9vdCBDQTAeFw0yMzA5MjAxMTE4NTRaFw0yNDA1MjExMTE4Mzla\n\
MBcxFTATBgNVBAMTDGFkcmFzdGVhLW5ldzBZMBMGByqGSM49AgEGCCqGSM49AwEH\n\
A0IABKbjzGl5ulzse5gGIEjkxRgedXu/YwtsT/nuu74zVBr6UYQ48znK7manvRuU\n\
QMPFFdyjo3BSvfu7dUlyBllro6GjgZYwgZMwDAYDVR0TAQH/BAIwADAOBgNVHQ8B\n\
Af8EBAMCBaAwHwYDVR0jBBgwFoAUvgc1fylRJCShxeLpH+1DmlZ0xAcwFAYDVR0R\n\
BA0wC4IJbG9jYWxob3N0MB0GA1UdJQQWMBQGCCsGAQUFBwMCBggrBgEFBQcDATAd\n\
BgNVHQ4EFgQUJcHturgmg8vRnlvy3X/hnJYp25UwCgYIKoZIzj0EAwIDSQAwRgIh\n\
AIgoQ6XRELcflsUU1hDlojdUu9gDHGoKZyIcU1WNDJowAiEA6pQ+NJXjkCJlo+T2\n\
H4TmOVnKMs7UYKfX4NTaZRy00wk=\n\
-----END CERTIFICATE-----\n\
-----BEGIN CERTIFICATE-----\n\
MIIBrjCCAVSgAwIBAgIIR97pFTC3gLwwCgYIKoZIzj0EAwIwHzELMAkGA1UEBhMC\n\
REUxEDAOBgNVBAMTB1Jvb3QgQ0EwHhcNMjMwOTIwMTExODM5WhcNMjQwNTIxMTEx\n\
ODM5WjAfMQswCQYDVQQGEwJERTEQMA4GA1UEAxMHUm9vdCBDQTBZMBMGByqGSM49\n\
AgEGCCqGSM49AwEHA0IABLj1mlinnPX8MuLCicgDdTB9HtiKGFxbuPeGTmHSb8XY\n\
BUSpzVjEA/hYpoiYLxwV0ipFqI3R4SeydiOIeh9TEKmjejB4MBIGA1UdEwEB/wQI\n\
MAYBAf8CAQMwDgYDVR0PAQH/BAQDAgIEMBQGA1UdEQQNMAuCCWxvY2FsaG9zdDAd\n\
BgNVHSUEFjAUBggrBgEFBQcDAgYIKwYBBQUHAwEwHQYDVR0OBBYEFL4HNX8pUSQk\n\
ocXi6R/tQ5pWdMQHMAoGCCqGSM49BAMCA0gAMEUCIQCtSwGYHxCiFPjJVkO1uYvV\n\
exzeXQEFtrWyIim3psw8xwIgUIv13kUka66i7DDOd7jqfjJDjK1yYDPabZV3mGZ6\n\
vM0=\n\
-----END CERTIFICATE-----"   */


// Digicert

/* #define azrootca "-----BEGIN CERTIFICATE-----\n\
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n\
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n\
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n\
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n\
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n\
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n\
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n\
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n\
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n\
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n\
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n\
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n\
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n\
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n\
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n\
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n\
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n\
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n\
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n\
MrY=\n\
-----END CERTIFICATE-----" */



/*   #define azdevkey "-----BEGIN EC PRIVATE KEY-----\n\
MHcCAQEEIC11aenYZ9xoXRG+WhDPysUNo7WOseSX2fXKjgd9fE2foAoGCCqGSM49AwEHoUQDQgAE\n\
puPMaXm6XOx7mAYgSOTFGB51e79jC2xP+e67vjNUGvpRhDjzOcruZqe9G5RAw8UV3KOjcFK9+7t1\n\
SXIGWWujoQ==\n\
-----END EC PRIVATE KEY-----"   */





















#define AZURE_DPS_ADDRESS "global.azure-devices-provisioning.net"
#define AZURE_DPS_USERNAME "0ne006E0511/registrations/adrastea-test-dev-1/api-version=2021-06-01&model-id=dtmi:calypso:Adrastea_fa;1"
#define MAX_URL_LEN 64
#define DEVICE_CREDENTIALS_MAX_LEN 64

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

/*File path to certificate files stored on Adrastea internal storage*/
#define ROOT_CA_PATH "user/azrootca"
#define ROOT_CA_1_PATH "user/azrootca1"
#define DEVICE_CERT_PATH "user/azdevcert"
#define DEVICE_KEY_PATH "user/azdevkey"
#define DEVICE_IOT_HUB_ADDRESS "user/iotHubAddr"
#define CONFIG_FILE_PATH "user/azdevconf"

extern int startDemo();
extern int Init_TIDS();
extern int Init_ITDS();
extern int Init_PADS();
extern int Init_HIDS();

#endif
