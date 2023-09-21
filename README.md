* Added all sensors TIDS, ISDS, HIDS and PADS - working with Pure UART for connection between Application MCU (M4) and Modem (ALT1250), and connection to host MCU with external UART working.
* MQTT connection to DPS of Azure IoT_Central Cloud - working

Commands to install and run the repository
1) Follow steps in https://developer.sony.com/cellular-iot/developer-docs/1250_MCU_Software_Distribution_Manual_en.html to install required tools
2) Clone the repository and import the folder in Eclipse IDE
3) To flash the binary file, reset the controller, then navigate to utils folder in the terminal of Eclipse IDE and use the cmd:
python flashmcu.py -L COM<port_number> -B 115200 -F none ../examples/ALT125X/SensorDemo/Eclipse/ALT1250/SensorDemo.bin
check COM<port_number> and replace in the command (3rd UART COM)
4) Reset the controller again and open TeraTerm or any other terminal emulators to the 1st UART (-UART1)
