# Software and hardware for requesting register contents and send them using MQTT

Read carefully the header of the .ino file.

you use this file and informations on your own risk !

This program and the circuit is designed and MUST be the ONLY device
connected to the Modbus port !

The program requests register values from the inverter / storage inverter and
publishes them by using MQTT to HomeAssistant with auto detection.

Don't forget to enter your credentials within this part of the .ino :


const char *WIFI_SSID = "xxxxxxxxx"; // Your SSiD

const char *WIFI_PASSWORD = "xxxxxxx"; // Your Wlan password

const char* mqttServer = "xxx.xxx.xxx.xxx"; // The IP of your MQTT broker

const int mqttPort = 1883;

const char* mqttUser = "xxxxxxx"; // Your mqtt user

const char* mqttPassword = "xxxxxx"; // Your mqtt password

#define SolisTZ TZ_Europe_Berlin // if needed replace TZ_Europe_Berlin with your TimeZone from ...\Arduino15\packages\esp8266\hardware\esp8266\3.1.2\cores\esp8266\TZ.h


if you want to know if this program will work with your
equipment check the content of the typedetect folder.
 
