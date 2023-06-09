/* Solis to MQTT Requester V090623-2 Copyright @ Pit Hermann 01.04.2023 */

/* You use this software on your own risk !!!
 * The software is provided as it is.
 * 
 * needed libraries:
 * PubSubClient by Nick O’Leary
 * ArduinoJson by Benoit Blanchon
 * ArduinoOTA by Juraj Andrassy 
 * ESP Telnet by Lennart Hennigs
 * 
 * Important: the address of the inverter / Storage inverter MUST be set to 1
 * 
 * Changes:
 * 08-06-2023 : Devicetype 0x10 added
 * 09-06-2023 : detection of inverter / hybrid if devicetype register is not avaiable
 *              the software now should run with every (storage-)Inverter EXCEPT RHI-1P(5-10)K-HVES-5G-US,
 *              so prior type detection is no more needed.
 * 
 * 
 * Features:
 * EPM is actually only supported combined with storage inverters 
 * Naming: connected to an inverter to programm shows up as inverter01 in HomeAssistant,
 * connected to a storage inverter to programm shows up as hybrid01 in HomeAssistant.
 * If you want an other name  feel free to make use of find and replace-all with the desired name BUT
 * the new name must be equal or shorter than ten chars.
 * 
 * a Power_check value 0 = do nothing if the inverter shut down ; 1 = set most of the values to zero 
 * if the inverter shuts down.This option ONLY works with none storage inverters AND only while  
 * fourmppt = 0 ; if fourmppt = 1 or the software detects a storage-inverter this option is disabled 
 * in case of MQTT restrictions.
 * 
 * There is a timecheck routine implemented basing on ntp protocol. Actually TimeZone is set to Berlin
 * to change to yours replace in '#define SolisTZ TZ_Europe_Berlin' TZ_Europe_Berlin with your TimeZone
 * from ...\Arduino15\packages\esp8266\hardware\esp8266\3.1.2\cores\esp8266\TZ.h
 * The timecheck routine runs if used with inverters every morning of start one time, if used
 * with storage inverters every night shortly after 0:00.
 *  
 * There is a startup delay of around five minutes after power on for the inverter to get stable running.
 * A storage inverter is allways running so the five minutes delay occur only on connecting the circuit.
 * 
 * Value to entity mapping:
 * Hybrid;Entity;Inverter
 * Date;..._Date;Date
 * Time;..._Time;Time
 * Total energy generation;..._E_Total;Total Energy
 * Current month energy generation;..._E_Month;Energy This Month
 * Last month energy generation;..._E_LMonth;Energy Last Month
 * Today energy generation,..._E_Day;Energy Today
 * Yesterday energy generation;..._E_LDay;Energy Last day
 * This year energy generation;..._E_Year;Energy This Year   
 * Last year energy generation;..._E_LYear;Energy Last Year      
 * DC Voltage 1;..._V_DC1;DC Voltage 1
 * DC Current 1;..._I_DC1;DC Current 1
 * DC Voltage 2;..._V_DC2;DC Voltage 2
 * DC Current 2;..._I_DC2;DC Current 2
 * DC Voltage 3;..._V_DC3;DC Voltage 3 (Type = Inbverter only with fourmppt = 1 setting)
 * DC Current 3;..._I_DC3;DC Current 3 (Type = Inbverter only with fourmppt = 1 setting)
 * DC Voltage 4;..._V_DC4;DC Voltage 4 (Type = Inbverter only with fourmppt = 1 setting)
 * DC Current 4;..._I_DC4;DC Current 4 (Type = Inbverter only with fourmppt = 1 setting)
 * (calculated) Power DC1;..._P_DC1;(calculated) Power DC1
 * (calculated) Power DC2;..._P_DC2;(calculated) Power DC2
 * (calculated) Power DC3;..._P_DC3;(calculated) Power DC3 (Type = Inbverter only with fourmppt = 1 setting)
 * (calculated) Power DC4;..._P_DC4;(calculated) Power DC4 (Type = Inbverter only with fourmppt = 1 setting)
 * AB line voltage/A phase voltage;..._V_ACU;A Phase Voltage
 * BC line voltage /B phase voltage;..._V_ACV;B Phase Voltage
 * CA line voltage /C phase voltage;..._V_ACW;C Phase Voltage
 * A phase current;..._I_ACU;A Phase Current
 * B Phase Current;..._I_ACV;B Phase Current
 * C Phase Current;..._I_ACW;C Phase Current
 * Apparent power;..._P_AC_Out;Apparent power
 * Active Power;..._P_AC_Total;Active Power
 * Inverter Temperature;..._T_Inv;Inverter Temperature
 * (calculated) Power factor;..._P_Fact_AC;(calculated) Power factor
 * Battery voltage;..._V_BAT;-
 * Battery current;..._I_BAT;-
 * Battery capacity SOC;..._B_SOC;-
 * Battery health SOH;..._B_SOH;-
 * Battery current direction;..._B_Charge;-
 * Today energy imported from grid;..._E_Day_fr_Grid;-
 * Yesterday energy imported from grid;..._E_LDay_fr_Grid;-
 * Total energy imported from grid;..._E_Total_fr_Grid;-
 * Today energy fed into grid;..._E_Day_to_Grid;-
 * Yesterday energy fed into grid;..._E_LDay_to_Grid;-
 * Total energy fed into grid;..._E_Total_to_Grid;-
 * Backup AC voltage (Phase A);..._V_Backup;- 
 * Backup AC current;..._I_Backup;-
 * Backup load power;..._P_Backup;- 
 * Household load power;..._P_House;-
 * Meter ac voltage A;..._Meter_V_Phase_A;-
 * Meter ac current A;..._Meter_I_Phase_A;-
 * Meter ac voltage B;..._Meter_V_Phase_B;-
 * Meter ac current B;..._Meter_I_Phase_B;-
 * Meter ac voltage C;..._Meter_V_Phase_C;-
 * Meter ac current C;..._Meter_I_Phase_C;-
 * Meter active power A;..._Meter_act_P_Phase_A;-
 * Meter active power B;..._Meter_act_P_Phase_B;-
 * Meter active power C;..._Meter_act_P_Phase_C;-
 * Meter total active energy from grid;..._Meter_tot_E_fr_Grid;-
 * Meter total active energy to grid;..._Meter_tot_E_to_Grid;-
 * Meter PF;..._Meter_PF;-
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "ESPTelnet.h"
#include "EscapeCodes.h"
#include <time.h>
#include <sys/time.h>
#include <sntp.h>
#include <TZ.h>

ESPTelnet telnet;
IPAddress ip;
EscapeCodes ansi;
uint16_t  port = 23;
uint8_t telnet_debug = 0;

#define SERIAL_RX_BUFFER_SIZE 2048
#define SERIAL_TX_BUFFER_SIZE 500

const char *WIFI_SSID = "xxxxxxxxx"; // Your SSiD
const char *WIFI_PASSWORD = "xxxxxxx"; // Your Wlan password
const char* mqttServer = "xxx.xxx.xxx.xxx"; // The IP of your MQTT broker
const int mqttPort = 1883;
const char* mqttUser = "xxxxxxx"; // Your mqtt user
const char* mqttPassword = "xxxxxx"; // Your mqtt password
#define SolisTZ TZ_Europe_Berlin // if needed replace TZ_Europe_Berlin with your TimeZone from ...\Arduino15\packages\esp8266\hardware\esp8266\3.1.2\cores\esp8266\TZ.h
uint8_t Power_check = 1; // 0 = no check ; 1 = check on Power check pin
uint8_t fourmppt = 0; //Only used with none storage inverter; 0 = two MPPT ; 1 = 3 or 4 MPPT

String mqttName = "Solis";
String stateTopic = "solis/inverter01/state";
String stateTopicH = "solis/hybrid01/state";
String inv_sn,inv_date,inv_time,HOUR,MINUTE,SECOND,charge;
#define updateinterval 30 // Update interval using seconds
#define re_upd_conf 240000 // re-update configuration using milliseconds 
#define req_waiter 30 //request waiter using milliseconds 
#define Timer_1 200 //first shut
#define Timer_2 150 //second shut


uint8_t POWDT = 16; // Power check pin

uint8_t Power_check1 = 0;
uint16_t Decoderselect = 0;

uint8_t sensorupdateprogress = 0;
uint8_t reader = 0;
uint8_t msg_d[600] = {0};
uint8_t *point_to;
uint8_t result1;
uint8_t result2;
uint8_t Errorshut = 0;
uint8_t Errorshutcounter = 0;
uint8_t msg0[8] = {1,4,11,183,0,28,67,193};
uint8_t msg1[8] = {1,4,11,208,0,26,114,28};
uint8_t msg2[8] = {1,4,11,234,0,28,210,19};
uint8_t msg0H[8] = {1,4,128,236,0,37,217,228};
uint8_t msg1H[8] = {1,4,129,25,0,45,201,236};
uint8_t msg2H[8] = {1,4,129,109,0,48,73,255};
uint8_t msg3H[8] = {1,4,129,226,0,37,185,219};
uint8_t msggettime[8] = {1,3,11,183,0,6,119,202};
uint8_t msggettimeH[8] = {1,3,167,248,0,6,103,77};
uint8_t msgsettime[21] = {1,16,11,183,0,6,12,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t msgsettimeH[21] = {1,16,167,248,0,6,12,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t msganssettime[21] = {1,16,11,183,0,6,242,9};
uint8_t msganssettimeH[21] = {1,16,167,248,0,6,226,142};
uint8_t msggettype[8] = {1,4,136,184,0,1,154,79};
uint8_t msggettypei[8] = {1,4,11,183,0,1,131,200};
uint8_t msggettypeh[8] = {1,4,128,232,0,1,152,62};
uint8_t msggetepm[8] = {1,4,140,160,0,27,154,179};
uint8_t SN_BUF[20];
uint8_t do_update = 0;
uint8_t SecondByte = 4;
uint8_t FirstRun = 1;
uint8_t DeviceType = 99; // 99 = auto ; 16 = inverter ; 32 = storage inverter
uint8_t DeviceType1 = 0;
uint8_t epm = 9;
uint8_t timeset = 0;
uint8_t CheckTimeH = 0;
uint8_t hour;

uint16_t v1;
uint16_t i1;
uint16_t v2;
uint16_t i2;
uint16_t v3;
uint16_t i3;
uint16_t v4;
uint16_t i4;
uint16_t v;
uint16_t result;
uint16_t counter;
uint16_t memstart = 0;

int16_t zz;

int32_t ww;
int32_t vaactotalDummy1;
int32_t pactotalDummy1;

double pactotalDummy;
double vaactotalDummy;
double vdc1,idc1,pdc1,vdc2,idc2,pdc2,vdc3,idc3,pdc3,vdc4,idc4,pdc4,vacu,iacu,vacv,iacv,vacw,iacw,vaactotal,pactotal,pfac,tinv;
double eday,emonth,eyear,elday,elmonth,elyear,etotal,vbat,ibat,bsoc,bsoh,etotalfrgrid,edayfrgrid;
double eldayfrgrid,etotaltogrid,edaytogrid,eldaytogrid,vbackup,ibackup,pbackup,phouse;
double mvphasea,miphasea,mvphaseb,miphaseb,mvphasec,miphasec,mactpa,mactpb,mactpc,mtotactfrgrid,mtotacttogrid,mpf;
double dummy1;

uint32_t w;

unsigned long lastMsg = 0;
unsigned long lastMsg1 = 0;
unsigned long updatetimer = 0;
unsigned long ErrorTimer;

static timeval tv;
static time_t now;

size_t pwrlostn;
char pwrlostdatatdat[512];


double round2digits(double value) {
   return (unsigned long)(value * 100) / 100.0;
}

double round3digits(double value) {
   return (unsigned long)(value * 1000) / 1000.0;
}

double round2digitsi(double value) {
   return (long)(value * 100) / 100.0;
}

double round3digitsi(double value) {
   return (long)(value * 1000) / 1000.0;
}

void PowerLostCreateData(){
        
    vdc1 = 0;
    idc1 = 0;
    pdc1 = 0;
    vdc2 = 0;
    idc2 = 0;
    pdc2 = 0;
    vacu = 0;
    iacu = 0;
    vacv = 0;
    iacv = 0;
    vacw = 0;
    iacw = 0;
    vaactotal = 0;
    pactotal = 0;
    pfac = 0;
    tinv = 0;

    DynamicJsonDocument doc(2048);
    char datadat[1024];
    
    doc["inverter01_Date"] = inv_date.c_str();
    doc["inverter01_Time"] = inv_time.c_str();
    doc["inverter01_SN"] = inv_sn.c_str();
    doc["inverter01_V_DC1"] = vdc1;
    doc["inverter01_I_DC1"] = idc1;
    doc["inverter01_P_DC1"] = pdc1;
    doc["inverter01_V_DC2"] = vdc2;
    doc["inverter01_I_DC2"] = idc2;
    doc["inverter01_P_DC2"] = pdc2;
    doc["inverter01_V_ACU"] = vacu;
    doc["inverter01_I_ACU"] = iacu;
    doc["inverter01_V_ACV"] = vacv;
    doc["inverter01_I_ACV"] = iacv;
    doc["inverter01_V_ACW"] = vacw;
    doc["inverter01_I_ACW"] = iacw;
    doc["inverter01_P_AC_Out"] = vaactotal;
    doc["inverter01_P_AC_Total"] = pactotal;
    doc["inverter01_P_Fact_AC"] = pfac;
    doc["inverter01_T_Inv"] = tinv;
    

    pwrlostn = serializeJson(doc, pwrlostdatatdat);
}

void errorMsg(String error, bool restart = true) {
  if (restart) {
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

void setupTelnet() {  
  // passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onInputReceived(onTelnetInput);
  telnet.begin(port);
}

// (optional) callback functions for telnet events
void onTelnetConnect(String ip) {
  telnet.print(ansi.cls());
  telnet.print(ansi.home());
  telnet.print(ansi.setFG(ANSI_BRIGHT_WHITE));
  telnet.println("\nWelcome @ Solis to MQTT Requester V090623-2");
  telnet.println("'bye' + ENTER = Exit; 'D' + ENTER = debug on; 'd' + ENTER = debug off");
  telnet.print(ansi.reset());
}

void onTelnetDisconnect(String ip) {
  telnet_debug = 0;
}

void onTelnetReconnect(String ip) {
}

void onTelnetConnectionAttempt(String ip) {
}

void onTelnetInput(String str) {
  if ((str == "Hello") || (str == "hello")) {
    telnet.print(ansi.setFG(ANSI_BRIGHT_WHITE));    
    telnet.println("> I'm NOT Joshua");
    telnet.print(ansi.reset());
  } 
  else if (str == "D"){
    telnet_debug = 1;
  }
  else if (str == "d"){
    telnet_debug = 0;
  }
  else if (str == "bye") {
    telnet.print(ansi.setFG(ANSI_BRIGHT_WHITE));
    telnet.println("> disconnecting you...");
    telnet.print(ansi.reset());
    telnet_debug = 0;
    telnet.disconnectClient();
  }
}


uint16_t CRC16 (const uint8_t *nData, uint16_t wLength)
{
static const uint16_t wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

uint8_t nTemp;
uint16_t wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;

}

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  Serial.begin(9600);

  
  delay(240000);
  while (Serial.available() !=0){
    Serial.read();
  }
  

  pinMode(POWDT, INPUT);    
  configTime(SolisTZ, "de.pool.ntp.org");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
    ip = WiFi.localIP();
  setupTelnet();
  client.setServer(mqttServer, mqttPort);
  client.setKeepAlive(300);
  while (!client.connected()) {
    String clientId = "ESP8266Client-";
    clientId += String(WiFi.macAddress());
    
    if (client.connect(mqttName.c_str(), mqttUser, mqttPassword)) {
      client.setBufferSize(512);
    } else {
      delay(2000);
    }
  }
  
  ArduinoOTA.setHostname(mqttName.c_str());  

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
  });
  ArduinoOTA.onEnd([]() {
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
  });
  ArduinoOTA.begin();
  updatetimer = millis() + (updateinterval*1000);
  //delay(60000);
}

void sendDiscoveryMsgs(){
  Inv_Date();
  delay(10);
  Inv_Time();
  delay(10);
  Inv_SN();
  delay(10);
  V_DC1();
  delay(10);
  I_DC1();
  delay(10);
  P_DC1();
  delay(10);
  V_DC2();
  delay(10);
  I_DC2();
  delay(10);
  P_DC2();
  if (fourmppt == 1){
    V_DC3();
    delay(10);
    I_DC3();
    delay(10);
    P_DC3();
    delay(10);
    V_DC4();
    delay(10);
    I_DC4();
    delay(10);
    P_DC4();
  }
  delay(10);
  V_ACU();
  delay(10);
  I_ACU();
  delay(10);
  V_ACV();
  delay(10);
  I_ACV();
  delay(10);
  V_ACW();
  delay(10);
  I_ACW();
  delay(10);
  P_AC_Out();
  delay(10);
  P_AC_Total();
  delay(10);
  P_Fact_AC();
  delay(10);
  T_Inv();
  delay(10);
  E_Day();
  delay(10);
  E_LDay();
  delay(10);
  E_Month();
  delay(10);
  E_LMonth();
  delay(10);
  E_Year();
  delay(10);
  E_LYear();
  delay(10);
  E_Total();
}

void sendDiscoveryMsgsH(){
  Inv_DateH();
  delay(10);
  Inv_TimeH();
  delay(10);
  Inv_SNH();
  delay(10);
  V_DC1H();
  delay(10);
  I_DC1H();
  delay(10);
  P_DC1H();
  delay(10);
  V_DC2H();
  delay(10);
  I_DC2H();
  delay(10);
  P_DC2H();
  delay(10);
  V_DC3H();
  delay(10);
  I_DC3H();
  delay(10);
  P_DC3H();
  delay(10);
  V_DC4H();
  delay(10);
  I_DC4H();
  delay(10);
  P_DC4H();
  delay(10);
  V_ACUH();
  delay(10);
  I_ACUH();
  delay(10);
  V_ACVH();
  delay(10);
  I_ACVH();
  delay(10);
  V_ACWH();
  delay(10);
  I_ACWH();
  delay(10);
  P_AC_OutH();
  delay(10);
  P_AC_TotalH();
  delay(10);
  P_Fact_ACH();
  delay(10);
  T_InvH();
  delay(10);
  E_DayH();
  delay(10);
  E_LDayH();
  delay(10);
  E_MonthH();
  delay(10);
  E_LMonthH();
  delay(10);
  E_YearH();
  delay(10);
  E_LYearH();
  delay(10);
  E_TotalH();
  delay(10);
  V_BATH();
  delay(10);
  I_BATH();
  delay(10);
  B_ChargeH();
  delay(10);
  B_SOCH();
  delay(10);
  B_SOHH();
  delay(10);
  E_Day_fr_GridH();
  delay(10);
  E_LDay_fr_GridH();
  delay(10);
  E_Total_fr_GridH();
  delay(10);
  E_Day_to_GridH();
  delay(10);
  E_LDay_to_GridH();
  delay(10);
  E_Total_to_GridH();
  delay(10);
  V_BackupH();
  delay(10);
  I_BackupH();
  delay(10);
  P_BackupH();
  delay(10);
  P_HouseH();
  delay(10);
  Meter_V_Phase_AH();
  delay(10);
  Meter_I_Phase_AH();
  delay(10);
  Meter_V_Phase_BH();
  delay(10);
  Meter_I_Phase_BH();
  delay(10);
  Meter_V_Phase_CH();
  delay(10);
  Meter_I_Phase_CH();
  delay(10);
  Meter_act_P_Phase_AH();
  delay(10);
  Meter_act_P_Phase_BH();
  delay(10);
  Meter_act_P_Phase_CH();
  delay(10);
  Meter_tot_E_fr_GridH();
  delay(10);
  Meter_tot_E_to_GridH();
  delay(10);
  Meter_PFH();
}

void inv_time_check() {
  if (Power_check1 == 1){
    PowerLost();
  }
  sensorupdateprogress = 0;
  SecondByte = 3;
  Errorshutcounter = 0;
  delay(req_waiter);
  Serial.write(msggettime,8);
  Serial.flush();
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("Inverter Time request sent");
  }
  while (sensorupdateprogress != 0x0001){
    if(Errorshut == 1){
      if (Errorshutcounter == 5){
        Errorshut = 0;
        Errorshutcounter = 0;
        Decoderselect = 0;
        break;
      }
      delay(req_waiter);
      Serial.write(msggettime,8);
      Serial.flush();
      Errorshut = 0;
      if (telnet_debug == 1){
        telnet.println("Shutcounter: " + String(Errorshutcounter));
      }
      Errorshutcounter++;
      if (telnet_debug == 1){
        telnet.println("Inverter Time request resent");
      }
    }
    ErrorTimer = millis() + Timer_1;
    RcvData();
  }
  if (sensorupdateprogress  == 0x0001){
    sensorupdateprogress = 0;
    Decoderselect = 0;
    if (telnet_debug == 1){
      telnet.println("Inv-Date: Year=" + String(msg_d[4]) + ",Month=" + String(msg_d[6]) + ",Day=" + String(msg_d[8]) + ",Hour=" + String(msg_d[10]) + ",Min=" + String(msg_d[12]) + ",Sec=" + String(msg_d[14]));
    }
    gettimeofday(&tv, nullptr);
    now = time(nullptr);
    struct tm *tm = localtime(&now);
    uint8_t year = tm->tm_year - 100;
    uint8_t mon = tm->tm_mon + 1;
    uint8_t mday = tm->tm_mday;
    uint8_t hour = tm->tm_hour;
    uint8_t min = tm->tm_min;
    uint8_t sec = tm->tm_sec;
    msgsettime[8] = tm->tm_year - 100;
    msgsettime[10] = tm->tm_mon + 1;
    msgsettime[12] = tm->tm_mday;
    msgsettime[14] = tm->tm_hour;
    msgsettime[16] = tm->tm_min;
    msgsettime[18] = tm->tm_sec;
    if (telnet_debug == 1){
      telnet.println("ntp-Date: Year=" + String(year) + ",Month=" + String(mon) + ",Day=" + String(mday) + ",Hour=" + String(hour) + ",Min=" + String(min) + ",Sec=" + String(sec));
    }
    point_to = &msgsettime[0];
    result = CRC16(point_to, 19);
    msgsettime[19] = result;
    msgsettime[20] = result >> 8;
    if (telnet_debug == 1){
     telnet.println("Timesend Data: " + String(msgsettime[0]) + " " + String(msgsettime[1]) + " " + String(msgsettime[2]) + " " + String(msgsettime[3]) + " " + String(msgsettime[4]) + " " + String(msgsettime[5]) + " " + String(msgsettime[6]) + " " + String(msgsettime[7]) + " " + String(msgsettime[8]) + " " + String(msgsettime[9]) + " " + String(msgsettime[10]) + " " + String(msgsettime[11]) + " " + String(msgsettime[12]) + " " + String(msgsettime[13]) + " " + String(msgsettime[14]) + " " + String(msgsettime[15]) + " " + String(msgsettime[16]) + " " + String(msgsettime[17]) + " " + String(msgsettime[18]) + " " + String(msgsettime[19]) + " " + String(msgsettime[20]));
    }
    uint8_t *point_to1 = &msgsettime[7];
    uint8_t *point_to2 = &msg_d[3];
    int diff0 = memcmp(point_to1,point_to2,10);
    point_to1 = &msgsettime[17];
    point_to2 = &msg_d[13];
    int diff1 = memcmp(point_to1,point_to2,2);
    if ((diff0 != 0) || (diff1 > 1) || (diff1 < -1)){
      if (Power_check1 == 1){
        PowerLost();
      }
      SecondByte = 3;
      Errorshutcounter = 0;
      delay(req_waiter);
      Serial.write(msgsettime,21);
      Serial.flush();
      Decoderselect = 0;
      if (telnet_debug == 1){
        telnet.println("Inverter set Time sent");
      }
      while (sensorupdateprogress != 0x0001){
        if(Errorshut == 1){
          if ((msg_d[1] == 16) && (msg_d[2] == 11) && (msg_d[3] == 183) && (msg_d[4] == 0) && (msg_d[5] == 6)){
            timeset = 1;
            Errorshut = 0;
            Errorshutcounter = 0;
            Decoderselect = 0;
            break;
          }
          else {
            timeset = 0;
          }
          if (Errorshutcounter == 5){
            Errorshut = 0;
            Errorshutcounter = 0;
            Decoderselect = 0;
            break;
          }
          delay(req_waiter);
          Serial.write(msgsettime,21);
          Serial.flush();
          Errorshut = 0;
          if (telnet_debug == 1){
            telnet.println("Shutcounter: " + String(Errorshutcounter));
          }
          Errorshutcounter++;
          if (telnet_debug == 1){
            telnet.println("Inverter set Time resent");
          }
        }
        ErrorTimer = millis() + Timer_1;
        RcvData();
      }
      if ((timeset == 1) && (telnet_debug == 1)){
        telnet.println("Time set done");
      }
    }
  }
  else if (sensorupdateprogress == 0x0000){
    sensorupdateprogress = 0;
    Decoderselect = 0;
    if (telnet_debug == 1){
      telnet.println("NO Inverter response");
    }
  }
}

void inv_time_checkH() {
  if (Power_check1 == 1){
    PowerLost();
  }
  sensorupdateprogress = 0;
  SecondByte = 3;
  Errorshutcounter = 0;
  delay(req_waiter);
  Serial.write(msggettimeH,8);
  Serial.flush();
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("InverterH Time request sent");
  }
  while (sensorupdateprogress != 0x0001){
    if(Errorshut == 1){
      if (Errorshutcounter == 5){
        Errorshut = 0;
        Errorshutcounter = 0;
        Decoderselect = 0;
        break;
      }
      delay(req_waiter);
      Serial.write(msggettimeH,8);
      Serial.flush();
      Errorshut = 0;
      if (telnet_debug == 1){
        telnet.println("Shutcounter: " + String(Errorshutcounter));
      }
      Errorshutcounter++;
      if (telnet_debug == 1){
        telnet.println("InverterH Time request resent");
      }
    }
    ErrorTimer = millis() + Timer_1;
    RcvData();
  }
  if (sensorupdateprogress == 0x0001){
    sensorupdateprogress = 0;
    Decoderselect = 0;
    if (telnet_debug == 1){
      telnet.println("Inv-Date: Year=" + String(msg_d[4]) + ",Month=" + String(msg_d[6]) + ",Day=" + String(msg_d[8]) + ",Hour=" + String(msg_d[10]) + ",Min=" + String(msg_d[12]) + ",Sec=" + String(msg_d[14]));
    }
    gettimeofday(&tv, nullptr);
    now = time(nullptr);
    struct tm *tm = localtime(&now);
    uint8_t year = tm->tm_year - 100;
    uint8_t mon = tm->tm_mon + 1;
    uint8_t mday = tm->tm_mday;
    uint8_t hour = tm->tm_hour;
    uint8_t min = tm->tm_min;
    uint8_t sec = tm->tm_sec;
    msgsettimeH[8] = tm->tm_year - 100;
    msgsettimeH[10] = tm->tm_mon + 1;
    msgsettimeH[12] = tm->tm_mday;
    msgsettimeH[14] = tm->tm_hour;
    msgsettimeH[16] = tm->tm_min;
    msgsettimeH[18] = tm->tm_sec;
    if (telnet_debug == 1){
      telnet.println("ntp-Date: Year=" + String(year) + ",Month=" + String(mon) + ",Day=" + String(mday) + ",Hour=" + String(hour) + ",Min=" + String(min) + ",Sec=" + String(sec));
    }
    point_to = &msgsettimeH[0];
    result = CRC16(point_to, 19);
    msgsettime[19] = result;
    msgsettime[20] = result >> 8;
    if (telnet_debug == 1){
      telnet.println("Timesend Data: " + String(msgsettimeH[0]) + " " + String(msgsettimeH[1]) + " " + String(msgsettimeH[2]) + " " + String(msgsettimeH[3]) + " " + String(msgsettimeH[4]) + " " + String(msgsettimeH[5]) + " " + String(msgsettimeH[6]) + " " + String(msgsettimeH[7]) + " " + String(msgsettimeH[8]) + " " + String(msgsettimeH[9]) + " " + String(msgsettimeH[10]) + " " + String(msgsettimeH[11]) + " " + String(msgsettimeH[12]) + " " + String(msgsettimeH[13]) + " " + String(msgsettimeH[14]) + " " + String(msgsettimeH[15]) + " " + String(msgsettimeH[16]) + " " + String(msgsettimeH[17]) + " " + String(msgsettimeH[18]) + " " + String(msgsettimeH[19]) + " " + String(msgsettimeH[20]));
    }
    uint8_t *point_to1 = &msgsettime[7];
    uint8_t *point_to2 = &msg_d[3];
    int diff0 = memcmp(point_to1,point_to2,10);
    point_to1 = &msgsettime[17];
    point_to2 = &msg_d[13];
    int diff1 = memcmp(point_to1,point_to2,2);
    if ((diff0 != 0) || (diff1 > 1) || (diff1 < -1)){
      if (Power_check1 == 1){
        PowerLost();
      }
      SecondByte = 3;
      Errorshutcounter = 0;
      delay(req_waiter);
      Serial.write(msgsettime,21);
      Serial.flush();
      Decoderselect = 0;
      if (telnet_debug == 1){
        telnet.println("InverterH set Time sent");
      }
      while (sensorupdateprogress != 0x0001){
        if(Errorshut == 1){
          if ((msg_d[1] == 16) && (msg_d[2] == 167) && (msg_d[3] == 248) && (msg_d[4] == 0) && (msg_d[5] == 6)){
            timeset = 1;
            Errorshut = 0;
            Errorshutcounter = 0;
            Decoderselect = 0;
            break;
          }
          else {
            timeset = 0;
          }
          if (Errorshutcounter == 5){
            Errorshut = 0;
            Errorshutcounter = 0;
            Decoderselect = 0;
            break;
          }
          delay(req_waiter);
          Serial.write(msgsettime,21);
          Serial.flush();
          Errorshut = 0;
          if (telnet_debug == 1){
            telnet.println("Shutcounter: " + String(Errorshutcounter));
          }
          Errorshutcounter++;
          if (telnet_debug == 1){
            telnet.println("InverterH set Time resent");
          }
        }
        ErrorTimer = millis() + Timer_1;
        RcvData();
      }
      if ((timeset == 1) && (telnet_debug == 1)){
        telnet.println("TimeH set done");
      }
    }
  }
  else if (sensorupdateprogress == 0x0000 ){
    sensorupdateprogress = 0;
    Decoderselect = 0;
    if (telnet_debug == 1){
      telnet.println("NO Inverter response");
    }
  }
}

void inv_time_check1H(){
  gettimeofday(&tv, nullptr);
  now = time(nullptr);
  struct tm *tm = localtime(&now);
  hour = tm->tm_hour;
  if (hour == 23){
    CheckTimeH = 1;
  }
  else if ((hour == 0) && (CheckTimeH == 1)){
    inv_time_checkH();
    CheckTimeH = 0;
  }
}
void inv_type_check() {
  sensorupdateprogress = 0;
  if (Power_check1 == 1){
    PowerLost();
  }
  SecondByte = 4;
  Errorshutcounter = 0;
  delay(req_waiter);
  Serial.write(msggettype,8);
  Serial.flush();
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("Inverter Type request sent");
  }
  while (sensorupdateprogress != 0x0001){
    if(Errorshut == 1){
      if (msg_d[1] == 132){
        Errorshut = 0;
        DeviceType = 55;
        break;
      }
      if (Errorshutcounter == 5){
        Errorshut = 0;
        Errorshutcounter = 0;
        Decoderselect = 0;
        break;
      }
      delay(req_waiter);
      Serial.write(msggettype,8);
      Serial.flush();
      Errorshut = 0;
      if (telnet_debug == 1){
        telnet.println("Shutcounter: " + String(Errorshutcounter));
      }
      Errorshutcounter++;
      if (telnet_debug == 1){
        telnet.println("Inverter Type request resent");
      }
    }
    ErrorTimer = millis() + Timer_1;
    RcvData();
  }
  if (sensorupdateprogress == 0x0001){
    DeviceType = msg_d[3];
    DeviceType1 = msg_d[4];
  }
  
  sensorupdateprogress = 0;
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("Inv-Type = " + String(DeviceType));
  }
}

void inv_type_checki() {
  sensorupdateprogress = 0;
  if (Power_check1 == 1){
    PowerLost();
  }
  SecondByte = 4;
  Errorshutcounter = 0;
  delay(req_waiter);
  Serial.write(msggettypei,8);
  Serial.flush();
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("Inverter Type request sent");
  }
  while (sensorupdateprogress != 0x0001){
    if(Errorshut == 1){
      if (msg_d[1] == 132){
        Errorshut = 0;
        DeviceType = 55;
        break;
      }
      if (Errorshutcounter == 5){
        Errorshut = 0;
        Errorshutcounter = 0;
        Decoderselect = 0;
        break;
      }
      delay(req_waiter);
      Serial.write(msggettypei,8);
      Serial.flush();
      Errorshut = 0;
      if (telnet_debug == 1){
        telnet.println("Shutcounter: " + String(Errorshutcounter));
      }
      Errorshutcounter++;
      if (telnet_debug == 1){
        telnet.println("Inverter Type request resent");
      }
    }
    ErrorTimer = millis() + Timer_1;
    RcvData();
  }
  if (sensorupdateprogress == 0x0001){
    DeviceType = 16;
  }
  
  sensorupdateprogress = 0;
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("Inv-Type = " + String(DeviceType) + " => Inverter");
  }
}

void inv_type_checkh() {
  sensorupdateprogress = 0;
  if (Power_check1 == 1){
    PowerLost();
  }
  SecondByte = 4;
  Errorshutcounter = 0;
  delay(req_waiter);
  Serial.write(msggettypeh,8);
  Serial.flush();
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("Inverter Type request sent");
  }
  while (sensorupdateprogress != 0x0001){
    if(Errorshut == 1){
      if (msg_d[1] == 132){
        Errorshut = 0;
        DeviceType = 55;
        break;
      }
      if (Errorshutcounter == 5){
        Errorshut = 0;
        Errorshutcounter = 0;
        Decoderselect = 0;
        break;
      }
      delay(req_waiter);
      Serial.write(msggettypeh,8);
      Serial.flush();
      Errorshut = 0;
      if (telnet_debug == 1){
        telnet.println("Shutcounter: " + String(Errorshutcounter));
      }
      Errorshutcounter++;
      if (telnet_debug == 1){
        telnet.println("Inverter Type request resent");
      }
    }
    ErrorTimer = millis() + Timer_1;
    RcvData();
  }
  if (sensorupdateprogress == 0x0001){
    DeviceType = 32;
  }
  
  sensorupdateprogress = 0;
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("Inv-Type = " + String(DeviceType) + " => Hybrid");
  }
}

void inv_epm_check() {
  sensorupdateprogress = 0;
  if (Power_check1 == 1){
    PowerLost();
  }
  SecondByte = 4;
  Errorshutcounter = 0;
  delay(req_waiter);
  Serial.write(msggetepm,8);
  Serial.flush();
  Decoderselect = 0;
  if (telnet_debug == 1){
    telnet.println("Inverter epm request sent");
  }
  while (sensorupdateprogress != 0x0001){
    if(Errorshut == 1){
      if (msg_d[1] == 132){
        Errorshut = 0;
        epm = 0;
        break;
      }
      if (Errorshutcounter == 5){
        Errorshut = 0;
        Errorshutcounter = 0;
        Decoderselect = 0;
        break;
      }
     
      delay(req_waiter);
      Serial.write(msggetepm,8);
      Serial.flush();
      Errorshut = 0;
      if (telnet_debug == 1){
        telnet.println("Shutcounter: " + String(Errorshutcounter));
      }
      Errorshutcounter++;
      if (telnet_debug == 1){
        telnet.println("Inverter epm request resent");
      }
    }
    ErrorTimer = millis() + Timer_1;
    RcvData();
  }
  if (sensorupdateprogress == 0x0001){
    sensorupdateprogress = 0;
    Decoderselect = 0;
    epm = 1;
    if (telnet_debug == 1){
      telnet.println("EPM = " + String(epm));
    }
  }
  else if (sensorupdateprogress == 0){
    sensorupdateprogress = 0;
    Decoderselect = 0;
    if (telnet_debug == 1){
      telnet.println("EPM = " + String(epm));
    }
  }
}

void S_E_msgs(){
  Inv_Date();
  delay(100);
  Inv_Time();
  delay(100);
  Inv_SN();
  delay(100);
  switch (DeviceType){
    case  55  : inv_date = "None of the detection";
                inv_sn = "registers avaiable,";
                inv_time = "wrong device ?";
                telnet.println("No device type avaiable, none of the detection registers avaiable, wrong device ?");
                break; 
    case  99  : inv_date = "No response, check";
                inv_sn = "circuit and cables";
                inv_time = "and device address";
                telnet.println("No response from device, check circuit and cables and device address.");
                break; 
    default  :  inv_date = "Unsupported";
                inv_sn = "device type";
                inv_time = " ";
                telnet.println("Unsupported device type.");
                break; 
  }
    
  DynamicJsonDocument doc(2048);
  char datadat[512];
    
  doc["inverter01_Date"] = inv_date.c_str();
  doc["inverter01_Time"] = inv_time.c_str();
  doc["inverter01_SN"] = inv_sn.c_str();
    
  size_t n = serializeJson(doc, datadat);
  client.publish(stateTopic.c_str(), datadat, n);
  if (telnet_debug == 1){
    telnet.println("Error Message sent");
  } 
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    
    String clientId = "ESP8266Client-";
    clientId += String(WiFi.macAddress());
    
    if (client.connect(mqttName.c_str(), mqttUser, mqttPassword)) {
      client.setBufferSize(512);
    }
    else {
      delay(2000);
    }
  }
}

void Inv_Date() {
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_Date/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["name"] = "inverter01_Date";
  doc["val_tpl"] = "{{value_json.inverter01_Date}}";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_Date";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void Inv_Time() {
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_Time/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["name"] = "inverter01_Time";
  doc["val_tpl"] = "{{value_json.inverter01_Time}}";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_Time";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void Inv_SN() {
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_SN/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["name"] = "inverter01_SN";
  doc["val_tpl"] = "{{value_json.inverter01_SN}}";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_SN";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void V_DC1() {
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_V_DC1/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "inverter01_V_DC1";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.inverter01_V_DC1|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_V_DC1";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void I_DC1(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_I_DC1/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "inverter01_I_DC1";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.inverter01_I_DC1|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_I_DC1";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}
   
void P_DC1(){  
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_P_DC1/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_P_DC1";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.inverter01_P_DC1|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_P_DC1";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  
  
void V_DC2(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_V_DC2/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "inverter01_V_DC2";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.inverter01_V_DC2|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_V_DC2";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
  
void I_DC2(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_I_DC2/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "inverter01_I_DC2";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.inverter01_I_DC2|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_I_DC2";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void P_DC2(){  
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_P_DC2/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_P_DC2";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.inverter01_P_DC2|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_P_DC2";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  

void V_DC3() {
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_V_DC3/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "inverter01_V_DC3";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.inverter01_V_DC3|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_V_DC3";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void I_DC3(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_I_DC3/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "inverter01_I_DC3";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.inverter01_I_DC3|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_I_DC3";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}
   
void P_DC3(){  
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_P_DC3/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_P_DC3";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.inverter01_P_DC3|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_P_DC3";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  
  
void V_DC4(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_V_DC4/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "inverter01_V_DC4";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.inverter01_V_DC4|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_V_DC4";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
  
void I_DC4(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_I_DC4/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "inverter01_I_DC4";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.inverter01_I_DC4|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_I_DC4";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void P_DC4(){  
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_P_DC4/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_P_DC4";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.inverter01_P_DC4|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_P_DC4";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  

void V_ACU(){  
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_V_ACU/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "inverter01_V_ACU";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.inverter01_V_ACU|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_V_ACU";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void I_ACU(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_I_ACU/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "inverter01_I_ACU";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.inverter01_I_ACU|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_I_ACU";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
   
void V_ACV(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_V_ACV/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "inverter01_V_ACV";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.inverter01_V_ACV|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_V_ACV";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void I_ACV(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_I_ACV/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "inverter01_I_ACV";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.inverter01_I_ACV|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_I_ACV";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
   
void V_ACW(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_V_ACW/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "inverter01_V_ACW";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.inverter01_V_ACW|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_V_ACW";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void I_ACW(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_I_ACW/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "inverter01_I_ACW";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.inverter01_I_ACW|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_I_ACW";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
   
void P_AC_Out(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_P_AC_Out/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_P_AC_Out";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.inverter01_P_AC_Out|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_P_AC_Out";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void P_AC_Total(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_P_AC_Total/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_P_AC_Total";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.inverter01_P_AC_Total|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_P_AC_Total";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void P_Fact_AC(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_P_Fact_AC/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power_factor";
  doc["name"] = "inverter01_P_Fact_AC";
  doc["val_tpl"] = "{{ value_json.inverter01_P_Fact_AC|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_P_Fact_AC";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
} 
    
void T_Inv(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_T_Inv/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "temperature";
  doc["name"] = "inverter01_T_Inv";
  doc["unit_of_measurement"] = "°C";
  doc["val_tpl"] = "{{ value_json.inverter01_T_Inv|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_T_Inv";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void E_Day(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_E_Day/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_E_Day";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.inverter01_E_Day|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_E_Day";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void E_LDay(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_E_LDay/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_E_LDay";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.inverter01_E_LDay|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_E_LDay";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
     
void E_Month(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_E_Month/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_E_Month";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.inverter01_E_Month|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_E_Month";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
     
void E_LMonth(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_E_LMonth/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_E_LMonth";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.inverter01_E_LMonth|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_E_LMonth";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
     
void E_Year(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_E_Year/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_E_Year";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.inverter01_E_Year|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_E_Year";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
     
void E_LYear(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_E_LYear/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_E_LYear";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.inverter01_E_LYear|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_E_LYear";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  

void E_Total(){
  String discoveryTopic = "homeassistant/sensor/inverter01/inverter01_E_Total/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "inverter01_E_Total";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.inverter01_E_Total|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopic;
  doc["unique_id"] = "inverter01_E_Total";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "inverter01";
  device["name"] = "inverter01";   
  device["model"] = "inverter01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  

void Inv_DateH() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Date/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["name"] = "hybrid01_Date";
  doc["val_tpl"] = "{{value_json.hybrid01_Date}}";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Date";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void Inv_TimeH() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Time/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["name"] = "hybrid01_Time";
  doc["val_tpl"] = "{{value_json.hybrid01_Time}}";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Time";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void Inv_SNH() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_SN/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["name"] = "hybrid01_SN";
  doc["val_tpl"] = "{{value_json.hybrid01_SN}}";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_SN";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void V_DC1H() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_DC1/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_DC1";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_DC1|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_DC1";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void I_DC1H(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_DC1/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_DC1";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_DC1|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_DC1";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}
   
void P_DC1H(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_DC1/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_P_DC1";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_DC1|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_DC1";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  
  
void V_DC2H(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_DC2/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_DC2";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_DC2|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_DC2";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
  
void I_DC2H(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_DC2/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_DC2";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_DC2|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_DC2";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void P_DC2H(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_DC2/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_P_DC2";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_DC2|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_DC2";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  

void V_DC3H() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_DC3/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_DC3";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_DC3|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_DC3";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void I_DC3H(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_DC3/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_DC3";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_DC3|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_DC3";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}
   
void P_DC3H(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_DC3/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_P_DC3";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_DC3|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_DC3";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  
  
void V_DC4H(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_DC4/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_DC4";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_DC4|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_DC4";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
  
void I_DC4H(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_DC4/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_DC4";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_DC4|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_DC4";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void P_DC4H(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_DC4/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_P_DC4";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_DC4|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_DC4";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
} 

void V_ACUH(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_ACU/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_ACU";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_ACU|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_ACU";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void I_ACUH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_ACU/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_ACU";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_ACU|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_ACU";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void V_ACVH(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_ACV/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_ACV";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_ACV|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_ACV";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void I_ACVH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_ACV/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_ACV";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_ACV|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_ACV";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void V_ACWH(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_ACW/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_ACW";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_ACW|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_ACW";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void I_ACWH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_ACW/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_ACW";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_ACW|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_ACW";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
   
void P_AC_OutH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_AC_Out/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_P_AC_Out";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_AC_Out|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_AC_Out";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void P_AC_TotalH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_AC_Total/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_P_AC_Total";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_AC_Total|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_AC_Total";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void P_Fact_ACH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_Fact_AC/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power_factor";
  doc["name"] = "hybrid01_P_Fact_AC";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_Fact_AC|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_Fact_AC";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
} 
    
void T_InvH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_T_Inv/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "temperature";
  doc["name"] = "hybrid01_T_Inv";
  doc["unit_of_measurement"] = "°C";
  doc["val_tpl"] = "{{ value_json.hybrid01_T_Inv|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_T_Inv";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void E_DayH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_Day/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_Day";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_Day|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_Day";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void E_LDayH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_LDay/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_LDay";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_LDay|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_LDay";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
     
void E_MonthH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_Month/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_Month";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_Month|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_Month";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
     
void E_LMonthH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_LMonth/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_LMonth";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_LMonth|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_LMonth";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
     
void E_YearH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_Year/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_Year";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_Year|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_Year";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
     
void E_LYearH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_LYear/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_LYear";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_LYear|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_LYear";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  

void E_TotalH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_Total/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_Total";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_Total|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_Total";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void V_BATH() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_BAT/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_BAT";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_BAT|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_BAT";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void I_BATH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_BAT/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_BAT";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_BAT|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_BAT";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}

void B_ChargeH() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_B_Charge/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["name"] = "hybrid01_B_Charge";
  doc["val_tpl"] = "{{value_json.hybrid01_B_Charge}}";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_B_Charge";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void B_SOCH() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_B_SOC/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "battery";
  doc["name"] = "hybrid01_B_SOC";
  doc["unit_of_measurement"] = "%";
  doc["val_tpl"] = "{{ value_json.hybrid01_B_SOC|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_B_SOC";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void B_SOHH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_B_SOH/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "battery";
  doc["name"] = "hybrid01_B_SOH";
  doc["unit_of_measurement"] = "%";
  doc["val_tpl"] = "{{ value_json.hybrid01_B_SOH|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_B_SOH";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}

void E_Day_fr_GridH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_Day_fr_Grid/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_Day_fr_Grid";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_Day_fr_Grid|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_Day_fr_Grid";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void E_LDay_fr_GridH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_LDay_fr_Grid/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_LDay_fr_Grid";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_LDay_fr_Grid|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_LDay_fr_Grid";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void E_Total_fr_GridH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_Total_fr_Grid/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_Total_fr_Grid";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_Total_fr_Grid|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_Total_fr_Grid";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void E_Day_to_GridH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_Day_to_Grid/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_Day_to_Grid";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_Day_to_Grid|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_Day_to_Grid";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void E_LDay_to_GridH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_LDay_to_Grid/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_LDay_to_Grid";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_LDay_to_Grid|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_LDay_to_Grid";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void E_Total_to_GridH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_E_Total_to_Grid/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_E_Total_to_Grid";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_E_Total_to_Grid|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_E_Total_to_Grid";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void V_BackupH() {
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_V_Backup/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_V_Backup";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_V_Backup|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_V_Backup";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}
  
void I_BackupH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_I_Backup/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_I_Backup";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_I_Backup|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_I_Backup";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}
   
void P_BackupH(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_Backup/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_P_Backup";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_Backup|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_Backup";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  

void P_HouseH(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_P_House/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_P_House";
  doc["unit_of_measurement"] = "W";
  doc["val_tpl"] = "{{ value_json.hybrid01_P_House|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_P_House";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n); 
}  

void Meter_V_Phase_AH(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_V_Phase_A/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_Meter_V_Phase_A";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_V_Phase_A|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_V_Phase_A";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void Meter_I_Phase_AH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_I_Phase_A/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_Meter_I_Phase_A";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_I_Phase_A|round(2)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_I_Phase_A";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void Meter_V_Phase_BH(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_V_Phase_B/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_Meter_V_Phase_B";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_V_Phase_B|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_V_Phase_B";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void Meter_I_Phase_BH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_I_Phase_B/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_Meter_I_Phase_B";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_I_Phase_B|round(2)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_I_Phase_B";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void Meter_V_Phase_CH(){  
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_V_Phase_C/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "voltage";
  doc["name"] = "hybrid01_Meter_V_Phase_C";
  doc["unit_of_measurement"] = "V";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_V_Phase_C|round(1)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_V_Phase_C";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}  
   
void Meter_I_Phase_CH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_I_Phase_C/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "current";
  doc["name"] = "hybrid01_Meter_I_Phase_C";
  doc["unit_of_measurement"] = "A";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_I_Phase_C|round(2)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_I_Phase_C";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void Meter_act_P_Phase_AH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_act_P_Phase_A/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_Meter_act_P_Phase_A";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_act_P_Phase_A|round(3)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_act_P_Phase_A";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void Meter_act_P_Phase_BH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_act_P_Phase_B/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_Meter_act_P_Phase_B";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_act_P_Phase_B|round(3)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_act_P_Phase_B";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void Meter_act_P_Phase_CH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_act_P_Phase_C/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_Meter_act_P_Phase_C";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_act_P_Phase_C|round(3)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_act_P_Phase_C";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void Meter_tot_E_fr_GridH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_tot_E_fr_Grid/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_Meter_tot_E_fr_Grid";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_tot_E_fr_Grid|round(2)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_tot_E_fr_Grid";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}


void Meter_tot_E_to_GridH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_tot_E_to_Grid/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power";
  doc["name"] = "hybrid01_Meter_tot_E_to_Grid";
  doc["unit_of_measurement"] = "kW";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_tot_E_to_Grid|round(2)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_tot_E_to_Grid";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
}

void Meter_PFH(){
  String discoveryTopic = "homeassistant/sensor/hybrid01/hybrid01_Meter_PF/config";
  char datadat[512];
  DynamicJsonDocument doc(1024);
  doc["device_class"] = "power_factor";
  doc["name"] = "hybrid01_Meter_PF";
  doc["val_tpl"] = "{{ value_json.hybrid01_Meter_PF|round(2)|default(0) }}";
  doc["state_class"] = "measurement";
  doc["state_topic"] = stateTopicH;
  doc["unique_id"] = "hybrid01_Meter_PF";
  JsonObject device = doc.createNestedObject("device");
  device["identifiers"][0] = "hybrid01";
  device["name"] = "hybrid01";   
  device["model"] = "hybrid01";
  device["manufacturer"] = "Solis";
  size_t n = serializeJson(doc, datadat);
  client.publish(discoveryTopic.c_str(), datadat, n);
} 

void SendMsgs1(){
    DynamicJsonDocument doc(2048);
    char datadat[512];
    doc["inverter01_Date"] = inv_date.c_str();
    doc["inverter01_Time"] = inv_time.c_str();
    doc["inverter01_SN"] = inv_sn.c_str();
    doc["inverter01_V_DC1"] = vdc1;
    doc["inverter01_I_DC1"] = idc1;
    doc["inverter01_P_DC1"] = pdc1;
    doc["inverter01_V_DC2"] = vdc2;
    doc["inverter01_I_DC2"] = idc2;
    doc["inverter01_P_DC2"] = pdc2;
    doc["inverter01_V_ACU"] = vacu;
    doc["inverter01_I_ACU"] = iacu;
    doc["inverter01_V_ACV"] = vacv;
    doc["inverter01_I_ACV"] = iacv;
    doc["inverter01_V_ACW"] = vacw;
    doc["inverter01_I_ACW"] = iacw;
        
    size_t n = serializeJson(doc, datadat);
    client.publish(stateTopic.c_str(), datadat, n);
    if (telnet_debug == 1){
      telnet.println("Messages Part 1 sent");
    }
}

void SendMsgs2(){
    DynamicJsonDocument doc(2048);
    char datadat[512];
    
    doc["inverter01_Date"] = inv_date.c_str();
    doc["inverter01_Time"] = inv_time.c_str();
    doc["inverter01_SN"] = inv_sn.c_str();
    doc["inverter01_P_AC_Out"] = vaactotal;
    doc["inverter01_P_AC_Total"] = pactotal;
    doc["inverter01_P_Fact_AC"] = pfac;
    doc["inverter01_T_Inv"] = tinv;
    doc["inverter01_E_Day"] = eday;
    doc["inverter01_E_LDay"] = elday;
    doc["inverter01_E_Month"] = emonth;
    doc["inverter01_E_LMonth"] = elmonth;
    doc["inverter01_E_Year"] = eyear;
    doc["inverter01_E_LYear"] = elyear;
    doc["inverter01_E_Total"] = etotal;

    size_t n = serializeJson(doc, datadat);
    client.publish(stateTopic.c_str(), datadat, n);
    if (telnet_debug == 1){
      telnet.println("Messages Part 2 sent");
    } 
}

void SendMsgs3(){
    DynamicJsonDocument doc(2048);
    char datadat[512];
    doc["inverter01_Date"] = inv_date.c_str();
    doc["inverter01_Time"] = inv_time.c_str();
    doc["inverter01_SN"] = inv_sn.c_str();
    doc["inverter01_V_DC3"] = vdc3;
    doc["inverter01_I_DC3"] = idc3;
    doc["inverter01_P_DC3"] = pdc3;
    doc["inverter01_V_DC4"] = vdc4;
    doc["inverter01_I_DC4"] = idc4;
    doc["inverter01_P_DC4"] = pdc4;
        
    size_t n = serializeJson(doc, datadat);
    client.publish(stateTopic.c_str(), datadat, n);
    if (telnet_debug == 1){
      telnet.println("Messages Part 3 sent");
    }
}

void SendMsgs1H(){
    DynamicJsonDocument doc(2048);
    char datadat[512];
    doc["hybrid01_Date"] = inv_date.c_str();
    doc["hybrid01_Time"] = inv_time.c_str();
    doc["hybrid01_SN"] = inv_sn.c_str();
    doc["hybrid01_B_Charge"] = charge.c_str();
    doc["hybrid01_V_DC1"] = vdc1;
    doc["hybrid01_I_DC1"] = idc1;
    doc["hybrid01_P_DC1"] = pdc1;
    doc["hybrid01_V_DC2"] = vdc2;
    doc["hybrid01_I_DC2"] = idc2;
    doc["hybrid01_P_DC2"] = pdc2;
    doc["hybrid01_V_DC3"] = vdc3;
    doc["hybrid01_I_DC3"] = idc3;
    doc["hybrid01_P_DC3"] = pdc3;
    doc["hybrid01_V_DC4"] = vdc4;
    doc["hybrid01_I_DC4"] = idc4;
    doc["hybrid01_P_DC4"] = pdc4;
    doc["hybrid01_V_ACU"] = vacu;
    doc["hybrid01_I_ACU"] = iacu;
            
    size_t n = serializeJson(doc, datadat);
    client.publish(stateTopicH.c_str(), datadat, n);
    if (telnet_debug == 1){
      telnet.println("Messages Part 1H sent");
    }
}

void SendMsgs2H(){
    DynamicJsonDocument doc(2048);
    char datadat[512];
    
    doc["hybrid01_Date"] = inv_date.c_str();
    doc["hybrid01_Time"] = inv_time.c_str();
    doc["hybrid01_SN"] = inv_sn.c_str();
    doc["hybrid01_B_Charge"] = charge.c_str();
    doc["hybrid01_V_ACV"] = vacv;
    doc["hybrid01_I_ACV"] = iacv;
    doc["hybrid01_V_ACW"] = vacw;
    doc["hybrid01_I_ACW"] = iacw;
    doc["hybrid01_V_BAT"] = vbat;
    doc["hybrid01_I_BAT"] = ibat;
    doc["hybrid01_B_SOC"] = bsoc;
    doc["hybrid01_B_SOH"] = bsoh;
    doc["hybrid01_P_AC_Out"] = vaactotal;
    doc["hybrid01_P_AC_Total"] = pactotal;
    doc["hybrid01_P_Fact_AC"] = pfac;
    doc["hybrid01_T_Inv"] = tinv;
    
        
    size_t n = serializeJson(doc, datadat);
    client.publish(stateTopicH.c_str(), datadat, n);
    if (telnet_debug == 1){
      telnet.println("Messages Part 2H sent");
    } 
}

void SendMsgs3H(){
    DynamicJsonDocument doc(2048);
    char datadat[512];
    
    doc["hybrid01_Date"] = inv_date.c_str();
    doc["hybrid01_Time"] = inv_time.c_str();
    doc["hybrid01_SN"] = inv_sn.c_str();
    doc["hybrid01_B_Charge"] = charge.c_str();
    doc["hybrid01_E_Day"] = eday;
    doc["hybrid01_E_LDay"] = elday;
    doc["hybrid01_E_Month"] = emonth;
    doc["hybrid01_E_LMonth"] = elmonth;
    doc["hybrid01_E_Year"] = eyear;
    doc["hybrid01_E_LYear"] = elyear;
    doc["hybrid01_E_Total"] = etotal;
    doc["hybrid01_E_Day_fr_Grid"] = edayfrgrid;
    doc["hybrid01_E_LDay_fr_Grid"] = eldayfrgrid;
    doc["hybrid01_E_Day_to_Grid"] = edaytogrid;
    doc["hybrid01_E_LDay_to_Grid"] = eldaytogrid;
    
        
    size_t n = serializeJson(doc, datadat);
    client.publish(stateTopicH.c_str(), datadat, n);
    if (telnet_debug == 1){
      telnet.println("Messages Part 3H sent");
    } 
}

void SendMsgs4H(){
    DynamicJsonDocument doc(2048);
    char datadat[512];
    
    doc["hybrid01_Date"] = inv_date.c_str();
    doc["hybrid01_Time"] = inv_time.c_str();
    doc["hybrid01_SN"] = inv_sn.c_str();
    doc["hybrid01_B_Charge"] = charge.c_str();
    doc["hybrid01_V_Backup"] = vbackup;
    doc["hybrid01_I_Backup"] = ibackup;
    doc["hybrid01_P_Backup"] = pbackup;
    doc["hybrid01_P_House"] = phouse;
    doc["hybrid01_B_Charge"] = charge.c_str();
    doc["hybrid01_Meter_V_Phase_A"] = mvphasea;
    doc["hybrid01_Meter_I_Phase_A"] = miphasea;
    doc["hybrid01_Meter_V_Phase_B"] = mvphaseb;
    doc["hybrid01_Meter_I_Phase_B"] = miphaseb;
    doc["hybrid01_Meter_V_Phase_C"] = mvphasec;
    doc["hybrid01_Meter_I_Phase_C"] = miphasec;
        
    size_t n = serializeJson(doc, datadat);
    client.publish(stateTopicH.c_str(), datadat, n);
    if (telnet_debug == 1){
      telnet.println("Messages Part 4H sent");
    } 
}

void SendMsgs5H(){
    DynamicJsonDocument doc(2048);
    char datadat[512];
    
    doc["hybrid01_Date"] = inv_date.c_str();
    doc["hybrid01_Time"] = inv_time.c_str();
    doc["hybrid01_SN"] = inv_sn.c_str();
    doc["hybrid01_B_Charge"] = charge.c_str();
    doc["hybrid01_Meter_act_P_Phase_A"] = mactpa;
    doc["hybrid01_Meter_act_P_Phase_B"] = mactpb;
    doc["hybrid01_Meter_act_P_Phase_C"] = mactpc;
    doc["hybrid01_Meter_tot_E_fr_Grid"] = mtotactfrgrid;
    doc["hybrid01_Meter_tot_E_to_Grid"] = mtotacttogrid;
    doc["hybrid01_E_Total_fr_Grid"] = etotalfrgrid;
    doc["hybrid01_E_Total_to_Grid"] = etotaltogrid;
    doc["hybrid01_Meter_PF"] = mpf;
    
    
    size_t n = serializeJson(doc, datadat);
    client.publish(stateTopicH.c_str(), datadat, n);
    if (telnet_debug == 1){
      telnet.println("Messages Part 5H sent");
    } 
}

void PowerLost(){

  if (digitalRead(POWDT)== 0){
    telnet.println("PowerLost GPIO16");
    client.publish(stateTopic.c_str(), pwrlostdatatdat, pwrlostn);
    delay(10000);
  }
}

void decoder(){
  
  w = (msg_d[13] <<24 | msg_d[14] <<16 | msg_d[15] <<8 | msg_d[16]);
  pactotalDummy = w;
  pactotal = w;
        
  w = (msg_d[21] <<24 | msg_d[22] <<16 |  msg_d[23] <<8 | msg_d[24]);
  etotal = w;
        
  w = (msg_d[25] <<24 | msg_d[26] <<16 |  msg_d[27] <<8 | msg_d[28]);
  emonth = w;

  w = (msg_d[29] <<24 | msg_d[30] <<16 |  msg_d[31] <<8 | msg_d[32]);
  elmonth = w;
        
  v = (msg_d[33] <<8 | msg_d[34]);
  dummy1 = v*0.1f;
  eday = round2digits(dummy1);

  v = (msg_d[35] <<8 | msg_d[36]);
  dummy1 = v*0.1f;
  elday = round2digits(dummy1);
  

  w = (msg_d[37] <<24 | msg_d[38] <<16 |  msg_d[39] <<8 | msg_d[40]);
  eyear = w;

  w = (msg_d[41] <<24 | msg_d[42] <<16 |  msg_d[43] <<8 | msg_d[44]);
  elyear = w;
        
  v = (msg_d[47] <<8 | msg_d[48]);
  v1 = v;
  dummy1 = v*0.1f;
  vdc1 = round2digits(dummy1);
          
  v = (msg_d[49] <<8 | msg_d[50]);
  i1 = v;
  dummy1 = v*0.1f;
  idc1 = round2digits(dummy1);
          
  v = (msg_d[51] <<8 | msg_d[52]);
  v2 = v;
  dummy1 = v*0.1f;
  vdc2 = round2digits(dummy1);
        
  v = (msg_d[53] <<8 | msg_d[54]);
  i2 = v;
  dummy1 = v*0.1f;
  idc2 = round2digits(dummy1);
        
  dummy1 = v1 * i1 * 0.01f;
  pdc1 = round2digits(dummy1);
        
  dummy1 = v2 * i2 * 0.01f;
  pdc2 = round2digits(dummy1);
    
  v = (msg_d[105] <<8 | msg_d[106]);
  v3 = v;
  dummy1 = v*0.1f;
  vdc3 = round2digits(dummy1);
          
  v = (msg_d[107] <<8 | msg_d[108]);
  i3 = v;
  dummy1 = v*0.1f;
  idc3 = round2digits(dummy1);
          
  v = (msg_d[109] <<8 | msg_d[110]);
  v4 = v;
  dummy1 = v*0.1f;
  vdc4 = round2digits(dummy1);
        
  v = (msg_d[111] <<8 | msg_d[112]);
  i4 = v;
  dummy1 = v*0.1f;
  idc4 = round2digits(dummy1);
        
  dummy1 = v3 * i3 * 0.01f;
  pdc3 = round2digits(dummy1);
        
  dummy1 = v4 * i4 * 0.01f;
  pdc4 = round2digits(dummy1);
  
  v = (msg_d[121] <<8 | msg_d[122]);
  dummy1 = v * 0.1f;
  vacu = round2digits(dummy1);
        
  v = (msg_d[123] <<8 | msg_d[124]);
  dummy1 = v * 0.1f;
  vacv = round2digits(dummy1);
      
  v = (msg_d[125] <<8 | msg_d[126]);
  dummy1 = v * 0.1f;
  vacw = round2digits(dummy1);
          
  v = (msg_d[127] <<8 | msg_d[128]);
  dummy1 = v * 0.1f;
  iacu = round2digits(dummy1);
        
  v = (msg_d[129] <<8 | msg_d[130]);
  dummy1 = v * 0.1f;
  iacv = round2digits(dummy1);
        
  v = (msg_d[131] <<8 | msg_d[132]);
  dummy1 = v * 0.1f;
  iacw = round2digits(dummy1);
        
  v = (msg_d[137] <<8 | msg_d[138]);
  dummy1 = v * 0.1f;
  tinv = round3digits(dummy1);
    
  ww = (msg_d[217] <<24 | msg_d[218] <<16 | msg_d[219] <<8 |  msg_d[220]);
  vaactotalDummy = ww;
  vaactotal = ww;
    
  SN_BUF[0] = msg_d[223] & 0x0f;
  SN_BUF[1] = (msg_d[223] >> 4) & 0x0f;
  SN_BUF[2] = msg_d[224] & 0x0f;
  SN_BUF[3] = (msg_d[224] >> 4) & 0x0f;
  SN_BUF[4] = msg_d[225] & 0x0f;
  SN_BUF[5] = (msg_d[225] >> 4) & 0x0f;
  SN_BUF[6] = msg_d[226] & 0x0f;
  SN_BUF[7] = (msg_d[226] >> 4) & 0x0f;
  SN_BUF[8] = msg_d[227] & 0x0f;
  SN_BUF[9] = (msg_d[227] >> 4) & 0x0f;
  SN_BUF[10] = msg_d[228] & 0x0f;
  SN_BUF[11] = (msg_d[228] >> 4) & 0x0f;
  SN_BUF[12] = msg_d[229] & 0x0f;
  SN_BUF[13] = (msg_d[229] >> 4) & 0x0f;
  SN_BUF[14] = msg_d[230] & 0x0f;
  SN_BUF[15] = (msg_d[230] >> 4) & 0x0f;
  if ((msg_d[252] < 10) && (msg_d[250] < 10)){
    inv_date =  "0" + String(msg_d[252], DEC) + "-" + "0" + String(msg_d[250], DEC) + "-20" + String(msg_d[248], DEC);
  }
  else if ((msg_d[252] < 10) && (msg_d[250] >= 10)){
    inv_date = "0" + String(msg_d[252], DEC) + "-" + String(msg_d[250], DEC) + "-20" + String(msg_d[248], DEC);
  }
  else if ((msg_d[252] >= 10) && (msg_d[250] < 10)){
    inv_date = String(msg_d[252], DEC) + "-" + "0"  + String(msg_d[250], DEC) + "-20" + String(msg_d[248], DEC);
  }
  else {
    inv_date = String(msg_d[252], DEC) + "-" + String(msg_d[250], DEC) + "-20" + String(msg_d[248], DEC);
  }
    
  if (msg_d[254] < 10){
    HOUR = "0" + String(msg_d[254], DEC);
  }
  else{
    HOUR = String(msg_d[254], DEC);
  }
  if (msg_d[256] < 10){
    MINUTE = "0" + String(msg_d[256], DEC);
  }
  else{
    MINUTE = String(msg_d[256], DEC);
  }
  if (msg_d[258] < 10){
    SECOND = "0" + String(msg_d[258], DEC);
  }
  else {
    SECOND = String(msg_d[258], DEC);
  }

  inv_time = HOUR + ":" + MINUTE + ":" + SECOND;
    
  inv_sn = String(SN_BUF[2], HEX) + String(SN_BUF[3], HEX) + String(SN_BUF[0], HEX) + String(SN_BUF[1], HEX) + String(SN_BUF[6], HEX) + String(SN_BUF[7], HEX) + String(SN_BUF[4], HEX) + String(SN_BUF[5], HEX) + String(SN_BUF[10], HEX) + String(SN_BUF[11], HEX) + String(SN_BUF[8], HEX) + String(SN_BUF[9], HEX) + String(SN_BUF[14], HEX) + String(SN_BUF[15], HEX) + String(SN_BUF[12], HEX) + String(SN_BUF[13], HEX);
    
  sensorupdateprogress = 0;
  Decoderselect = 0;
        
  if (double(vaactotalDummy / pactotalDummy) > 1.0){
    double dummy = 1.0;
    pfac = dummy;
  }
  else if (double(vaactotalDummy / pactotalDummy) <= 1.0) {
    dummy1 = double(vaactotalDummy / pactotalDummy);
    pfac = round2digits(dummy1);
  }
  
  if (telnet_debug == 1){
    telnet.println("Messages decoded");
  }  
}

void decoderH(){
  
  w = (msg_d[53] <<24 | msg_d[54] <<16 |  msg_d[55] <<8 | msg_d[56]);
  etotal = w;
        
  w = (msg_d[57] <<24 | msg_d[58] <<16 |  msg_d[59] <<8 | msg_d[60]);
  emonth = w;

  w = (msg_d[61] <<24 | msg_d[62] <<16 |  msg_d[63] <<8 | msg_d[64]);
  elmonth = w;
        
  v = (msg_d[65] <<8 | msg_d[66]);
  dummy1 = v*0.1f;
  eday = round2digits(dummy1);

  v = (msg_d[67] <<8 | msg_d[68]);
  dummy1 = v*0.1f;
  elday = round3digits(dummy1);
  
  w = (msg_d[69] <<24 | msg_d[70] <<16 |  msg_d[71] <<8 | msg_d[72]);
  eyear = w;

  w = (msg_d[73] <<24 | msg_d[74] <<16 |  msg_d[75] <<8 | msg_d[76]);
  elyear = w;
        
  v = (msg_d[103] <<8 | msg_d[104]);
  v1 = v;
  dummy1 = v*0.1f;
  vdc1 = round2digits(dummy1);
          
  v = (msg_d[105] <<8 | msg_d[106]);
  i1 = v;
  dummy1 = v*0.1f;
  idc1 = round2digits(dummy1);
          
  v = (msg_d[107] <<8 | msg_d[108]);
  v2 = v;
  dummy1 = v*0.1f;
  vdc2 = round2digits(dummy1);
        
  v = (msg_d[109] <<8 | msg_d[110]);
  i2 = v;
  dummy1 = v*0.1f;
  idc2 = round2digits(dummy1);
        
  dummy1 = v1 * i1 * 0.01f;
  pdc1 = round2digits(pdc1);
        
  dummy1 = v2 * i2 * 0.01f;
  pdc2 = round2digits(dummy1);
    
  v = (msg_d[111] <<8 | msg_d[112]);
  v3 = v;
  dummy1 = v*0.1f;
  vdc3 = round2digits(dummy1);
          
  v = (msg_d[113] <<8 | msg_d[114]);
  i3 = v;
  dummy1 = v*0.1f;
  idc3 = round2digits(dummy1);
          
  v = (msg_d[115] <<8 | msg_d[116]);
  v4 = v;
  dummy1 = v*0.1f;
  vdc4 = round2digits(dummy1);
        
  v = (msg_d[117] <<8 | msg_d[118]);
  i4 = v;
  dummy1 = v*0.1f;
  idc4 = round2digits(dummy1);
        
  dummy1 = v3 * i3 * 0.01f;
  pdc3 = round2digits(pdc1);
        
  dummy1 = v4 * i4 * 0.01f;
  pdc4 = round2digits(dummy1);
  
  v = (msg_d[151] <<8 | msg_d[152]);
  dummy1 = v * 0.1f;
  vacu = round2digits(dummy1);

  v = (msg_d[153] <<8 | msg_d[154]);
  dummy1 = v * 0.1f;
  vacv = round2digits(dummy1);

  v = (msg_d[155] <<8 | msg_d[156]);
  dummy1 = v * 0.1f;
  vacw = round2digits(dummy1);
              
  v = (msg_d[157] <<8 | msg_d[158]);
  dummy1 = v * 0.1f;
  iacu = round2digits(dummy1);

  v = (msg_d[159] <<8 | msg_d[160]);
  dummy1 = v * 0.1f;
  iacv = round2digits(dummy1);

  v = (msg_d[161] <<8 | msg_d[162]);
  dummy1 = v * 0.1f;
  iacw = round2digits(dummy1);
    
  ww = (msg_d[163] <<24 | msg_d[164] <<16 | msg_d[165] <<8 | msg_d[166]);
  pactotalDummy = ww;
  pactotal = ww;
     
  ww = (msg_d[171] <<24 | msg_d[172] <<16 | msg_d[173] <<8 |  msg_d[174]);
  vaactotalDummy = ww;
  vaactotal = ww;

  zz = (msg_d[191] <<8 | msg_d[192]);
  dummy1 = zz;
  dummy1 = dummy1 * 0.1f;
  tinv = round2digits(dummy1);

  v = (msg_d[203] <<8 | msg_d[204]);
  dummy1 = v*0.1f;
  vbat = round2digits(dummy1);
          
  zz = (msg_d[205] <<8 | msg_d[206]);
  dummy1 = zz;
  dummy1 = dummy1 * 0.1f;
  ibat = round2digitsi(dummy1);
  
  switch (msg_d[208]){
    case  0 : charge = "charge";
              break;
    case  1 : charge = "discharge";
              if(ibat > 0){
                ibat = ibat * (-1);
              }
              break;
  }
  
  v = (msg_d[211] <<8 | msg_d[212]);
  dummy1 = v*0.1f;
  vbackup = round2digits(dummy1);
          
  v = (msg_d[213] <<8 | msg_d[214]);
  dummy1 = v*0.1f;
  ibackup = round2digits(dummy1);

  bsoc = msg_d[216];

  bsoh = msg_d[218];
  
  v = (msg_d[231] <<8 | msg_d[232]);
  phouse = v;
  
  v = (msg_d[233] <<8 | msg_d[234]);
  pbackup = v;

  w = (msg_d[275] <<24 | msg_d[276] <<16 |  msg_d[277] <<8 | msg_d[278]);
  etotalfrgrid = w;
  
  v = (msg_d[279] <<8 | msg_d[280]);
  dummy1 = v*0.1f;
  edayfrgrid = round2digits(dummy1);

  v = (msg_d[281] <<8 | msg_d[282]);
  dummy1 = v*0.1f;
  eldayfrgrid = round2digits(dummy1);

  w = (msg_d[283] <<24 | msg_d[284] <<16 |  msg_d[285] <<8 | msg_d[286]);
  etotaltogrid = w;
  
  v = (msg_d[287] <<8 | msg_d[288]);
  dummy1 = v*0.1f;
  edaytogrid = round2digits(dummy1);

  v = (msg_d[289] <<8 | msg_d[290]);
  dummy1 = v*0.1f;
  eldaytogrid = round2digits(dummy1);

  v = (msg_d[505] <<8 | msg_d[506]);
  dummy1 = v * 0.1f;
  mvphasea = round2digits(dummy1);
        
  v = (msg_d[507] <<8 | msg_d[508]);
  dummy1 = v * 0.01f;
  miphasea = round2digits(dummy1);

  v = (msg_d[509] <<8 | msg_d[510]);
  dummy1 = v * 0.1f;
  mvphaseb = round2digits(dummy1);
        
  v = (msg_d[511] <<8 | msg_d[512]);
  dummy1 = v * 0.01f;
  miphaseb = round2digits(dummy1);

  v = (msg_d[513] <<8 | msg_d[514]);
  dummy1 = v * 0.1f;
  mvphasec = round2digits(dummy1);
        
  v = (msg_d[515] <<8 | msg_d[516]);
  dummy1 = v * 0.01f;
  miphasec = round2digits(dummy1);

  ww = (msg_d[517] <<24 | msg_d[518] <<16 | msg_d[519] <<8 | msg_d[520]);
  dummy1 = ww;
  dummy1 = dummy1  * 0.001f;
  mactpa = round3digitsi(dummy1);
  
  ww = (msg_d[521] <<24 | msg_d[522] <<16 | msg_d[523] <<8 | msg_d[524]);
  dummy1 = ww;
  dummy1 = dummy1  * 0.001f;
  mactpb = round3digitsi(dummy1);

  ww = (msg_d[525] <<24 | msg_d[526] <<16 | msg_d[527] <<8 | msg_d[528]);
  dummy1 = ww;
  dummy1 = dummy1  * 0.001f;
  mactpc = round3digitsi(dummy1);

  zz = (msg_d[565] <<8 | msg_d[566]);
  dummy1 = zz;
  dummy1 = dummy1 * 0.01f;
  mpf = round2digitsi(dummy1);
  if(mpf < 0){
    mpf = mpf * (-1);
  }
  
  w = (msg_d[569] <<24 | msg_d[570] <<16 | msg_d[571] <<8 | msg_d[572]);
  dummy1 = w * 0.01f;
  mtotactfrgrid = round2digits(dummy1);

  w = (msg_d[573] <<24 | msg_d[574] <<16 | msg_d[575] <<8 | msg_d[576]);
  dummy1 = w * 0.01f;
  mtotacttogrid = round2digits(dummy1);
  
  if ((msg_d[44] < 10) && (msg_d[42] < 10)){
    inv_date =  "0" + String(msg_d[44], DEC) + "-" + "0" + String(msg_d[42], DEC) + "-20" + String(msg_d[40], DEC);
  }
  else if ((msg_d[44] < 10) && (msg_d[42] >= 10)){
    inv_date = "0" + String(msg_d[44], DEC) + "-" + String(msg_d[42], DEC) + "-20" + String(msg_d[40], DEC);
  }
  else if ((msg_d[44] >= 10) && (msg_d[42] < 10)){
    inv_date = String(msg_d[44], DEC) + "-" + "0"  + String(msg_d[42], DEC) + "-20" + String(msg_d[40], DEC);
  }
  else {
    inv_date = String(msg_d[44], DEC) + "-" + String(msg_d[42], DEC) + "-20" + String(msg_d[40], DEC);
  }
    
  if (msg_d[46] < 10){
    HOUR = "0" + String(msg_d[46], DEC);
  }
  else{
    HOUR = String(msg_d[46], DEC);
  }
  if (msg_d[48] < 10){
    MINUTE = "0" + String(msg_d[48], DEC);
  }
  else{
    MINUTE = String(msg_d[48], DEC);
  }
  if (msg_d[50] < 10){
    SECOND = "0" + String(msg_d[50], DEC);
  }
  else {
    SECOND = String(msg_d[50], DEC);
  }

  inv_time = HOUR + ":" + MINUTE + ":" + SECOND;
    
  inv_sn = String(char(msg_d[3])) + String(char(msg_d[4])) + String(char(msg_d[5])) + String(char(msg_d[6])) + String(char(msg_d[7])) + String(char(msg_d[8])) + String(char(msg_d[9])) + String(char(msg_d[10])) + String(char(msg_d[11])) + String(char(msg_d[12])) + String(char(msg_d[13])) +String(char(msg_d[14])) + String(char(msg_d[15])) + String(char(msg_d[16])) + String(char(msg_d[17])) + String(char(msg_d[18]));
    
  sensorupdateprogress = 0;
  Decoderselect = 0;
        
  if (vaactotalDummy1 < 0){
    vaactotalDummy1 = vaactotalDummy1 * (-1);
  }
  if (pactotalDummy1 < 0){
    pactotalDummy1 = pactotalDummy1 * (-1);
    }
  if (vaactotalDummy1 == 0){
    vaactotalDummy1 = 1;
  }
  if (pactotalDummy1 == 0){
    pactotalDummy1 = 1;
  }
  if (double(vaactotalDummy1 / pactotalDummy1) > 1.0){
    double dummy = 1.0;
    pfac = dummy;
  }
  else if (double(vaactotalDummy1 / pactotalDummy1) <= 1.0) {
    dummy1 = double(vaactotalDummy1 / pactotalDummy1);
    if (dummy1 < 0){
      dummy1 = dummy1 * (-1);
    }
    pfac = round2digits(dummy1);
  }
  
  if (telnet_debug == 1){
    telnet.println("Messages decoded");
  }
}

void RcvData(){
  memstart = 100 * Decoderselect;
  ErrorTimer = millis() + Timer_2;
  while (Serial.available() < 3 ) {
    delay(1);
    if (Power_check1 == 1){
      PowerLost();
    }
    if (millis() > ErrorTimer){
      Errorshut = 1;
      Decoderselect = 0;
      if (telnet_debug == 1){
        telnet.println("Shut loop " + String(Decoderselect) + "-0");
      }
      break;
    }
  }
  if (Errorshut == 0){
    while(reader != 1){
      while (Serial.available() == 0){
        delay(1);
        if (Power_check1 == 1){
          PowerLost();
        }
      }
      if (Power_check1 == 1){
        PowerLost();
      }
      reader = Serial.read();
    }
    if (reader == 1){
      msg_d[memstart] = reader;
      while (Serial.available() == 0){
        delay(1);
        if (Power_check1 == 1){
          PowerLost();
        }
      }
      reader = Serial.read();
      if (reader == SecondByte){
        msg_d[memstart + 1] = reader;
        while (Serial.available() == 0){
          delay(1);
          if (Power_check1 == 1){
            PowerLost();
          }
        }
        reader = Serial.read();
        msg_d[memstart + 2] = reader;
        counter = memstart + 3;
        ErrorTimer = millis() + Timer_2;
        while (counter < (msg_d[memstart + 2] + 5 + memstart)){
          if (Power_check1 == 1){
            PowerLost();
          }
          while (Serial.available() == 0){
            delay(1);
            if (Power_check1 == 1){
              PowerLost();
            }
            if (millis() > ErrorTimer){
              Errorshut = 1;
              counter = msg_d[memstart + 2] + 5 + memstart;
              if (telnet_debug == 1){
                telnet.println("Shut loop " + String(Decoderselect) + "-1");
              }
              break;
            }
          }
          if (Errorshut == 0){
            msg_d[counter] = Serial.read();
          }
          counter++;
        }
        point_to = &msg_d[memstart];
        result = CRC16(point_to, (msg_d[memstart + 2] + 3));
        result1 = result;
        result2 = result >> 8;
        if ((msg_d[msg_d[memstart + 2] + 3 + memstart] == result1) && (msg_d[msg_d[memstart + 2] + 4 + memstart] == result2)){
          if (telnet_debug == 1){
            telnet.println("Message " + String(Decoderselect) + " stored");
          }
          switch (Decoderselect){
            case 0: sensorupdateprogress |= 0x0001;
                    break;
            case 1: sensorupdateprogress |= 0x0002;
                    break;
            case 2: sensorupdateprogress |= 0x0004;
                    break;
            case 3: sensorupdateprogress |= 0x0008;
                    break;
            case 4: sensorupdateprogress |= 0x0010;
                    break;
            case 5: sensorupdateprogress |= 0x0020;
                    break;
            case 6: sensorupdateprogress |= 0x0040;
                    break;
            case 7: sensorupdateprogress |= 0x0080;
                    break;
            case 8: sensorupdateprogress |= 0x0100;
                    break;
            case 9: sensorupdateprogress |= 0x0200;
                    break;
            case 10: sensorupdateprogress |= 0x0400;
                     break;
            case 11: sensorupdateprogress |= 0x0800;
                     break;
            case 12: sensorupdateprogress |= 0x1000;
                     break;
            case 13: sensorupdateprogress |= 0x2000;
                     break;
            case 14: sensorupdateprogress |= 0x4000;
                     break;
            case 15: sensorupdateprogress |= 0x8000;
                     break;
          }
        }
        else{
          if (telnet_debug == 1){
            telnet.println("Message " + String(Decoderselect) + " CRC FAIL");
          }
          Errorshut = 1;
        }
      }
      else if (reader == 132){
        msg_d[memstart + 1] = reader;
        counter = memstart + 2;
        ErrorTimer = millis() + Timer_2;
        while (counter < 5){
          if (Power_check1 == 1){
            PowerLost();
          }
          while (Serial.available() == 0){
            delay(1);
            if (Power_check1 == 1){
              PowerLost();
            }
            if (millis() > ErrorTimer){
              Errorshut = 1;
              counter = 5;
              if (telnet_debug == 1){
                telnet.println("Shut loop store 132");
              }
              break;
            }
          }
          if (Errorshut == 0){
            msg_d[counter] = Serial.read();
          }
          counter++;
        }
        
        if (telnet_debug == 1){
          telnet.println("Shut " + String(Decoderselect) + "-132");
        }
        Errorshut = 1;
      }
      else{
        msg_d[memstart + 1] = reader;
        counter = memstart + 2;
        ErrorTimer = millis() + Timer_2;
        
        while (Serial.available() > 0){
          delay(2);
          if (Power_check1 == 1){
            PowerLost();
          }
          if (millis() > ErrorTimer){
            Errorshut = 1;
            if (telnet_debug == 1){
              telnet.println("Shut loop store unknown");
            }
            break;
          }
          if (Errorshut == 0){
            msg_d[counter] = Serial.read();
          }
          counter++;
        }
        
        if (telnet_debug == 1){
          telnet.println("Unknown answer " + String(Decoderselect) + " Value: " + String(reader));
        }
        Errorshut = 1;            
      }
    }
  }
}

void loop() {

  telnet.loop();
  
  ArduinoOTA.handle();
    
  if (!client.connected()) {
    reconnect();
  }

  if (millis() > lastMsg1){
    lastMsg1 = millis()+ 10000;
    client.loop();
  }

  if (millis() > lastMsg + re_upd_conf){
    lastMsg = millis();
    switch (DeviceType){
      case 0 :  sendDiscoveryMsgs();
                break;
      case 16 :  sendDiscoveryMsgs();
                break;
      case 32 : sendDiscoveryMsgsH();
                break;
      default : break;
    }
  }

  if (millis() > updatetimer){
    updatetimer = millis() + (updateinterval*1000);
    do_update = 1;
  }
  
  if ((sensorupdateprogress == 0x0007) && (DeviceType == 0)){
    decoder();
    SendMsgs1();
    SendMsgs2();
    if (fourmppt == 1){
      SendMsgs3();
    }
    if (Power_check1 == 1){
      PowerLostCreateData();
    }
  }
  if ((sensorupdateprogress == 0x0007) && (DeviceType == 16)){
    decoder();
    SendMsgs1();
    SendMsgs2();
    if (fourmppt == 1){
      SendMsgs3();
    }
    if (Power_check1 == 1){
      PowerLostCreateData();
    }
  }
  if ((sensorupdateprogress == 0x0027) && (DeviceType == 32)){
    decoderH();
    SendMsgs1H();
    SendMsgs2H();
    SendMsgs3H();
    SendMsgs4H();
    SendMsgs5H();
  }

  if (Power_check1 == 1){
    PowerLostCreateData();
  }
  

  if ((do_update == 1) && (DeviceType == 99)){
    inv_type_check();
    if (DeviceType == 55){
      inv_type_checki();
    }
    if (DeviceType == 55){
      inv_type_checkh();
    }
    inv_epm_check();
    sensorupdateprogress = 0;
 }

  if ((Power_check == 1) && (DeviceType == 0) && (fourmppt == 0)){
    Power_check1 = 1;
  }
  else if ((Power_check == 1) && (DeviceType == 16) && (fourmppt == 0)){
    Power_check1 = 1;
  }
  
  if (Power_check1 == 1){
    PowerLost();
  }
   
  if (do_update == 1){         
    switch(DeviceType){
        case  0 : if (FirstRun == 1){
                    sendDiscoveryMsgs();
                    FirstRun = 0;
                    delay(100);
                    inv_time_check();
                    sensorupdateprogress = 0;
                  }
                  if (sensorupdateprogress == 0x0000){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg0,8);
                    Serial.flush();
                    Decoderselect = 0;
                    if (telnet_debug == 1){
                      telnet.println("Message " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 0x0001){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 0;
                        Serial.write(msg0,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                  }
                  if (sensorupdateprogress == 0x0001){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg1,8);
                    Serial.flush();
                    Decoderselect = 1;
                    if (telnet_debug == 1){
                      telnet.println("Message " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 0x0003){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 1;
                        Serial.write(msg1,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                  }
                  if (sensorupdateprogress == 0x03){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg2,8);
                    Serial.flush();
                    Decoderselect = 2;
                    if (telnet_debug == 1){
                      telnet.println("Message " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 7){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 2;
                        Serial.write(msg2,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                    do_update = 0;
                  }
                  break;

      case  16 : if (FirstRun == 1){
                    sendDiscoveryMsgs();
                    FirstRun = 0;
                    delay(100);
                    inv_time_check();
                    sensorupdateprogress = 0;
                  }
                  if (sensorupdateprogress == 0x0000){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg0,8);
                    Serial.flush();
                    Decoderselect = 0;
                    if (telnet_debug == 1){
                      telnet.println("Message " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 0x0001){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 0;
                        Serial.write(msg0,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                  }
                  if (sensorupdateprogress == 0x0001){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg1,8);
                    Serial.flush();
                    Decoderselect = 1;
                    if (telnet_debug == 1){
                      telnet.println("Message " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 0x0003){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 1;
                        Serial.write(msg1,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                  }
                  if (sensorupdateprogress == 0x03){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg2,8);
                    Serial.flush();
                    Decoderselect = 2;
                    if (telnet_debug == 1){
                      telnet.println("Message " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 7){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 2;
                        Serial.write(msg2,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                    do_update = 0;
                  }
                  break;

      case  32 :  inv_time_check1H();
                  if (FirstRun == 1){
                    sendDiscoveryMsgsH();
                    FirstRun = 0;
                    delay(100);
                    sensorupdateprogress = 0;
                  }
                  if (sensorupdateprogress == 0x0000){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg0H,8);
                    Serial.flush();
                    Decoderselect = 0;
                    if (telnet_debug == 1){
                      telnet.println("Message Hybrid " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 0x0001){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 0;
                        Serial.write(msg0H,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message Hybrid " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                  }
                  if (sensorupdateprogress == 0x0001){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg1H,8);
                    Serial.flush();
                    Decoderselect = 1;
                    if (telnet_debug == 1){
                      telnet.println("Message Hybrid " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 0x0003){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 1;
                        Serial.write(msg1H,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message Hybrid " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                  }
                  if (sensorupdateprogress == 0x0003){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg2H,8);
                    Serial.flush();
                    Decoderselect = 2;
                    if (telnet_debug == 1){
                      telnet.println("Message Hybrid " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 0x0007){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 2;
                        Serial.write(msg2H,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message Hybrid " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                  }
                  if (sensorupdateprogress == 0x0007){
                    if (Power_check1 == 1){
                      PowerLost();
                    }
                    SecondByte = 4;
                    Errorshutcounter = 0;
                    delay(req_waiter);
                    Serial.write(msg3H,8);
                    Serial.flush();
                    Decoderselect = 5;
                    if (telnet_debug == 1){
                      telnet.println("Message Hybrid " + String(Decoderselect) + " request sent");
                    }
                    while (sensorupdateprogress != 0x0027){
                      if(Errorshut == 1){
                        if (Errorshutcounter == 5){
                          Errorshut = 0;
                          do_update = 0;
                          Errorshutcounter = 0;
                          break;
                        }
                        delay(req_waiter);
                        Decoderselect = 5;
                        Serial.write(msg3H,8);
                        Serial.flush();
                        Errorshut = 0;
                        if (telnet_debug == 1){
                          telnet.println("Shutcounter: " + String(Errorshutcounter));
                        }
                        Errorshutcounter++;
                        if (telnet_debug == 1){
                          telnet.println("Message Hybrid " + String(Decoderselect) + " request resent");
                        }
                      }
                      ErrorTimer = millis() + Timer_1;
                      RcvData();
                    }
                    do_update = 0;
                  }
                  break;
  
      default :   S_E_msgs();
                  do_update = 0;
                  break;
    }
  }
}
