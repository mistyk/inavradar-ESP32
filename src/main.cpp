#define RADARESP32
//#define RADARAT168

#include <lib/MSP.h>
#include <lib/LoRa.h>
#include <Arduino.h>
#include <SimpleCLI.h>
using namespace simplecli;
#include <main.h>

#ifdef RADARESP32
#include <SSD1306.h>
#include <esp_system.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <FS.h>
#include <Hash.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <lib/ESPAsyncWiFiManager.h>

#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)
#endif

#ifdef RADARAT168
#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)
#endif

#define CFGVER 8 // bump up to overwrite setting with new defaults
// ----------------------------------------------------------------------------- global vars
config cfg;
MSP msp;
bool booted = 0;
Stream *serialConsole[1];
int cNum = 0;
bool dalternate = 1;

#ifdef RADARESP32
SSD1306 display (0x3c, 4, 15);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");
DNSServer dns;
AsyncWiFiManager wifiManager(&server,&dns);
#endif

long sendLastTime = 0;
long displayLastTime = 0;
long pdLastTime = 0;

msp_analog_t fcanalog;
msp_raw_gps_t homepos; // set on arm
planeData pd; // our uav data
planeData pdIn; // new air packet
planesData pds[5]; // uav db
planeData fakepd; // debugging
char planeFC[20]; // uav fc name
bool wifiON = 1;
bool loraRX = 0; // display RX
bool loraTX = 0; // display TX
planeData loraMsg; // incoming packet

#ifdef RADARESP32
// ----------------------------------------------------------------------------- EEPROM / config
void saveConfig () {
  for(size_t i = 0; i < sizeof(cfg); i++) {
    char data = ((char *)&cfg)[i];
    EEPROM.write(i, data);
  }
  EEPROM.commit();
}

void initConfig () {
  size_t size = sizeof(cfg);
  EEPROM.begin(size * 2);
  for(size_t i = 0; i < size; i++)  {
    char data = EEPROM.read(i);
    ((char *)&cfg)[i] = data;
  }
  if (cfg.configVersion != CFGVER) { // write default config
    cfg.configVersion = CFGVER;
    String("ADS-RC").toCharArray(cfg.loraHeader,7); // protocol identifier
    cfg.loraAddress = 2; // local lora address
    cfg.loraFrequency = 433E6; // 433E6, 868E6, 915E6
    cfg.loraBandwidth =  250000;// 250000 bps
    cfg.loraCodingRate4 = 6; // Error correction rate 4/6
    cfg.loraSpreadingFactor = 7; // 7 is shortest time on air - 12 is longest
    cfg.intervalSend = 250; // in ms + random
    cfg.intervalDisplay = 100; // in ms
    cfg.intervalStatus = 1000; // in ms
    cfg.uavTimeout = 10; // in sec
    cfg.mspTX = 23; // pin for msp serial TX
    cfg.mspRX = 17; // pin for msp serial RX
    cfg.mspPOI = 1; // POI type: 1 (Wayponit), 2 (Plane) # TODO
    cfg.debugOutput = true;
    cfg.debugFakeWPs = false;
    cfg.debugFakePlanes = false;
    cfg.debugFakeMoving = false;
    EEPROM.begin(size * 2);
    for(size_t i = 0; i < size; i++) {
      char data = ((char *)&cfg)[i];
      EEPROM.write(i, data);
    }
    EEPROM.commit();
    Serial.println("Default config written to EEPROM!");
  } else {
    Serial.println("Config found!");
  }
}
// ----------------------------------------------------------------------------- calc gps distance
#include <math.h>
#include <cmath>
#define earthRadiusKm 6371.0

// This function converts decimal degrees to radians
double deg2rad(double deg) {
  return (deg * M_PI / 180);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
  return (rad * 180 / M_PI);
}

/**
 * Returns the distance between two points on the Earth.
 * Direct translation from http://en.wikipedia.org/wiki/Haversine_formula
 * @param lat1d Latitude of the first point in degrees
 * @param lon1d Longitude of the first point in degrees
 * @param lat2d Latitude of the second point in degrees
 * @param lon2d Longitude of the second point in degrees
 * @return The distance between the two points in kilometers
 */
double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = deg2rad(lat1d);
  lon1r = deg2rad(lon1d);
  lat2r = deg2rad(lat2d);
  lon2r = deg2rad(lon2d);
  u = sin((lat2r - lat1r)/2);
  v = sin((lon2r - lon1r)/2);
  return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}
// ----------------------------------------------------------------------------- String split

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
// ----------------------------------------------------------------------------- CLI
SimpleCLI* cli;

int  serIn;             // var that will hold the bytes-in read from the serialBuffer
char serInString[100];  // array that will hold the different bytes  100=100characters;
                        // -> you must state how long the array will be else it won't work.
int  serInIndx  = 0;    // index of serInString[] in which to insert the next incoming byte
int  serOutIndx = 0;    // index of the outgoing serInString[] array;

void readCli () {
  int sb;
  if(serialConsole[0]->available()) {
    while (serialConsole[0]->available()){
      sb = serialConsole[0]->read();
      serInString[serInIndx] = sb;
      serInIndx++;
      serialConsole[0]->write(sb);
      if (sb == '\n') {
        cNum = 0;
        cli->parse(serInString);
        serInIndx = 0;
        memset(serInString, 0, sizeof(serInString));
        serialConsole[0]->print("> ");
      }
    }
  }
}
void cliLog (String log) {
  if (cfg.debugOutput) {
    if (booted) {
      Serial.println();
      Serial.print("LOG: ");
      Serial.print(log);
    } else {
      Serial.println(log);
    }
  }
}
void cliStatus(int n) {
  serialConsole[n]->println("================== Status ==================");
  serialConsole[n]->print("FC:               ");
  serialConsole[n]->println(planeFC);
  serialConsole[n]->print("Name:             ");
  serialConsole[n]->println(pd.planeName);
  serialConsole[n]->print("Battery:          ");
  serialConsole[n]->print((float)fcanalog.vbat/10);
  serialConsole[n]->println(" V");
  serialConsole[n]->print("Arm state:        ");
  serialConsole[n]->println(pd.armState ? "ARMED" : "DISARMED");
  serialConsole[n]->print("GPS:              ");
  serialConsole[n]->println(String(pd.gps.numSat) + " Sats");
  serialConsole[n]->println("statusend");
}
void cliHelp(int n) {
  serialConsole[n]->println("================= Commands =================");
  serialConsole[n]->println("status            - Show whats going on");
  serialConsole[n]->println("help              - List all commands");
  serialConsole[n]->println("config            - List all settings");
  serialConsole[n]->println("reboot            - Reset MCU and radio");
  serialConsole[n]->println("gpspos            - Show last GPS position");
  //serialConsole[n]->println("fcpass            - start FC passthru mode");
  serialConsole[n]->println("debug             - Toggle debug output");
  serialConsole[n]->println("localfakeplanes   - Send fake planes to FC");
  serialConsole[n]->println("radiofakeplanes   - Send fake planes via radio");
  serialConsole[n]->println("movefakeplanes    - Move fake planes");
}
void cliConfig(int n) {
  serialConsole[n]->println("=============== Configuration ==============");
  serialConsole[n]->print("Lora local address:    ");
  serialConsole[n]->println(cfg.loraAddress);
  serialConsole[n]->print("Lora frequency:        ");
  serialConsole[n]->print(cfg.loraFrequency);
  serialConsole[n]->println(" Hz");
  serialConsole[n]->print("Lora bandwidth:        ");
  serialConsole[n]->print(cfg.loraBandwidth);
  serialConsole[n]->println(" Hz");
  serialConsole[n]->print("Lora spreading factor: ");
  serialConsole[n]->println(cfg.loraSpreadingFactor);
  serialConsole[n]->print("Lora coding rate 4:    ");
  serialConsole[n]->println(cfg.loraCodingRate4);
  serialConsole[n]->print("UAV timeout:           ");
  serialConsole[n]->print(cfg.uavTimeout);
  serialConsole[n]->println(" sec");
  serialConsole[n]->print("MSP RX pin:            ");
  serialConsole[n]->println(cfg.mspRX);
  serialConsole[n]->print("MSP TX pin:            ");
  serialConsole[n]->println(cfg.mspTX);
  serialConsole[n]->print("Debug output:          ");
  serialConsole[n]->println(cfg.debugOutput ? "ON" : "OFF");
  serialConsole[n]->print("Local fake planes:     ");
  serialConsole[n]->println(cfg.debugFakeWPs ? "ON" : "OFF");
  serialConsole[n]->print("Radio fake planes:     ");
  serialConsole[n]->println(cfg.debugFakePlanes ? "ON" : "OFF");
  serialConsole[n]->print("Move fake planes:      ");
  serialConsole[n]->println(cfg.debugFakeMoving ? "ON" : "OFF");
  serialConsole[n]->println("cfgend");
}
void cliDebug(int n) {
  cfg.debugOutput = !cfg.debugOutput;
  saveConfig();
  serialConsole[n]->println("CLI debugging: " + String(cfg.debugOutput));
}
void cliLocalFake(int n) {
  cfg.debugFakeWPs = !cfg.debugFakeWPs;
  saveConfig();
  serialConsole[n]->println("Local fake planes: " + String(cfg.debugFakeWPs));
}
void cliRadioFake(int n) {
  cfg.debugFakePlanes = !cfg.debugFakePlanes;
  saveConfig();
  serialConsole[n]->println("Radio fake planes: " + String(cfg.debugFakePlanes));
}
void cliMoveFake(int n) {
  cfg.debugFakeMoving = !cfg.debugFakeMoving;
  saveConfig();
  serialConsole[n]->println("Move fake planes: " + String(cfg.debugFakeMoving));
}
void cliReboot(int n) {
  serialConsole[n]->println("Rebooting ...");
  serialConsole[n]->println();
  delay(1000);
  ESP.restart();
}
void cliGPSpos(int n) {
  serialConsole[n]->print("Status: ");
  serialConsole[n]->println(pd.gps.fixType ? String(pd.gps.numSat) + " Sats" : "no fix");
  serialConsole[n]->print("Position: ");
  serialConsole[n]->print(pd.gps.lat/10000000);
  serialConsole[n]->print(",");
  serialConsole[n]->print(pd.gps.lon/10000000);
  serialConsole[n]->print(",");
  serialConsole[n]->println(pd.gps.alt);
}
void cliFCpass(int n) {
  serialConsole[n]->println("=============== FC passthru ================");
}

void initCli () {
  cli = new SimpleCLI();
  cli->onNotFound = [](String str) {
    Serial.println("\"" + str + "\" not found");
  };
  cli->addCmd(new Command("status", [](Cmd* cmd) { cliStatus(cNum); } ));
  cli->addCmd(new Command("help", [](Cmd* cmd) { cliHelp(cNum); } ));
  cli->addCmd(new Command("debug", [](Cmd* cmd) { cliDebug(cNum); } ));
  cli->addCmd(new Command("localfakeplanes", [](Cmd* cmd) { cliLocalFake(cNum); } ));
  cli->addCmd(new Command("radiofakeplanes", [](Cmd* cmd) { cliRadioFake(cNum); } ));
  cli->addCmd(new Command("movefakeplanes", [](Cmd* cmd) { cliMoveFake(cNum); } ));
  cli->addCmd(new Command("reboot", [](Cmd* cmd) { cliReboot(cNum); } ));
  cli->addCmd(new Command("gpspos", [](Cmd* cmd) { cliGPSpos(cNum); } ));

  Command* config = new Command("config", [](Cmd* cmd) {
    String arg1 = cmd->getValue(0);
    String arg2 = cmd->getValue(1);
    if (arg1 == "") cliConfig(cNum);
    if (arg1 == "loraFreq") {
      if ( arg2.toInt() == 433E6 || arg2.toInt() == 868E6 || arg2.toInt() == 915E6) {
        cfg.loraFrequency = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora frequency changed!");
      } else {
        serialConsole[cNum]->println("Lora frequency not correct: 433000000, 868000000, 915000000");
      }
    }
    if (arg1 == "loraBandwidth") {
      if (arg2.toInt() == 250000 || arg2.toInt() == 62500 || arg2.toInt() == 7800) {
        cfg.loraBandwidth = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora bandwidth changed!");
      } else {
        serialConsole[cNum]->println("Lora bandwidth not correct: 250000, 62500, 7800");
      }
    }
    if (arg1 == "loraSpread") {
    if (arg2.toInt() >= 7 && arg2.toInt() <= 12) {
        cfg.loraSpreadingFactor = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora spreading factor changed!");
      } else {
        serialConsole[cNum]->println("Lora Lora spreading factor not correct: 7 - 12");
      }
    }
    if (arg1 == "loraAddress") {
      if (arg2.toInt() >= 1 && arg2.toInt() <= 250) {
        cfg.loraAddress = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora address changed!");
      } else {
        serialConsole[cNum]->println("Lora address not correct: 1 - 250");
      }
    }
    if (arg1 == "uavtimeout") {
      if (arg2.toInt() >= 5 && arg2.toInt() <= 600) {
        cfg.uavTimeout = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("UAV timeout changed!");
      } else {
        serialConsole[cNum]->println("UAV timeout not correct: 5 - 600");
      }
    }
  });
  config->addArg(new AnonymOptArg(""));
  config->addArg(new AnonymOptArg(""));

  cli->addCmd(config);
  //cli->parse("ping");
  //cli->parse("hello");
}
#endif
// ----------------------------------------------------------------------------- Wifi
String wiConfig () {
  String out;
  out += "Config: ";
  out += "Lora local address>";
  out += cfg.loraAddress;
  out += "<Lora frequency>";
  out += cfg.loraFrequency;
  out += "<Lora bandwidth>";
  out += cfg.loraBandwidth;
  out += "<Lora spreading factor>";
  out += cfg.loraSpreadingFactor;
  out += "<Lora coding rate 4>";
  out += cfg.loraCodingRate4;
  out += "<UAV timeout>";
  out += cfg.uavTimeout;
  out += "<MSP RX pin>";
  out += cfg.mspRX;
  out += "<MSP TX pin>";
  out += cfg.mspTX;
  out += "<Debug output>";
  out += cfg.debugOutput;
  out += "<Local fake planes>";
  out += cfg.debugFakeWPs;
  out += "<Radio fake planes>";
  out += cfg.debugFakePlanes;
  out += "<Move fake planes>";
  out += cfg.debugFakeMoving;
  return String(out);
}
void newWifiMsg (AsyncWebSocketClient * client, String msg) {
  cliLog(getValue(msg, ' ', 0));
  String arg0 = getValue(msg, ' ', 0);
  String arg1 = getValue(msg, ' ', 1);


  if (arg0 == "debug\n") {
    cfg.debugOutput = !cfg.debugOutput;
    saveConfig();
    serialConsole[cNum]->println("CLI debugging: " + String(cfg.debugOutput));
  }
  if (arg0 == "localfakeplanes\n") {
    cfg.debugFakeWPs = !cfg.debugFakeWPs;
    saveConfig();
    serialConsole[cNum]->println("Local fake planes: " + String(cfg.debugFakeWPs));
  }
  if (arg0 == "radiofakeplanes\n") {
    cfg.debugFakePlanes = !cfg.debugFakePlanes;
    saveConfig();
    serialConsole[cNum]->println("Radio fake planes: " + String(cfg.debugFakePlanes));
  }
  if (arg0 == "movefakeplanes\n") {
    cfg.debugFakeMoving = !cfg.debugFakeMoving;
    saveConfig();
    serialConsole[cNum]->println("Move fake planes: " + String(cfg.debugFakeMoving));
  }
  if (arg0 == "config") {
    String arg2 = getValue(msg, ' ', 2);
    if (arg1 == "loraFreq") {
      if ( arg2.toInt() == 433E6 || arg2.toInt() == 868E6 || arg2.toInt() == 915E6) {
        cfg.loraFrequency = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora frequency changed!");
      } else {
        serialConsole[cNum]->println("Lora frequency not correct: 433000000, 868000000, 915000000");
      }
    }
    if (arg1 == "loraBandwidth") {
      if (arg2.toInt() == 250000 || arg2.toInt() == 62500 || arg2.toInt() == 7800) {
        cfg.loraBandwidth = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora bandwidth changed!");
      } else {
        serialConsole[cNum]->println("Lora bandwidth not correct: 250000, 62500, 7800");
      }
    }
    if (arg1 == "loraSpread") {
    if (arg2.toInt() >= 7 && arg2.toInt() <= 12) {
        cfg.loraSpreadingFactor = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora spreading factor changed!");
      } else {
        serialConsole[cNum]->println("Lora Lora spreading factor not correct: 7 - 12");
      }
    }
    if (arg1 == "loraAddress") {
      if (arg2.toInt() >= 1 && arg2.toInt() <= 250) {
        cfg.loraAddress = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora address changed!");
      } else {
        serialConsole[cNum]->println("Lora address not correct: 1 - 250");
      }
    }
    if (arg1 == "uavtimeout") {
      if (arg2.toInt() >= 5 && arg2.toInt() <= 600) {
        cfg.uavTimeout = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("UAV timeout changed!");
      } else {
        serialConsole[cNum]->println("UAV timeout not correct: 5 - 600");
      }
    }
  }
}
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    //client->printf("Status: %s, %s, %s, %s, %s", String(planeFC).c_str(), String(pd.planeName).c_str(), String(((float)fcanalog.vbat/10)).c_str(),String(pd.armState).c_str(),String(pd.gps.numSat).c_str());
    client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
  } else if(type == WS_EVT_ERROR){
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if(type == WS_EVT_PONG){
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");
  } else if(type == WS_EVT_DATA){
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    String msg = "";
    if(info->final && info->index == 0 && info->len == len){
      //the whole message is in a single frame and we got all of it's data
      Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      if(info->opcode == WS_TEXT)
        newWifiMsg(client, msg);
      else
        client->binary("I got your binary message");
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(info->index == 0){
        if(info->num == 0)
          Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      if((info->index + len) == info->len){
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if(info->final){
          Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
          if(info->message_opcode == WS_TEXT)
            client->text("I got your text message");
          else
            client->binary("I got your binary message");
        }
      }
    }
  }
}



const char * hostName = "esp-async";
const char* http_username = "admin";
const char* http_password = "admin";

void wifisetup(){

  display.drawString (0, 32, "WIFI ");
  display.display();
  wifiManager.autoConnect("INAV-Radar");
  display.drawString (100, 32, "OK");
  display.display();
  //wifiManager.startConfigPortal("INAV-Radar");

  //Send OTA events to the browser
  ArduinoOTA.onStart([]() { events.send("Update Start", "ota"); });
  ArduinoOTA.onEnd([]() { events.send("Update End", "ota"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char p[32];
    sprintf(p, "Progress: %u%%\n", (progress/(total/100)));
    events.send(p, "ota");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if(error == OTA_AUTH_ERROR) events.send("Auth Failed", "ota");
    else if(error == OTA_BEGIN_ERROR) events.send("Begin Failed", "ota");
    else if(error == OTA_CONNECT_ERROR) events.send("Connect Failed", "ota");
    else if(error == OTA_RECEIVE_ERROR) events.send("Recieve Failed", "ota");
    else if(error == OTA_END_ERROR) events.send("End Failed", "ota");
  });
  ArduinoOTA.setHostname(hostName);
  ArduinoOTA.begin();

  MDNS.addService("http","tcp",80);

  SPIFFS.begin();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  events.onConnect([](AsyncEventSourceClient *client){
    //client->send("hello!",NULL,millis(),1000);
    client->send(String("Status: " + String(planeFC)+ ", " +String(pd.planeName)+ ", " +String(((float)fcanalog.vbat/10))+ ", " +String(pd.armState)+ ", " +String(pd.gps.numSat)).c_str(),NULL,millis(),1000);

    client->send(wiConfig().c_str(),NULL);
  });
  server.addHandler(&events);

  server.addHandler(new SPIFFSEditor(SPIFFS, http_username,http_password));

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

  server.onNotFound([](AsyncWebServerRequest *request){
    Serial.printf("NOT_FOUND: ");
    if(request->method() == HTTP_GET)
      Serial.printf("GET");
    else if(request->method() == HTTP_POST)
      Serial.printf("POST");
    else if(request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if(request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if(request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if(request->method() == HTTP_HEAD)
      Serial.printf("HEAD");
    else if(request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

    if(request->contentLength()){
      Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
      Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
    }

    int headers = request->headers();
    int i;
    for(i=0;i<headers;i++){
      AsyncWebHeader* h = request->getHeader(i);
      Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
    }

    int params = request->params();
    for(i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isFile()){
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      } else if(p->isPost()){
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      } else {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }

    request->send(404);
  });
  server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index)
      Serial.printf("UploadStart: %s\n", filename.c_str());
    Serial.printf("%s", (const char*)data);
    if(final)
      Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
  });
  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    if(!index)
      Serial.printf("BodyStart: %u\n", total);
    Serial.printf("%s", (const char*)data);
    if(index + len == total)
      Serial.printf("BodyEnd: %u\n", total);
  });
  server.begin();
}


// ----------------------------------------------------------------------------- LoRa
void sendMessage(planeData *outgoing) {
  while (!LoRa.beginPacket()) {  }
  LoRa.write((uint8_t*)outgoing, sizeof(fakepd));
  LoRa.endPacket(false);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;
  LoRa.readBytes((uint8_t *)&loraMsg, packetSize);
  //cliLog(loraMsg.header);
  //cliLog(cfg.loraHeader);
  if (String(loraMsg.header) == String(cfg.loraHeader)) { // new plane data
      loraRX = 1;
      pdIn = loraMsg;
      cliLog("New air packet");
      bool found = 0;
      int free = -1;
      for (size_t i = 0; i <= 4; i++) {
        if (String(pds[i].pd.planeName) == String(pdIn.planeName) ) { // update plane
          pds[i].pd = pdIn;
          pds[i].pd.armState = 1;
          pds[i].waypointNumber = i+1;
          pds[i].lastUpdate = millis();
          found = 1;
          cliLog("UAV DB update POI #" + String(i+1));
          break;
        }
        if (free == -1 && pds[i].waypointNumber == 0) free = i; // find free slot
      }
      if (!found) { // if not there put it in free slot
          pds[free].waypointNumber = free+1;
          pds[free].pd = pdIn;
          pds[free].pd.armState = 1;
          pds[free].lastUpdate = millis();
          cliLog("UAV DB new POI #" + String(free+1));
          free = 0;
      }
  }
}
int moving = 0;
void sendFakePlanes () {
  if (cfg.debugFakeMoving && moving > 100) {
    moving = 0;
  } else {
    if (!cfg.debugFakeMoving) moving = 0;
  }
  cliLog("Sending fake UAVs via radio ...");
  String("ADS-RC").toCharArray(fakepd.header,7);
  fakepd.loraAddress = (char)5;
  String("Testplane #1").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
//  fakepd.gps.lat = homepos.lat + (10 * moving);
//  fakepd.gps.lon = homepos.lon;
  fakepd.gps.alt = 300;
  fakepd.gps.groundSpeed = 50;
  //sendMessage(&fakepd);
  //delay(300);
  /*
  fakepd.loraAddress = (char)2;
  String("Testplane #2").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1; */
  // -------------------------------------------------------- fixed GPS pos radio fake planes

  fakepd.gps.lat = 47.345446 * 10000000; // + (500 * moving);
  fakepd.gps.lon = -1.543392 * 10000000;
  sendMessage(&fakepd);
  // 50.088233, 8.782278 ... 50.088233, 8.785693 ... 341 * 100
  // 50.100400, 8.762835
  // 47.345446, -1.543392
  /*
  delay(300);
  fakepd.loraAddress = (char)3;
  String("Testplane #3").toCharArray(fakepd.planeName,20);
  fakepd.gps.alt = 500;
  fakepd.armState=  1;
  fakepd.gps.lat = 50.088233 * 10000000;
  fakepd.gps.lon = 8.782278 * 10000000 + (341 * moving);
  sendMessage(&fakepd);
  delay(300);
  fakepd.loraAddress = (char)4;
  String("Testplane #4").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
  fakepd.gps.lat = 50.088233 * 10000000;
  fakepd.gps.lon = 8.782278 * 10000000 + (600 * moving);
  fakepd.gps.alt = 500;
  sendMessage(&fakepd);
  delay(300);
  fakepd.loraAddress = (char)5;
  String("Testplane #5").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
  fakepd.gps.lat = 50.1006770 * 10000000 + (1000 * moving);
  fakepd.gps.lon = 8.762406 * 10000000;
  sendMessage(&fakepd);
  delay(300);
*/
  cliLog("Fake UAVs sent.");
  moving++;
}
void initLora() {
  #ifdef RADARESP32
	display.drawString (0, 8, "LORA");
  display.display();
  #endif
  cliLog("Starting radio ...");
  pd.loraAddress = cfg.loraAddress;
  String(cfg.loraHeader).toCharArray(pd.header,7);
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(cfg.loraFrequency)) {
    cliLog("Radio start failed!");
    #ifdef RADARESP32
    display.drawString (94, 8, "FAIL");
    #endif
    while (1);
  }
  LoRa.sleep();
  LoRa.setSignalBandwidth(cfg.loraBandwidth);
  LoRa.setCodingRate4(cfg.loraCodingRate4);
  LoRa.setSpreadingFactor(cfg.loraSpreadingFactor);
  LoRa.idle();
  LoRa.onReceive(onReceive);
  LoRa.enableCrc();
  LoRa.receive();
  cliLog("Radio started.");
  #ifdef RADARESP32
  display.drawString (100, 8, "OK");
  display.display();
  #endif
}
// ----------------------------------------------------------------------------- Display
#ifdef RADARESP32

void drawDisplay () {
  display.clear();
  if (dalternate) {
    display.setFont (ArialMT_Plain_10);
    display.setTextAlignment (TEXT_ALIGN_LEFT);
    display.drawString (0,54, pd.armState ? "Armed" : "Disarmed");
    display.drawString (48,54, String(pd.gps.numSat) + " Sat");
    display.drawString (84,54, "TX");
    display.drawString (106,54, "RX");

    display.drawXbm(98, 55, 8, 8, loraTX ? activeSymbol : inactiveSymbol);
    display.drawXbm(120, 55, 8, 8, loraRX ? activeSymbol : inactiveSymbol);

    for (size_t i = 0; i <=4 ; i++) {
      if (pds[i].waypointNumber != 0) {
        display.drawString (0,i*8, pds[i].pd.planeName);
        display.drawString (80,i*8,String(pds[i].distance));
      }
    }
  } else {
    display.setFont (ArialMT_Plain_24);
    display.setTextAlignment (TEXT_ALIGN_CENTER);
    display.drawString (64,32, String((float)fcanalog.vbat/10) + " V");
  }
  display.display();
}

void initDisplay () {
  cliLog("Starting display ...");
  pinMode (16, OUTPUT);
  pinMode (2, OUTPUT);
  digitalWrite (16, LOW); // set GPIO16 low to reset OLED
  delay (50);
  digitalWrite (16, HIGH); // while OLED is running, GPIO16 must go high
  display.init ();
  display.flipScreenVertically ();
  display.setFont (ArialMT_Plain_10);
  display.setTextAlignment (TEXT_ALIGN_LEFT);
  display.drawString (0, 0, "DISPLAY");
  display.drawString (100, 0, "OK");
  display.display();
  cliLog("Display started!");
}
#endif
// ----------------------------------------------------------------------------- MSP and FC
void getPlanetArmed () {
  uint32_t planeModes;
  msp.getActiveModes(&planeModes);
  //Serial.print("Arm State: ");
  //Serial.println(bitRead(planeModes,0));
  pd.armState = bitRead(planeModes,0);
}

void getPlaneGPS () {
  if (msp.request(MSP_RAW_GPS, &pd.gps, sizeof(pd.gps))) {
  }
}

void getPlaneBat () {
  if (msp.request(MSP_ANALOG, &fcanalog, sizeof(fcanalog))) {
    //cliLog(String(fcanalog.vbat));
  }
}

void getPlaneData () {
  String("No Name").toCharArray(pd.planeName,20);
  String("No FC").toCharArray(planeFC,20);
  if (msp.request(10, &pd.planeName, sizeof(pd.planeName))) {
    //Serial.println(pd.planeName);
  }
  if (msp.request(2, &planeFC, sizeof(planeFC))) {
    //Serial.println(planeFC);
  }
}

void planeSetWP () {
  msp_set_wp_t wp;
  for (size_t i = 0; i <= 4; i++) {
    if (pds[i].waypointNumber != 0) {
      wp.waypointNumber = pds[i].waypointNumber;
      wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
      wp.lat = pds[i].pd.gps.lat;
      wp.lon = pds[i].pd.gps.lon;
      wp.alt = pds[i].pd.gps.alt;
      wp.p1 = pds[i].pd.gps.groundSpeed;
      cliLog("P1:" + String(wp.p1));
      wp.p2 = 0;
      wp.p3 = pds[i].pd.armState;
      cliLog("P3:" + String(wp.p3));
      if (i == 4 || pds[i+1].waypointNumber==0) {
        wp.flag = 0xa5;
        cliLog("last flag - POI #" + String(wp.waypointNumber));
      }
      else wp.flag = 0;
      msp.command(MSP_SET_WP, &wp, sizeof(wp));
      cliLog("Sent to FC - POI #" + String(wp.waypointNumber));
    }
    //else break;
  }
  //cliLog("POIs sent to FC.");
  //msp.command(MSP_SET_WP, &wp, sizeof(wp));
}
void planeFakeWPv2 () {
  msp_raw_planes_t wp;
  wp.id = 1;
  wp.arm_state = 1;
  wp.lat = 50.1006770 * 10000000;
  wp.lon = 8.7613380 * 10000000;
  wp.alt = 100;
  wp.heading = 0;
  wp.speed = 0;
  wp.callsign0 = 'D';
  wp.callsign1 = 'A';
  wp.callsign2 = 'N';
  msp.commandv2(MSP2_ESP32, &wp, sizeof(wp));
}

void planeFakeWP () {
    msp_set_wp_t wp;
    if (cfg.debugFakeMoving && moving > 100) {
      moving = 0;
    } else {
      moving = 0;
    }
/*    wp.waypointNumber = 1;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.1006770 * 10000000;
    wp.lon = 8.7613380 * 10000000;
    wp.alt = 100;
    wp.p1 = 200;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));
    wp.waypointNumber = 2;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.1020320 * 10000000;
    wp.lon = 8.7615830 * 10000000;
    wp.alt = 200;
    wp.p1 = 100;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));
    wp.waypointNumber = 3;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.102137 * 10000000;
    wp.lon = 8.762990 * 10000000;
    wp.alt = 300;
    wp.p1 = 0;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));
    wp.waypointNumber = 4;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.100547 * 10000000;
    wp.lon = 8.764052 * 10000000;
    wp.alt = 0;
    wp.p1 = 500;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));
*/

    if (pd.gps.fixType > 0) {
      wp.waypointNumber = 1;
      wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
      wp.lat = homepos.lat-100 + (moving * 20);
      wp.lon = homepos.lon;
      wp.alt = homepos.alt + 300;
      wp.p1 = 1000;
      wp.p2 = 0;
      wp.p3 = 1;
      wp.flag = 0xa5;
      msp.command(MSP_SET_WP, &wp, sizeof(wp));
      cliLog("Fake POIs sent to FC.");
    } else {
      cliLog("Fake POIs waiting for GPS fix.");
    }
}

void initMSP () {
  cliLog("Starting MSP ...");
  #ifdef RADARESP32
  display.drawString (0, 16, "MSP ");
  display.display();
  #endif
  delay(200);
  Serial1.begin(115200, SERIAL_8N1, cfg.mspRX , cfg.mspTX);
  msp.begin(Serial1);
  cliLog("MSP started!");
  #ifdef RADARESP32
  display.drawString (100, 16, "OK");
  display.display();
  display.drawString (0, 24, "FC ");
  display.display();
  #endif
  cliLog("Waiting for FC to start ...");
  delay(2000);
  getPlaneData();
  //getPlanetArmed();
  getPlaneGPS();
  cliLog("FC detected: " + String(planeFC));
  #ifdef RADARESP32
  display.drawString (100, 24, planeFC);
  display.display();
  #endif
}




// ----------------------------------------------------------------------------- main init
void setup() {

/*  cli.RegisterCmd("status",&cliStatus);
  cli.RegisterCmd("help",&cliHelp);
  cli.RegisterCmd("config",&cliConfig);
  cli.RegisterCmd("debug",&cliDebug);
  cli.RegisterCmd("reboot",&cliReboot); */

  Serial.begin(115200);
  serialConsole[0] = &Serial;
  initCli();
  initConfig();
  initDisplay();
  initLora();
  delay(1500);
  initMSP();
  delay(1000);
  //wifisetup();

  for (size_t i = 0; i <= 4; i++) {
    pds[i].pd.loraAddress= 0x00;
    pds[i].waypointNumber = 0;
  }
  booted = 1;
  serialConsole[0]->print("> ");
}
void noloop()
{
  int relativAlt=8;
	//Calculate Height of waypoint angle=asin(h/d)
	int distanceFromMe=9;

  float anglePoiY=relativAlt/distanceFromMe;
  delay(1000);
  cliLog(String(anglePoiY));
}

// ----------------------------------------------------------------------------- main loop
void loop() {
  ArduinoOTA.handle();

  if (millis() - displayLastTime > cfg.intervalDisplay) {
    #ifdef RADARESP32
    drawDisplay();
    readCli();
    #endif

    loraTX = 0;
    loraRX = 0;
    displayLastTime = millis();
  }
  if (millis() - pdLastTime > cfg.intervalStatus) {
    for (size_t i = 0; i <= 4; i++) {
      if (pd.gps.fixType != 0) pds[i].distance = distanceEarth(pd.gps.lat/10000000, pd.gps.lon/10000000, pds[i].pd.gps.lat/10000000, pds[i].pd.gps.lon/10000000);
      if (pds[i].pd.loraAddress != 0 && millis() - pds[i].lastUpdate > cfg.uavTimeout*1000 ) { // plane timeout
        //pds[i].pd.gps.lat = 0;
        //pds[i].pd.gps.lon = 0;
        //pds[i].pd.gps.alt = 0;
        pds[i].pd.armState = 2;
        planeSetWP();
        planeSetWP();
        pds[i].waypointNumber = 0;
        pds[i].pd.loraAddress = 0;
        String("").toCharArray(pds[i].pd.planeName,20);
        cliLog("UAV DB delete POI #" + String(i+1));
      }
    }
    getPlaneData();
    dalternate = !dalternate;
    if (String(pd.planeName) != "No Name" ) {
      getPlanetArmed();
      getPlaneBat();
      if (!pd.armState) getPlaneGPS();
    }

    if (!pd.armState) {

      loraTX = 1;
      if (pd.gps.fixType != 0) sendMessage(&pd);
      LoRa.receive();
      homepos = pd.gps;

    }
    pdLastTime = millis();
  }

  if ((millis() - sendLastTime) > cfg.intervalSend ) {
    sendLastTime = millis()+ random(0, 50);

    if (pd.armState) {

      getPlaneGPS();
      loraTX = 1;
      if (pd.gps.fixType != 0) sendMessage(&pd);
      LoRa.receive();
    }
    if (cfg.debugFakePlanes) sendFakePlanes();
    //if (pd.armState) planeSetWP();
    planeSetWP();
    if (cfg.debugFakeWPs) planeFakeWP();
    //planeFakeWPv2();

  }
}
