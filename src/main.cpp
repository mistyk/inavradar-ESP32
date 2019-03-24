#include <Arduino.h>
#include <esp_system.h>
#include <lib/MSP.h>
#include <lib/LoRa.h>
#include <SSD1306.h>
#include <EEPROM.h>
#include <SimpleCLI.h>
using namespace simplecli;
#include <main.h>

#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)

#define CFGVER 15 // bump up to overwrite setting with new defaults
// ----------------------------------------------------------------------------- global vars
config cfg;
MSP msp;
bool booted = 0;
Stream *serialConsole[1];
int cNum = 0;
int displayPage = 0;
SSD1306 display (0x3c, 4, 15);

long sendLastTime = 0;
long displayLastTime = 0;
long pdLastTime = 0;

msp_analog_t fcanalog; // analog values from FC
msp_status_ex_t fcstatusex; // extended status from FC
msp_raw_gps_t homepos; // set on arm
planeData pd; // our uav data
planeData pdIn; // new air packet
planeData loraMsg; // incoming packet
planesData pds[5]; // uav db
planeData fakepd; // debugging plane
char planeFC[20]; // uav fc name
bool loraRX = 0; // display RX
bool loraTX = 0; // display TX
int numPlanes = 0;
String rssi = "0";
bool buttonState = 1;
bool buttonPressed = 0;
long lastDebounceTime = 0;
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
    //cfg.loraAddress = 2; // local lora address
    cfg.loraFrequency = 433E6; // 433E6, 868E6, 915E6
    cfg.loraBandwidth =  250000;// 250000 bps
    cfg.loraCodingRate4 = 6; // Error correction rate 4/6
    cfg.loraSpreadingFactor = 8; // 7 is shortest time on air - 12 is longest
    cfg.intervalSend = 500; // in ms + random
    cfg.intervalDisplay = 100; // in ms
    cfg.intervalStatus = 1000; // in ms
    cfg.uavTimeout = 8; // in sec
    cfg.fcTimeout = 5; // in sec
    cfg.mspTX = 23; // pin for msp serial TX
    cfg.mspRX = 17; // pin for msp serial RX
    cfg.mspPOI = 1; // POI type: 1 (Wayponit), 2 (Plane) # TODO
    cfg.debugOutput = false;
    cfg.debugFakeWPs = false;
    cfg.debugFakePlanes = false;
    cfg.debugFakeMoving = false;
    cfg.debugGpsLat = 50.100400 * 10000000;
    cfg.debugGpsLon = 8.762835 * 10000000;
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

double deg2rad(double deg) {
  return (deg * M_PI / 180);
}

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
String getValue(String data, char separator, int index) {
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
  serialConsole[n]->println(pd.state ? "ARMED" : "DISARMED");
  serialConsole[n]->print("GPS:              ");
  serialConsole[n]->println(String(pd.gps.numSat) + " Sats");
  serialConsole[n]->println("statusend");
}
void cliHelp(int n) {
  serialConsole[n]->println("================= Commands =================");
  serialConsole[n]->println("status                  - Show whats going on");
  serialConsole[n]->println("help                    - List all commands");
  serialConsole[n]->println("config                  - List all settings");
  serialConsole[n]->println("config loraFreq n       - Set frequency in Hz (e.g. n = 433000000)");
  serialConsole[n]->println("config loraBandwidth n  - Set bandwidth in Hz (e.g. n = 250000)");
  serialConsole[n]->println("config loraSpread n     - Set SF (e.g. n = 10)");
  serialConsole[n]->println("config uavtimeout n     - Set UAV timeout in sec (e.g. n = 10)");
  serialConsole[n]->println("config fctimeout n      - Set FC timeout in sec (e.g. n = 5)");
  serialConsole[n]->println("config debuglat n       - Set debug GPS lat (e.g. n = 50.1004900)");
  serialConsole[n]->println("config debuglon n       - Set debug GPS lon (e.g. n = 8.7632280)");
  serialConsole[n]->println("reboot                  - Reset MCU and radio");
  serialConsole[n]->println("gpspos                  - Show last GPS position");
  //serialConsole[n]->println("fcpass                - start FC passthru mode");
  serialConsole[n]->println("debug                   - Toggle debug output");
  serialConsole[n]->println("localfakeplanes         - Send fake plane to FC");
  serialConsole[n]->println("lfp                     - Send fake plane to FC");
  serialConsole[n]->println("radiofakeplanes         - Send fake plane via radio");
  serialConsole[n]->println("rfp                     - Send fake plane via radio");
  serialConsole[n]->println("movefakeplanes          - Move fake plane");
  serialConsole[n]->println("mfp                     - Move fake plane");
}
void cliConfig(int n) {
  serialConsole[n]->println("=============== Configuration ==============");
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
  serialConsole[n]->print("FC timeout:            ");
  serialConsole[n]->print(cfg.fcTimeout);
  serialConsole[n]->println(" sec");
  serialConsole[n]->print("MSP RX pin:            ");
  serialConsole[n]->println(cfg.mspRX);
  serialConsole[n]->print("MSP TX pin:            ");
  serialConsole[n]->println(cfg.mspTX);
  serialConsole[n]->print("Debug output:          ");
  serialConsole[n]->println(cfg.debugOutput ? "ON" : "OFF");
  serialConsole[n]->print("Debug GPS lat:         ");
  serialConsole[n]->println(cfg.debugGpsLat);
  serialConsole[n]->print("Debug GPS lat:         ");
  serialConsole[n]->println(cfg.debugGpsLon);
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
  cli->addCmd(new Command("lfp", [](Cmd* cmd) { cliLocalFake(cNum); } ));
  cli->addCmd(new Command("rfp", [](Cmd* cmd) { cliRadioFake(cNum); } ));
  cli->addCmd(new Command("mfp", [](Cmd* cmd) { cliMoveFake(cNum); } ));
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
    if (arg1 == "fctimeout") {
      if (arg2.toInt() >= 1 && arg2.toInt() <= 250) {
        cfg.fcTimeout = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("FC timout changed!");
      } else {
        serialConsole[cNum]->println("FC timout not correct: 1 - 250");
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
    if (arg1 == "debuglat") {
      if (arg2.toInt() >= 0) {
        float lat = arg2.toFloat() * 10000000;
        cfg.debugGpsLat = (int32_t) lat;
        saveConfig();
        serialConsole[cNum]->println("Debug GPS lat changed!");
      } else {
        serialConsole[cNum]->println("Debug GPS lat not correct: 50.088251");
      }
    }
    if (arg1 == "debuglon") {
      if (arg2.toInt() >= 0) {
        float lon = arg2.toFloat() * 10000000;
        cfg.debugGpsLon = (int32_t) lon;
        saveConfig();
        serialConsole[cNum]->println("Debug GPS lon changed!");
      } else {
        serialConsole[cNum]->println("Debug GPS lon not correct: 8.783871");
      }
    }
  });
  config->addArg(new AnonymOptArg(""));
  config->addArg(new AnonymOptArg(""));

  cli->addCmd(config);
  //cli->parse("ping");
  //cli->parse("hello");
}
// ----------------------------------------------------------------------------- LoRa
uint8_t loraSeqNum = 0;
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
      //rssi = String(LoRa.packetRssi());
      loraRX = 1;
      pdIn = loraMsg;
      cliLog("New air packet");
      bool found = 0;
      int free = -1;
      for (size_t i = 0; i <= 4; i++) {
        if (String(pds[i].pd.planeName) == String(pdIn.planeName) ) { // update plane
          pds[i].pd = pdIn;
          pds[i].rssi = LoRa.packetRssi();
          pds[i].pd.state = 1;
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
          pds[free].pd.state = 1;
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
  //fakepd.loraAddress = (char)5;
  String("Testplane #1").toCharArray(fakepd.planeName,20);
  fakepd.state=  1;
  fakepd.gps.alt = 300;
  fakepd.gps.groundSpeed = 450;
  fakepd.gps.lat = cfg.debugGpsLat; // + (250 * moving);
  fakepd.gps.lon = cfg.debugGpsLon + (250 * moving);
  sendMessage(&fakepd);
  cliLog("Fake UAVs sent.");
  moving++;
}
void initLora() {
	display.drawString (0, 8, "LORA");
  display.display();
  cliLog("Starting radio ...");
  //pd.loraAddress = cfg.loraAddress;
  String(cfg.loraHeader).toCharArray(pd.header,7);
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(cfg.loraFrequency)) {
    cliLog("Radio start failed!");
    display.drawString (94, 8, "FAIL");
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
  display.drawString (100, 8, "OK");
  display.display();
}
// ----------------------------------------------------------------------------- Display
void drawDisplay () {
  display.clear();
  if (displayPage == 0) {
    display.setFont (ArialMT_Plain_24);
    display.setTextAlignment (TEXT_ALIGN_RIGHT);
    display.drawString (30,5, String(pd.gps.numSat));
    display.drawString (30,30, String(numPlanes));
    display.drawString (114,5, String((float)fcanalog.vbat/10));
    display.drawString (114,30, rssi);
    display.setFont (ArialMT_Plain_10);
    display.setTextAlignment (TEXT_ALIGN_LEFT);
    //if (pd.gps.fixType == 0) display.drawString (33,7, "NF");
    if (pd.gps.fixType == 1) display.drawString (33,7, "2D");
    if (pd.gps.fixType == 2) display.drawString (33,7, "3D");
    display.drawString (33,16, "SAT");
    display.drawString (33,42, "UAV");
    display.drawString (116,16, "V");
    display.drawString (116,42, "dB");
    if (String(planeFC) == String("No FC")) {
      if (cfg.debugOutput) {
        if (cfg.debugFakeWPs) display.drawString (0,54, "LFP");
        if (cfg.debugFakePlanes) display.drawString (30,54, "RFP");
        if (cfg.debugFakeMoving) display.drawString (60,54, "MFP");
      }
    } else {
      //if (bitRead(fcstatusex.armingFlags,17) == 0) display.drawString (0,54, "RC LINK LOST");
      //else
      // TODO
      display.drawString (0,54, pd.planeName);
      //if (fcstatusex.armingFlags != 0) display.drawXbm(61, 54, 8, 8, warnSymbol);
    }
    display.drawString (84,54, "TX");
    display.drawString (106,54, "RX");
    display.drawXbm(98, 55, 8, 8, loraTX ? activeSymbol : inactiveSymbol);
    display.drawXbm(120, 55, 8, 8, loraRX ? activeSymbol : inactiveSymbol);
  }
  if (displayPage == 1) {
    if (numPlanes == 0) display.drawString (0,0, "no UAVs detected ...");
    else {
      display.setFont (ArialMT_Plain_10);
      for (size_t i = 0; i <=2 ; i++) {
        if (pds[i].waypointNumber != 0) {
          display.setTextAlignment (TEXT_ALIGN_LEFT);
          display.drawString (0,i*16, pds[i].pd.planeName);
          display.drawString (0,8+i*16, "RSSI " + String(pds[i].rssi));
          display.setTextAlignment (TEXT_ALIGN_RIGHT);
          display.drawString (128,i*16, String((float)pds[i].pd.gps.lat/10000000,6));
          display.drawString (128,8+i*16, String((float)pds[i].pd.gps.lon/10000000,6));
        }
      }
    }
  }
  if (displayPage == 2) {
    display.setFont (ArialMT_Plain_10);
    for (size_t i = 0; i <=1 ; i++) {
      if (pds[i].waypointNumber != 0) {
        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawString (0,i*16, pds[i+3].pd.planeName);
        display.drawString (0,8+i*16, "RSSI " + String(pds[i].rssi));
        display.setTextAlignment (TEXT_ALIGN_RIGHT);
        display.drawString (65,i*16, String((float)pds[i+3].pd.gps.lat/10000000,6));
        display.drawString (65,8+i*16, String((float)pds[i+3].pd.gps.lon/10000000,6));
      }
    }
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
// ----------------------------------------------------------------------------- MSP and FC
void getPlanetArmed () {
  uint32_t planeModes;
  msp.getActiveModes(&planeModes);
  //Serial.print("Arm State: ");
  //Serial.println(bitRead(planeModes,0));
  pd.state = bitRead(planeModes,0);
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
void getPlaneStatusEx () {
  if (msp.request(MSP_STATUS_EX, &fcstatusex, sizeof(fcstatusex))) {
    cliLog(String(bitRead(fcstatusex.armingFlags,18)));
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
      wp.p2 = pds[i].pd.seqNum;
      wp.p3 = pds[i].pd.state;
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
    }
    if (!cfg.debugFakeMoving) {
      moving = 0;
    }

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
  display.drawString (0, 16, "MSP ");
  display.display();
  delay(200);
  Serial1.begin(57600, SERIAL_8N1, cfg.mspRX , cfg.mspTX);
  msp.begin(Serial1);
  cliLog("MSP started!");
  display.drawString (100, 16, "OK");
  display.display();
  display.drawString (0, 24, "FC ");
  display.display();
  cliLog("Waiting for FC to start ...");
  delay(cfg.fcTimeout*1000);
  getPlaneData();
  //getPlanetArmed();
  getPlaneGPS();
  cliLog("FC detected: " + String(planeFC));
  display.drawString (100, 24, planeFC);
  display.display();
}
// ----------------------------------------------------------------------------- main init

const byte interruptPin = 0;
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  if (buttonPressed == 0) {
    buttonPressed = 1;
    if (displayPage >= 1) displayPage = 0;
    else displayPage++;
    //drawDisplay();
    lastDebounceTime = millis();
  }
  portEXIT_CRITICAL_ISR(&mux);
}


void setup() {
  Serial.begin(115200);
  serialConsole[0] = &Serial;
  initCli();
  initConfig();
  initDisplay();
  initLora();
  delay(1500);
  initMSP();
  delay(1000);
  pinMode(interruptPin, INPUT);
  buttonPressed = 0;
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);

  for (size_t i = 0; i <= 4; i++) {
    //pds[i].pd.loraAddress= 0x00;
    pds[i].waypointNumber = 0;
  }
  booted = 1;
  serialConsole[0]->print("> ");
}

// ----------------------------------------------------------------------------- main loop
void loop() {
  if ( (millis() - lastDebounceTime) > 150 && buttonPressed == 1) buttonPressed = 0;

  if (millis() - displayLastTime > cfg.intervalDisplay) {
    drawDisplay();
    readCli();

    loraTX = 0;
    loraRX = 0;
    displayLastTime = millis();
  }

  if (millis() - pdLastTime > cfg.intervalStatus) {
    numPlanes = 0;
    for (size_t i = 0; i <= 4; i++) {
      if (pds[i].waypointNumber != 0) numPlanes++;
      //if (pd.gps.fixType != 0) pds[i].distance = distanceEarth(pd.gps.lat/10000000, pd.gps.lon/10000000, pds[i].pd.gps.lat/10000000, pds[i].pd.gps.lon/10000000);
      if (pds[i].waypointNumber != 0 && millis() - pds[i].lastUpdate > cfg.uavTimeout*1000 ) { // plane timeout
        pds[i].pd.state = 2;
        planeSetWP();
        planeSetWP();
        pds[i].waypointNumber = 0;
        //pds[i].pd.loraAddress = 0;
        rssi = "0";
        String("").toCharArray(pds[i].pd.planeName,20);
        cliLog("UAV DB delete POI #" + String(i+1));
      }
    }
    //getPlaneStatusEx();
    if (String(pd.planeName) != "No Name" ) {
      getPlaneData();
      getPlanetArmed();
      getPlaneBat();
      if (!pd.state) {
        getPlaneGPS();
        if (pd.gps.fixType != 0) {
          homepos = pd.gps;
          sendMessage(&pd);
          loraTX = 1;
        }
        LoRa.receive();
      }
    }
    pdLastTime = millis();
  }

  if ((millis() - sendLastTime) > cfg.intervalSend ) {
    sendLastTime = millis()+ random(0, 50);
    if (String(pd.planeName) != "No Name" ) {
      if (pd.state) {
        getPlaneGPS();
        loraTX = 1;
        if (pd.gps.fixType != 0) {
          if (loraSeqNum < 255) loraSeqNum++;
          else loraSeqNum = 0;
          pd.seqNum = loraSeqNum;
          sendMessage(&pd);
          LoRa.receive();
        }
      }
      //if (pd.armState) planeSetWP();
      planeSetWP();
      if (cfg.debugFakeWPs) planeFakeWP();
    }
    if (cfg.debugFakePlanes) {
      sendFakePlanes();
      loraTX = 1;
    }
  }
}
