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
#include <BluetoothSerial.h>
#include <EEPROM.h>

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

#define CFGVER 5 // bump up to overwrite setting with new defaults
// ----------------------------------------------------------------------------- global vars
config cfg;
MSP msp;
bool booted = 0;
bool bton = 0;
Stream *serialConsole[2];
int cNum = 0;
bool dalternate = 1;

#ifdef RADARESP32
SSD1306 display (0x3c, 4, 15);
BluetoothSerial SerialBT;
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
    cfg.loraFrequency = 868E6; // 433E6, 868E6, 915E6
    cfg.loraBandwidth =  250000;// 250000 bps
    cfg.loraCodingRate4 = 6; // Error correction rate 4/6
    cfg.loraSpreadingFactor = 7; // 7 is shortest time on air - 12 is longest
    cfg.intervalSend = 500; // in ms + random
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
// ----------------------------------------------------------------------------- CLI
SimpleCLI* cli;

int  serIn;             // var that will hold the bytes-in read from the serialBuffer
char serInString[100];  // array that will hold the different bytes  100=100characters;
                        // -> you must state how long the array will be else it won't work.
int  serInIndx  = 0;    // index of serInString[] in which to insert the next incoming byte
int  serOutIndx = 0;    // index of the outgoing serInString[] array;

void readCli () {
  for (size_t i = 0; i <= 1; i++) {
    int sb;
    if(serialConsole[i]->available()) {
      while (serialConsole[i]->available()){
        sb = serialConsole[i]->read();
        serInString[serInIndx] = sb;
        serInIndx++;
        serialConsole[i]->write(sb);
        if (sb == '\n') {
          cNum = i;
          cli->parse(serInString);
          serInIndx = 0;
          memset(serInString, 0, sizeof(serInString));
          serialConsole[i]->print("> ");
        }
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
  serialConsole[n]->println(pd.gps.fixType ? String(pd.gps.numSat) + " Sats" : "no fix");
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
  serialConsole[n]->println(" b/s");
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
  cli->addCmd(new Command("debug", [](Cmd* cmd) { cliConfig(cNum); } ));
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
// ----------------------------------------------------------------------------- LoRa
void sendMessage(planeData *outgoing) {
  while (!LoRa.beginPacket()) {  }
  LoRa.write((uint8_t*)outgoing, sizeof(fakepd));
  LoRa.endPacket(false);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;
  LoRa.readBytes((uint8_t *)&loraMsg, packetSize);
  cliLog(loraMsg.header);
  cliLog(cfg.loraHeader);
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
  fakepd.gps.alt = 100;
  fakepd.gps.groundSpeed = 50;
  //sendMessage(&fakepd);
  //delay(300);
  /*
  fakepd.loraAddress = (char)2;
  String("Testplane #2").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1; */
  // -------------------------------------------------------- fixed GPS pos radio fake planes

  fakepd.gps.lat = 50.100627 * 10000000 + (700 * moving);
  fakepd.gps.lon = 8.762765 * 10000000;
  sendMessage(&fakepd);
  // 50.088233, 8.782278 ... 50.088233, 8.785693 ... 341 * 100
  //delay(600);
  //fakepd.loraAddress = (char)3;
  //String("Testplane #3").toCharArray(fakepd.planeName,20);
  //fakepd.armState=  1;
  //fakepd.gps.lat = 50.088233 * 10000000;
  //fakepd.gps.lon = 8.782278 * 10000000 + (341 * moving);
  //sendMessage(&fakepd);
  delay(300);
  fakepd.loraAddress = (char)4;
  String("Testplane #4").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
  fakepd.gps.lat = 50.099836 * 10000000;
  fakepd.gps.lon = 8.762406 * 10000000 + (1000 * moving);
  sendMessage(&fakepd);
  delay(300);/*
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
  String("No FC").toCharArray(pd.planeName,20);
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
      wp.p2 = 0;
      wp.p3 = pds[i].pd.armState;
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
      wp.p3 = 0;
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
  //SerialBT.begin(String(pd.planeName));
  serialConsole[1] = &SerialBT;

  initCli();

  initConfig();
  initDisplay();

  initLora();
  delay(1500);
  initMSP();
  delay(1000);

  for (size_t i = 0; i <= 4; i++) {
    pds[i].pd.loraAddress= 0x00;
    pds[i].waypointNumber = 0;
  }
  booted = 1;
  serialConsole[0]->print("> ");
  serialConsole[1]->print("> ");
}
// ----------------------------------------------------------------------------- main loop
void loop() {

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
        pds[i].waypointNumber = 0;
        pds[i].pd.loraAddress = 0;
        String("").toCharArray(pds[i].pd.planeName,20);
        cliLog("UAV DB delete POI #" + String(i+1));
      }
    }
    getPlaneData();
    dalternate = !dalternate;
    if (String(pd.planeName) != "No FC" ) {
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
    if (pd.armState && bton) {
      bton = 0;
      SerialBT.end();
    }
    if (!pd.armState && !bton) {
      bton = 1;
      SerialBT.begin("INAV-" + String(pd.planeName));
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
