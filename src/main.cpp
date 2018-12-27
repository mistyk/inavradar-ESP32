#define RADARESP32
//#define RADARATMEGA168

#include <lib/MSP.h>
#include <lib/LoRa.h>
#include <Arduino.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>
#include <main.h>

#ifdef RADARESP32
#include <SSD1306.h>
#include <esp_system.h>
//#include <lib/CLI.h>
#include <SimpleCLI.h>
using namespace simplecli;
#define rxPin 17
#define txPin 23
#endif

#ifdef RADARATMEGA168
#define rxPin 17
#define txPin 23
#endif
//
#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)
#define BAND 868E6 // 915E6

#define CFGVER 2
// ----------------------------------------------------------------------------- global vars
config cfg;
SSD1306 display (0x3c, 4, 15);
BluetoothSerial SerialBT;
MSP msp;
bool booted = 0;

long sendLastTime = 0;
long displayLastTime = 0;
long pdLastTime = 0;

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
  //size_t size = sizeof(cfg);
  //EEPROM.begin(size * 2);
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
    cfg.loraAddress = 1; // local lora address
    cfg.loraFrequency = 868E6; // 433E6, 868E6, 915E6
    cfg.loraBandwidth =  250000;// 250000 bps
    cfg.loraCodingRate4 = 6; // 6?
    cfg.loraSpreadingFactor = 7; // 7?
    cfg.intervalSend = 300; // in ms + random
    cfg.intervalDisplay = 100; // in ms
    cfg.intervalStatus = 1000; // in ms
    cfg.uavTimeout = 10; // in sec
    cfg.mspTX = 23; // pin for msp serial TX
    cfg.mspRX = 17; // pin for msp serial RX
    cfg.mspPOI = 1; // POI type: 1 (Wayponit), 2 (Plane)
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
// ----------------------------------------------------------------------------- CLI
SimpleCLI* cli;

int  serIn;             // var that will hold the bytes-in read from the serialBuffer
char serInString[100];  // array that will hold the different bytes  100=100characters;
                        // -> you must state how long the array will be else it won't work.
int  serInIndx  = 0;    // index of serInString[] in which to insert the next incoming byte
int  serOutIndx = 0;    // index of the outgoing serInString[] array;

void readCli () {

  int sb;
  if(Serial.available()) {
  //Serial.print("reading Serial String: ");     //optional confirmation
    while (Serial.available()){
      sb = Serial.read();
      serInString[serInIndx] = sb;
      serInIndx++;
      Serial.write(sb);
      if (sb == '\n') {
        cli->parse(serInString);
        serInIndx = 0;
        memset(serInString, 0, sizeof(serInString));
        Serial.print("> ");
      }
    }
  }
}

void initCli () {
  // =========== Create CommandParser =========== //
  cli = new SimpleCLI();

   // when no valid command could be found for given user input
   cli->onNotFound = [](String str) {
                         Serial.println("\"" + str + "\" not found");
                     };
   // ============================================ //


   // =========== Add hello command ========== //
   // hello => hello world!
   cli->addCmd(new Command("hello", [](Cmd* cmd) {
       Serial.println("hello world");
   }));
   // ======================================== //


   // =========== Add ping command =========== //
   // ping                 => pong
   // ping -s ponk         => ponk
   // ping -s ponk -n 2    => ponkponk
   Command* ping = new Command("ping", [](Cmd* cmd) {

     Serial.println(cmd->getValue(0));
     Serial.println(cmd->getValue(1));
   });
   ping->addArg(new AnonymOptArg("ping!"));
   ping->addArg(new AnonymOptArg("1"));
   cli->addCmd(ping);
   // ======================================== //


   // run tests
   cli->parse("ping");
   cli->parse("hello");
}


//Cli cli = Cli(Serial);
void cliLog (String log) {
  if (cfg.debugOutput) {
    if (booted) {
      Serial.print("LOG: ");
      Serial.println(log);
    } else {
      Serial.println(log);
    }
  }
}
void cliStatus(void) {
  Serial.println();
  Serial.println("================== Status ==================");
  Serial.print("FC:               ");
  Serial.println(planeFC);
  Serial.print("Name:             ");
  Serial.println(pd.planeName);
  Serial.print("Arm state:        ");
  Serial.println(pd.armState ? "ARMED" : "DISARMED");
  Serial.print("GPS:              ");
  Serial.println(pd.gps.fixType ? String(pd.gps.numSat) + " Sats" : "no fix");

}
void cliHelp(void) {
  Serial.println();
  Serial.println("================= Commands =================");
  Serial.println("status            - Show whats going on");
  Serial.println("help              - List all commands");
  Serial.println("config            - List all settings");
  Serial.println("reboot            - Reset MCU and radio");
  //Serial.println("gpspos            - Show last GPS position");
  //Serial.println("fcpass            - start FC passthru mode");
  Serial.println("debug             - Toggle debug mode");
}
void cliConfig(void) {
  Serial.println();
  Serial.println("=============== Configuration ==============");

}
void cliDebug(void) {
  Serial.println();
  cfg.debugOutput = !cfg.debugOutput;
  saveConfig();
  Serial.println("CLI debugging: " + String(cfg.debugOutput));
}
void cliReboot(void) {
  Serial.println();
  Serial.println("Rebooting ...");
  Serial.println();
  delay(1000);
  ESP.restart();
}
void cliGPS(void) {
  Serial.println();
  Serial.println("================= GPS mode =================");
}
void cliFCpass(void) {
  Serial.println();
  Serial.println("=============== FC passthru ================");
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
      size_t free = 0;
      for (size_t i = 0; i <= 4; i++) {
        if (pds[i].pd.loraAddress == pdIn.loraAddress ) { // update plane
          pds[i].pd = pdIn;
          pds[i].waypointNumber = i+1;
          pds[i].lastUpdate = millis();
          found = 1;
          cliLog("UAV DB update POI #" + String(i+1));
        }
        if (!free && pds[i].waypointNumber == 0) free = i; // find free slot
      }
      if (!found) { // if not there put it in free slot
          pds[free].waypointNumber = free+1;
          pds[free].pd = pdIn;
          pds[free].lastUpdate = millis();
          cliLog("UAV DB new POI #" + String(free+1));
          free = 0;
      }
  }
}
int moving = 0;
void sendFakePlanes () {
  cliLog("Sending fake UAVs via radio ...");
  String("ADS-RC").toCharArray(fakepd.header,7);
  fakepd.loraAddress = (char)1;
  String("Testplane #1").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
  fakepd.gps.lat = 50.1006770 * 10000000 + (1000 * moving);
  fakepd.gps.lon = 8.7613380 * 10000000;
  fakepd.gps.alt = 100;
  fakepd.gps.groundSpeed = 50;
  sendMessage(&fakepd);
  delay(300);
  fakepd.loraAddress = (char)2;
  String("Testplane #2").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
  fakepd.gps.lat = 50.100627 * 10000000 + (1000 * moving);
  fakepd.gps.lon = 8.762765 * 10000000;
  sendMessage(&fakepd);
  delay(300);
  fakepd.loraAddress = (char)3;
  String("Testplane #3").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
  fakepd.gps.lat = 50.100679 * 10000000;
  fakepd.gps.lon = 8.762159 * 10000000 + (1000 * moving);
  sendMessage(&fakepd);
  delay(300);
  fakepd.loraAddress = (char)4;
  String("Testplane #4").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
  fakepd.gps.lat = 50.099836 * 10000000;
  fakepd.gps.lon = 8.762406 * 10000000 + (1000 * moving);
  sendMessage(&fakepd);
  delay(300);
  fakepd.loraAddress = (char)5;
  String("Testplane #5").toCharArray(fakepd.planeName,20);
  fakepd.armState=  1;
  fakepd.gps.lat = 50.1006770 * 10000000 + (1000 * moving);
  fakepd.gps.lon = 8.762406 * 10000000;
  sendMessage(&fakepd);
  delay(300);
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
  if (!LoRa.begin(BAND)) {
    cliLog("Radio start failed!");
    #ifdef RADARESP32
    display.drawString (94, 8, "FAIL");
    #endif
    while (1);
  }
  LoRa.sleep();
  LoRa.setSignalBandwidth(250000);
  LoRa.setCodingRate4(6);
  LoRa.setSpreadingFactor(7);
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
  display.setTextAlignment (TEXT_ALIGN_LEFT);
  display.drawString (0,54, pd.armState ? "Armed" : "Disarmed");
  display.drawString (48,54, pd.gps.fixType ? String(pd.gps.numSat) + " Sat" : "no fix");
  display.drawString (84,54, "TX");
  display.drawString (106,54, "RX");

  display.drawXbm(98, 55, 8, 8, loraTX ? activeSymbol : inactiveSymbol);
  display.drawXbm(120, 55, 8, 8, loraRX ? activeSymbol : inactiveSymbol);

  for (size_t i = 0; i <=4 ; i++) {
    if (pds[i].waypointNumber != 0) display.drawString (0,i*8, pds[i].pd.planeName);
  }
/*
  display.drawString (0,0, "Camille");
  display.drawString (0,8, "Daniel");
  display.setTextAlignment (TEXT_ALIGN_RIGHT);
  display.drawString (128,0, "1,2km A");
  display.drawString (128,8, "5m A");
*/
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
}

void getPlaneGPS () {
  if (msp.request(MSP_RAW_GPS, &pd.gps, sizeof(pd.gps))) {
    //Serial.println(pd.gps.fixType);
    //Serial.println(pd.gps.numSat);
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
      wp.p3 = 0;
      if (i == 4 || pds[i+1].waypointNumber!=0) wp.flag = 0xa5;
      else wp.flag = 0;
      msp.command(MSP_SET_WP, &wp, sizeof(wp));
    }
    else break;
  }
  cliLog("POIs sent to FC.");
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
    wp.waypointNumber = 1;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.100306 * 10000000;
    wp.lon = 8.760833 * 10000000;
    wp.alt = 0;
    wp.p1 = 1000;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0xa5;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));
    cliLog("Fake POIs sent to FC.");
}

void initMSP () {
  cliLog("Starting MSP ...");
  #ifdef RADARESP32
  display.drawString (0, 16, "MSP ");
  display.display();
  #endif
  delay(200);
  Serial1.begin(115200, SERIAL_8N1, rxPin , txPin);
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
  getPlanetArmed();
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
  //SerialBT.begin("ESP32");
  initCli();

  initConfig();
  initDisplay();

  initLora();
  delay(1500);
  initMSP();
  delay(1000);

  for (size_t i = 0; i <= 4; i++) {
    pds[i].pd.loraAddress= 0x00;
  }
  booted = 1;
}
// ----------------------------------------------------------------------------- main loop
void loop() {

  if (millis() - displayLastTime > cfg.intervalDisplay) {
    #ifdef RADARESP32
    drawDisplay();
    readCli();
    #endif
    for (size_t i = 0; i <= 4; i++) {
      if (pds[i].pd.loraAddress != 0 && millis() - pds[i].lastUpdate > cfg.uavTimeout*1000 ) { // plane timeout
        pds[i].waypointNumber = 0;
        pds[i].pd.loraAddress = 0;
        cliLog("UAV DB delete POI #" + String(i+1));
      }
    }
    loraTX = 0;
    loraRX = 0;
    displayLastTime = millis();
  }
  if (millis() - pdLastTime > cfg.intervalStatus) {
    getPlaneData();
    getPlanetArmed();
    if (!pd.armState) {
      loraTX = 1;
      sendMessage(&pd);
      LoRa.receive();
    }
    pdLastTime = millis();
  }

  if (millis() - sendLastTime > cfg.intervalSend + random(0, 20)) {


    if (pd.armState) {
      getPlaneGPS();
      loraTX = 1;
      sendMessage(&pd);
      LoRa.receive();
    }
    if (cfg.debugFakePlanes) sendFakePlanes();
    if (pd.armState) planeSetWP();
    if (cfg.debugFakeWPs) planeFakeWP();
    //planeFakeWPv2();
    sendLastTime = millis();
  }
}
