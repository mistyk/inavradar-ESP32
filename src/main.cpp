#include <Arduino.h>
#include <esp_system.h>
#include <lib/MSP.h>
#include <lib/LoRa.h>
#include <lib/CRC8.h>
#include <SSD1306.h>
#include <EEPROM.h>
#include <SimpleCLI.h>
//using namespace simplecli;
#include <main.h>

#include <math.h>
#include <cmath>

#define CFGVER 24 // bump up to overwrite setting with new defaults
#define VERSION "A82"
// ----------------------------------------------------------------------------- global vars
config cfg;
MSP msp;
bool booted = 0;
Stream *serialConsole[1];
CRC8 crc8;
uint8_t FLchecksum;
int cNum = 0;
int displayPage = 0;
SSD1306 display (0x3c, 4, 15);

long sendLastTime = 0;
long displayLastTime = 0;
long pdLastTime = 0;
long pickupTime = 0;
long currentUpdateTime = 0;

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
uint8_t loraSeqNum = 0;
int numPlanes = 0;
String rssi = "0";
uint8_t pps = 0;
uint8_t ppsc = 0;
bool buttonState = 1;
bool buttonPressed = 0;
long lastDebounceTime = 0;
bool displayon = 1;
uint8_t loraMode = 0; // 0 init, 1 pickup, 2 RX, 3 TX
// ----------------------------------------------------------------------------- EEPROM / config
void saveConfig () {
  for(size_t i = 0; i < sizeof(cfg); i++) {
    char data = ((char *)&cfg)[i];
    EEPROM.write(i, data);
  }
  EEPROM.commit();
}

// --- Counters

uint8_t sys_pps = 0;
uint8_t sys_ppsc = 0;
int sys_num_peers = 0;
uint8_t sys_lora_sent;

// --- Inputs outputs

uint8_t sys_display_page = 0;
uint32_t io_button_released = 0;
bool io_button_pressed = 0;
bool display_enabled = 1;

// Timing

uint32_t cycle_scan_begin;
uint32_t last_tx_begin;
uint32_t last_tx_end;
uint32_t lora_last_tx = 0;
uint32_t lora_next_tx = 0;
uint32_t lora_last_rx = 0;
int32_t lora_drift = 0;
uint32_t stats_updated = 0;
uint32_t msp_next_cycle = 0;
int msp_step = 0;
uint32_t display_updated = 0;
uint32_t now = 0;
int drift_correction = 0;

// -------- SYSTEM

void set_mode(uint8_t mode) {

    switch (mode) {

    case 0 : // SF9 250
        cfg.lora_frequency = 433E6; // 433E6, 868E6, 915E6
        cfg.lora_bandwidth = 250000;
        cfg.lora_coding_rate = 5;
        cfg.lora_spreading_factor = 9;
        cfg.lora_power = 20;
        cfg.lora_cycle = 500;
        cfg.lora_slot_spacing = 125;
        cfg.lora_timing_delay = -60;
        cfg.lora_antidrift_threshold = 5;
        cfg.lora_antidrift_correction = 5;
        cfg.lora_peer_timeout = 6000;

        cfg.msp_fc_timeout = 7000;
        cfg.msp_after_tx_delay = 85;

        cfg.cycle_scan = 2500;
        cfg.cycle_display = 250;
        cfg.cycle_stats = 1000;
        
        break;

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

void reset_peers() {
    now = millis();
    for (int i = 0; i < LORA_NODES; i++) {
        peers[i].id = 0;
        peers[i].host = 0;
        peers[i].state = 0;
        peers[i].broadcast = 0;
        peers[i].lq_updated = now;
        peers[i].lq_tick = 0;
        peers[i].lq = 0;
        peers[i].updated = 0;
        peers[i].rssi = 0;
        peers[i].distance = 0;
        peers[i].direction = 0;
        peers[i].relalt = 0;
        strcpy(peers[i].name, "");
    }
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

// ----------------------------------------------------------------------------- calc gps distance

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
 * @return The distance between the two points in meters
 */

double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
  double lat1r, lon1r, lat2r, lon2r, u, v;
  lat1r = deg2rad(lat1d);
  lon1r = deg2rad(lon1d);
  lat2r = deg2rad(lat2d);
  lon2r = deg2rad(lon2d);
  u = sin((lat2r - lat1r)/2);
  v = sin((lon2r - lon1r)/2);
  return 2.0 * 6371000 * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

// -------- LoRa

// ----------------------------------------------------------------------------- CLI
SimpleCLI cli;
Command Cmd;
int  serIn;             // var that will hold the bytes-in read from the serialBuffer
char serInString[100];  // array that will hold the different bytes  100=100characters;
                        // -> you must state how long the array will be else it won't work.
int  serInIndx  = 0;    // index of serInString[] in which to insert the next incoming byte
int  serOutIndx = 0;    // index of the outgoing serInString[] array;
/*
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
        cli.parse(serInString);
        serInIndx = 0;
        memset(serInString, 0, sizeof(serInString));
        serialConsole[0]->print("> ");
      }
    }
    else {
        air_0.id = curr.id;
        air_0.type = 0;
        air_0.lat = curr.gps.lat / 100;
        air_0.lon = curr.gps.lon / 100;
        air_0.alt = curr.gps.alt / 100;
        air_0.heading = curr.gps.groundCourse / 10;

        while (!LoRa.beginPacket()) {  }
        LoRa.write((uint8_t*)&air_0, sizeof(air_0));
        LoRa.endPacket(false);
    }
}
*/
void cliLog (String log) {
   //logger.append(log);
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
/*
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
  serialConsole[n]->println("config loraPower n      - Set output power (e.g. n = 0 - 20)");
  serialConsole[n]->println("config uavtimeout n     - Set UAV timeout in sec (e.g. n = 10)");
  serialConsole[n]->println("config fctimeout n      - Set FC timeout in sec (e.g. n = 5)");
  serialConsole[n]->println("config radiointerval n  - Set radio interval in ms (e.g. n = 500)");
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
  serialConsole[n]->print("Lora power:            ");
  serialConsole[n]->println(cfg.loraPower);
  serialConsole[n]->print("UAV timeout:           ");
  serialConsole[n]->print(cfg.uavTimeout);
  serialConsole[n]->println(" sec");
  serialConsole[n]->print("FC timeout:            ");
  serialConsole[n]->print(cfg.fcTimeout);
  serialConsole[n]->println(" sec");
  serialConsole[n]->print("Radio interval:        ");
  serialConsole[n]->print(cfg.intervalSend);
  serialConsole[n]->println(" ms");
  serialConsole[n]->print("MSP RX pin:            ");
  serialConsole[n]->println(cfg.mspRX);
  serialConsole[n]->print("MSP TX pin:            ");
  serialConsole[n]->println(cfg.mspTX);
  serialConsole[n]->print("Debug output:          ");
  serialConsole[n]->println(cfg.debugOutput ? "ON" : "OFF");
  serialConsole[n]->print("Debug GPS lat:         ");
  serialConsole[n]->println((float) cfg.debugGpsLat / 10000000);
  serialConsole[n]->print("Debug GPS lon:         ");
  serialConsole[n]->println((float) cfg.debugGpsLon / 10000000);
  serialConsole[n]->print("Local fake planes:     ");
  serialConsole[n]->println(cfg.debugFakeWPs ? "ON" : "OFF");
  serialConsole[n]->print("Radio fake planes:     ");
  serialConsole[n]->println(cfg.debugFakePlanes ? "ON" : "OFF");
  serialConsole[n]->print("Move fake planes:      ");
  serialConsole[n]->println(cfg.debugFakeMoving ? "ON" : "OFF");
  serialConsole[n]->print("Firmware version:      ");
  serialConsole[n]->println(VERSION);
  serialConsole[n]->println("cfgend");
}
void cliShowLog(int n) {
  //logger.flush();
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
  //cli = new SimpleCLI();
//  cli.onNotFound = [](String str) {
//    Serial.println("\"" + str + "\" not found");
//  };
  cli.addCmd(new Command("status", [](Cmd* cmd) { cliStatus(cNum); } ));
  cli.addCmd(new Command("help", [](Cmd* cmd) { cliHelp(cNum); } ));
  cli.addCmd(new Command("debug", [](Cmd* cmd) { cliDebug(cNum); } ));
  cli.addCmd(new Command("log", [](Cmd* cmd) { cliShowLog(cNum); } ));
  cli.addCmd(new Command("localfakeplanes", [](Cmd* cmd) { cliLocalFake(cNum); } ));
  cli.addCmd(new Command("radiofakeplanes", [](Cmd* cmd) { cliRadioFake(cNum); } ));
  cli.addCmd(new Command("movefakeplanes", [](Cmd* cmd) { cliMoveFake(cNum); } ));
  cli.addCmd(new Command("lfp", [](Cmd* cmd) { cliLocalFake(cNum); } ));
  cli.addCmd(new Command("rfp", [](Cmd* cmd) { cliRadioFake(cNum); } ));
  cli.addCmd(new Command("mfp", [](Cmd* cmd) { cliMoveFake(cNum); } ));
  cli.addCmd(new Command("reboot", [](Cmd* cmd) { cliReboot(cNum); } ));
  cli.addCmd(new Command("gpspos", [](Cmd* cmd) { cliGPSpos(cNum); } ));

  Command* config = new Command("config", [](Cmd* cmd) {
    String arg1 = cmd.getValue(0);
    String arg2 = cmd.getValue(1);
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

    sys_num_peers = count_peers();

    if ((last_received_id == curr.id) && (main_mode > MODE_LORA_SYNC) && !silent_mode) { // Same slot, conflict
        uint32_t cs1 = peers[id].name[0] + peers[id].name[1] * 26 + peers[id].name[2] * 26 * 26 ;
        uint32_t cs2 = curr.name[0] + curr.name[1] * 26 + curr.name[2] * 26 * 26;
        if (cs1 < cs2) { // Pick another slot
            sprintf(sys_message, "%s", "ID CONFLICT");
            pick_id();
            resync_tx_slot(cfg.lora_timing_delay);
        }
    }
    if (arg1 == "loraSpread") {
      if (arg2.toInt() >= 7 && arg2.toInt() <= 12) {
        cfg.loraSpreadingFactor = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora spreading factor changed!");
      } else {
        serialConsole[cNum]->println("Lora spreading factor not correct: 7 - 12");
      }
    }
    if (arg1 == "loraPower") {
      if (arg2.toInt() >= 0 && arg2.toInt() <= 20) {
        cfg.loraPower = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Lora power factor changed!");
      } else {
        serialConsole[cNum]->println("Lora power factor not correct: 0 - 20");
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
    if (arg1 == "radiointerval") {
      if (arg2.toInt() >= 50 && arg2.toInt() <= 1000) {
        cfg.intervalSend = arg2.toInt();
        saveConfig();
        serialConsole[cNum]->println("Radio interval changed!");
      } else {
        serialConsole[cNum]->println("Radio interval not correct: 50 - 1000");
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
      if (1) {
        serialConsole[cNum]->println(arg2);
        float lat = arg2.toFloat() * 10000000;
        cfg.debugGpsLat = (int32_t) lat;
        saveConfig();
        serialConsole[cNum]->println("Debug GPS lat changed!");
      } else {
        serialConsole[cNum]->println("Debug GPS lat not correct: 50.088251");
      }
    }
    if (arg1 == "debuglon") {
      if (1) {
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

  cli.addCmd(config);
  //cli.parse("ping");
  //cli.parse("hello");
}
*/
// ----------------------------------------------------------------------------- Logger
void initLogger () {

}
// ----------------------------------------------------------------------------- LoRa
void sendMessage(planeData *outgoing) {
  if (loraSeqNum < 255) loraSeqNum++;
  else loraSeqNum = 0;
  pd.seqNum = loraSeqNum;
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
      //cliLog(cfg.loraHeader);
      rssi = String(LoRa.packetRssi());
      ppsc++;
      loraRX = 1;
      pdIn = loraMsg;
      cliLog("New air packet");
      uint8_t id = loraMsg.id - 1;
      pds[id].id = loraMsg.id;
      pds[id].pd = loraMsg;
      pds[id].rssi = LoRa.packetRssi();
      pds[id].pd.state = 1;
      pds[id].lastUpdate = millis();
      cliLog("UAV DB update POI #" + String(loraMsg.id));
  }
}

void display_draw() {
    display.clear();

    int j = 0;
    int line;

    if (sys_display_page == 0) {

        display.setFont(ArialMT_Plain_24);
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(26, 11, String(curr.gps.numSat));
        display.drawString(13, 42, String(sys_num_peers));
        display.drawString (125, 11, String(peer_slotname[curr.id]));

        // display.drawString(119, 0, String((float)curr.vbat / 10));

        display.setFont(ArialMT_Plain_10);
        display.drawString (126, 29, "_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ ");
        display.drawString (107, 44, String(stats.percent_received));
        display.drawString(107, 54, String(sys_rssi));

        display.setTextAlignment (TEXT_ALIGN_CENTER);
        display.drawString (64, 0, String(sys_message));

        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawString (60, 12, String(curr.name));
        display.drawString (27, 23, "SAT");
        display.drawString (108, 44, "%E");

        display.drawString(23, 44, String(sys_pps) + "p/s");
        display.drawString (109, 54, "dB");
        display.drawString (60, 23, String(host_name[curr.host]));

        if (last_received_id > 0) {
            display.drawString (36 + last_received_id * 8, 54, String(peer_slotname[last_received_id]));
        }

        if (sys_num_peers == 0) {
            display.drawString (15, 54, "Solo");
            }
        else if (sys_num_peers == 1) {
            display.drawString (15, 54, "Peer");
            }
        else {
            display.drawString (15, 54, "Peers");
            }

        if (curr.gps.fixType == 1) display.drawString (27, 12, "2D");
        if (curr.gps.fixType == 2) display.drawString (27, 12, "3D");
    }

    else if (sys_display_page == 1) {

       display.setFont (ArialMT_Plain_10);
        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawHorizontalLine(0, 11, 128);
        display.drawVerticalLine(0, 9, 6);
        display.drawVerticalLine(64, 11, 4);
        display.drawVerticalLine(127, 9, 6);
        display.drawString (2, 12 , String(cfg.lora_cycle));
        display.drawString (119, 12 , "0");

        long pos[LORA_NODES];
        now = millis();
        long diff;

        for (int i = 0; i < LORA_NODES ; i++) {
            if (peers[i].id != 0 && peers[i].state != 2) {
                diff = now - peers[i].updated;
                if ( diff < cfg.lora_cycle) {
                    pos[i] = 120 - 120 * diff / cfg.lora_cycle;
                }
            }
            else {
                pos[i] = 0;
            }
        }

        diff = now - lora_last_tx;
        if ( diff < cfg.lora_cycle) {
            display.drawString (120 - 120 * diff / cfg.lora_cycle, 0, String(peer_slotname[curr.id]));
        }

        for (int i = 0; i < LORA_NODES ; i++) {
            if (pos[i] > 0) {
                display.drawString (pos[i], 0, String(peer_slotname[peers[i].id]));
            }

            if (peers[i].id > 0 && j < 4) {
            line = j * 9 + 24;
                display.setTextAlignment (TEXT_ALIGN_LEFT);
                display.drawString (0, line, String(peer_slotname[peers[i].id]));
                display.drawString (16, line, String(peers[i].name));
                display.drawString (56, line, String(host_name[peers[i].host]));
                display.setTextAlignment (TEXT_ALIGN_RIGHT);

                if (peers[i].state == 2) { // Peer timed out (L)
                    display.drawString (127, line, "L:" + String((int)((lora_last_tx - peers[i].updated) / 1000)) + "s" );
                }
                else {
                    if (lora_last_tx > peers[i].updated) {
                        display.drawString (119, line, String(lora_last_tx - peers[i].updated));
                        display.drawString (127, line, "-");
                    }
                    else {
                        display.drawString (119, line, String((peers[i].updated) - lora_last_tx));
                        display.drawString (127, line, "+");

                    }
                }
            j++;
            }
        }

    }

    else if (sys_display_page == 2) {

        stats.last_rx_duration = cfg.lora_cycle - stats.last_tx_duration - stats.last_msp_tx_duration - stats.last_msp_rx_duration - stats.last_oled_duration * cfg.lora_cycle / cfg.cycle_display;

        display.setFont (ArialMT_Plain_10);
        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "TX TIME");
        display.drawString(0, 10, "MSP TX TIME");
        display.drawString(0, 20, "MSP RX TIME");
        display.drawString(0, 30, "OLED TIME");
        display.drawString(0, 40, "RX+IDLE TIME");
        display.drawString(0, 50, "LORA CYCLE");

        display.drawString(112, 0, "ms");
        display.drawString(112, 10, "ms");
        display.drawString(112, 20, "ms");
        display.drawString(112, 30, "ms");
        display.drawString(112, 40, "ms");
        display.drawString(112, 50, "ms");

        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString (111, 0, String(stats.last_tx_duration));
        display.drawString (111, 10, String(stats.last_msp_tx_duration));
        display.drawString (111, 20, String(stats.last_msp_rx_duration));
        display.drawString (111, 30, String(stats.last_oled_duration));
        display.drawString (111, 40, String(stats.last_rx_duration));
        display.drawString (111, 50, String(cfg.lora_cycle));


    }
    else if (sys_display_page >= 3) {

        int i = constrain(sys_display_page + 1 - LORA_NODES, 0, LORA_NODES - 1);
        bool iscurrent = (i + 1 == curr.id);

        display.setFont(ArialMT_Plain_24);
        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawString (0, 0, String(peer_slotname[i + 1]));

        display.setFont(ArialMT_Plain_16);
        display.setTextAlignment(TEXT_ALIGN_RIGHT);

        if (iscurrent) {
           display.drawString (128, 0, String(curr.name));
        }
        else {
            display.drawString (128, 0, String(peers[i].name));
        }

        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.setFont (ArialMT_Plain_10);

        if (peers[i].id > 0 || iscurrent) {

            if (peers[i].state == 2 && !iscurrent) { display.drawString (19, 0, "LOST"); }
                else if (peers[i].lq == 0 && !iscurrent) { display.drawString (19, 0, "x"); }
                else if (peers[i].lq == 1) { display.drawXbm(19, 2, 8, 8, icon_lq_1); }
                else if (peers[i].lq == 2) { display.drawXbm(19, 2, 8, 8, icon_lq_2); }
                else if (peers[i].lq == 3) { display.drawXbm(19, 2, 8, 8, icon_lq_3); }
                else if (peers[i].lq == 4) { display.drawXbm(19, 2, 8, 8, icon_lq_4); }

                if (iscurrent) {
                    display.drawString (19, 0, "<HOST>");
                    display.drawString (19, 12, String(host_name[curr.host]));
                }
                else {
                    if (peers[i].state != 2) {
                        display.drawString (38, 0, String(peers[i].rssi) + "db");
                    }
                    display.drawString (19, 12, String(host_name[peers[i].host]));
                }

                if (iscurrent) {
                    display.drawString (50, 12, String(host_state[curr.state]));
                }
                else {
                    display.drawString (50, 12, String(host_state[peers[i].state]));
                }

                display.setTextAlignment (TEXT_ALIGN_RIGHT);

                if (iscurrent) {
                    display.drawString (128, 24, "LA " + String((float)curr.gps.lat / 10000000, 6));
                    display.drawString (128, 34, "LO "+ String((float)curr.gps.lon / 10000000, 6));
                }
                else {
                    display.drawString (128, 24, "LA " + String((float)peers[i].gpsrec.lat / 10000000, 6));
                    display.drawString (128, 34, "LO "+ String((float)peers[i].gpsrec.lon / 10000000, 6));
                }

                display.setTextAlignment (TEXT_ALIGN_LEFT);

                if (iscurrent) {
                    display.drawString (0, 24, "A " + String(curr.gps.alt) + "m");
                    display.drawString (0, 34, "S " + String(peers[i].gpsrec.groundSpeed) + "m/s");
                    display.drawString (0, 44, "C " + String(curr.gps.groundCourse / 10) + "°");
                }
                else {
                    display.drawString (0, 24, "A " + String(peers[i].gpsrec.alt) + "m");
                    display.drawString (0, 34, "S " + String(peers[i].gpsrec.groundSpeed) + "m/s");
                    display.drawString (0, 44, "C " + String(peers[i].gpsrec.groundCourse / 10) + "°");
                }

                if (peers[i].gpsrec.lat != 0 && peers[i].gpsrec.lon != 0 && curr.gps.lat != 0 && curr.gps.lon != 0 && !iscurrent) {
                    peers[i].distance = distanceEarth(curr.gps.lat / 10000000, curr.gps.lon / 10000000, peers[i].gpsrec.lat / 10000000, peers[i].gpsrec.lon / 10000000);
                    peers[i].direction = 0;
                    peers[i].relalt = peers[i].gpsrec.alt - curr.gps.alt;
                }
                else {
                    peers[i].distance = 0;
                    peers[i].direction = 0;
                    peers[i].relalt = 0;
                }

                display.drawString (40, 44, "B " + String(peers[i].direction) + "°");
                display.drawString (88, 44, "D " + String(peers[i].distance) + "m");
                display.drawString (0, 54, "R " + String(peers[i].relalt) + "m");
                if (iscurrent) {
                    display.drawString (40, 54, String((float)curr.fcanalog.vbat / 10) + "v");
                    display.drawString (88, 54, String((int)curr.fcanalog.mAhDrawn) + "mah");
                }

                display.setTextAlignment (TEXT_ALIGN_RIGHT);

            }
        else {
            display.drawString (35, 7, "SLOT IS EMPTY");
        }

    }

    last_received_id = 0;
    sys_message[0] = 0;
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
  display.drawString (0, 8, "FIRMWARE");
  display.drawString (100, 8, VERSION);
  display.display();
  cliLog("Display started!");
}
// ----------------------------------------------------------------------------- MSP and FC
void getPlanetArmed () {
  uint32_t planeModes;
  msp.getActiveModes(&planeModes);
  pd.state = bitRead(planeModes,0);
  cliLog("FC: Arm state " + String(pd.state));
}

void getPlaneGPS () {
  if (msp.request(MSP_RAW_GPS, &pd.gps, sizeof(pd.gps))) {
    cliLog("FC: GPS " + String(pd.gps.fixType));
  }
}

void getPlaneBat () {
  if (msp.request(MSP_ANALOG, &fcanalog, sizeof(fcanalog))) {
    cliLog("FC: Bat " + String(fcanalog.vbat));
  }
}
void getPlaneStatusEx () {
  if (msp.request(MSP_STATUS_EX, &fcstatusex, sizeof(fcstatusex))) {
    cliLog("FC: Status EX " + String(bitRead(fcstatusex.armingFlags,18)));
  }
}
void getPlaneData () {
  for (size_t i = 0; i < 5; i++) {
    pd.planeName[i] = (char) random(65,90);
  }
  if (msp.request(MSP_NAME, &pd.planeName, sizeof(pd.planeName))) {
    cliLog("FC: UAV name " + String(pd.planeName));
  }
  String("No FC").toCharArray(planeFC,20);
  if (msp.request(MSP_FC_VARIANT, &planeFC, sizeof(planeFC))) {
    cliLog("FC: Firmware " + String(planeFC));
  }
}
void planeSetWP () {
  msp_radar_pos_t radarPos;
  for (size_t i = 0; i <= 4; i++) {
    if (pds[i].id != 0) {
//      wp.waypointNumber = pds[i].waypointNumber;
//      wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
      radarPos.id = i;
      radarPos.state = pds[i].pd.state;
      radarPos.lat = pds[i].pd.gps.lat;
      radarPos.lon = pds[i].pd.gps.lon;
      radarPos.alt = pds[i].pd.gps.alt;
      radarPos.speed = pds[i].pd.gps.groundSpeed;
      radarPos.heading = pds[i].pd.gps.groundCourse / 10;
      if (millis() - pds[i].lastUpdate < cfg.intervalSend+100) radarPos.lq =  constrain ( (130 + pds[i].rssi ) , 1 , 100);
      else radarPos.lq = 0;
      msp.command(MSP_SET_RADAR_POS, &radarPos, sizeof(radarPos));
      cliLog("Sent to FC - POI #" + String(i));
    }
  }
}
void planeFakeWPv2 () {
  /*
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
  */
}

void planeFakeWP () {
    msp_set_wp_t wp;
    if (cfg.debugFakeMoving && moving > 100) {
      moving = 0;
    }
    if (!cfg.debugFakeMoving) {
      moving = 0;
    }

 void msp_set_fcanalog() {
  msp.request(MSP_ANALOG, &curr.fcanalog, sizeof(curr.fcanalog));
}

void msp_send_peers() {
    msp_radar_pos_t radarPos;
    for (int i = 0; i < LORA_NODES; i++) {
        if (peers[i].id > 0) {
            radarPos.id = i;
            radarPos.state = peers[i].state;
            radarPos.lat = peers[i].gps.lat;
            radarPos.lon = peers[i].gps.lon;
            radarPos.alt = peers[i].gps.alt;
            radarPos.heading = peers[i].gps.groundCourse;
            radarPos.speed = peers[i].gps.groundSpeed;
            radarPos.lq = peers[i].lq;
            msp.command(MSP_SET_RADAR_POS, &radarPos, sizeof(radarPos));
        }
    }
}

void initMSP () {
  cliLog("Starting MSP ...");
  display.drawString (0, 24, "MSP ");
  display.display();
  delay(200);
  Serial1.begin(57600, SERIAL_8N1, cfg.mspRX , cfg.mspTX);
  msp.begin(Serial1);
  cliLog("MSP started!");
  display.drawString (100, 24, "OK");
  display.display();
  display.drawString (0, 32, "FC ");
  display.display();
  cliLog("Waiting for FC to start ...");
  delay(cfg.fcTimeout*1000);
  getPlaneData();
  getPlaneGPS();
  cliLog("FC detected: " + String(planeFC));
  display.drawString (100, 32, planeFC);
  display.display();
}
// ----------------------------------------------------------------------------- main init

const byte interruptPin = 0;
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterrupt() {
    portENTER_CRITICAL_ISR(&mux);

    if (io_button_pressed == 0) {
        io_button_pressed = 1;

        if (sys_display_page >= 3 + LORA_NODES) {
            sys_display_page = 0;
        }
        else {
            sys_display_page++;
        }

        io_button_released = millis();
    }
    portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  serialConsole[0] = &Serial;
  initLogger();
  //initCli();
  initConfig();
  initDisplay();
  initLora();
  delay(1500);
  crc8.begin();
  initMSP();
  delay(1000);
  pinMode(interruptPin, INPUT);
  buttonPressed = 0;
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
  for (size_t i = 0; i <= 4; i++) {
    pds[i].id = 0;
  }
  booted = 1;
  serialConsole[0]->print("> ");
}

// ----------------------------------------------------------------------------- main loop
void loop() {
  if ( (millis() - lastDebounceTime) > 150 && buttonPressed == 1) buttonPressed = 0;

  if (loraMode == LA_INIT && pd.id == 0) {
    loraMode = LA_PICKUP;
    pickupTime = millis();
  }

  if (loraMode == LA_PICKUP && millis() - pickupTime > cfg.loraPickupTime * 1000) {
    numPlanes = 0;
    for (size_t i = 0; i < sizeof(pds); i++) {
      if (pds[i].id == 0 && pd.id == 0) {
        pd.id = i+1;
        loraMode = LA_RX;
      }
      if (pds[i].id != 0) numPlanes++;
    }
    if (pds[0].id == 1) sendLastTime = pds[0].lastUpdate;
  }

  if (loraMode == LA_RX) {
    if (pd.id == 1 && (millis() - sendLastTime) > cfg.intervalSend) {
      loraMode = LA_TX;
      cliLog("Master TX"  + String(millis() - sendLastTime));
      sendLastTime = millis();

    }
    if (pd.id > 1 && currentUpdateTime != pds[0].lastUpdate && millis() - pds[0].lastUpdate > cfg.intervalSend / 5 * pd.id) {
      loraMode = LA_TX;
      currentUpdateTime = pds[0].lastUpdate;
      cliLog(String(pd.id) + " TX in time "  + String(millis() - sendLastTime));
      sendLastTime = millis();
    } else if (pd.id > 1 && (millis() - sendLastTime) > cfg.intervalSend && currentUpdateTime == pds[0].lastUpdate) {
      loraMode = LA_TX;
      cliLog(String(pd.id) + " TX no time "  + String(millis() - sendLastTime));
      sendLastTime = millis();
    }

  }

  if (displayon && millis() - displayLastTime > cfg.intervalDisplay) {
    drawDisplay();
    //readCli();
    loraTX = 0;
    loraRX = 0;
    displayLastTime = millis();
    if (pd.state == 1) displayArmed();
  }

            display.drawString (90, 27, String(host_name[curr.host]));
            display.drawString (0, 38, "SCAN");
            display.display();

            LoRa.sleep();
            LoRa.receive();

            cycle_scan_begin = millis();
            main_mode = MODE_LORA_INIT;

        } else { // Still scanning
            if ((millis() > display_updated + cfg.cycle_display / 2) && display_enabled) {

                delay(100);
                msp_set_fc();

                display.drawProgressBar(35, 30, 48, 6, 100 * (millis() - cycle_scan_begin) / cfg.msp_fc_timeout);
                display.display();
                display_updated = millis();
            }
        }
    }

// ---------------------- LORA INIT

    if (main_mode == MODE_LORA_INIT) {
        if (millis() > (cycle_scan_begin + cfg.cycle_scan)) {  // End of the scan, set the ID then sync

            sys_num_peers = count_peers();

            if (sys_num_peers >= LORA_NODES || io_button_released > 0) {
                silent_mode = 1;
                sys_display_page = 0;
            }
            else {
                pick_id();
            }
            main_mode = MODE_LORA_SYNC;

        } else { // Still scanning
            if ((millis() > display_updated + cfg.cycle_display / 2) && display_enabled) {
                for (int i = 0; i < LORA_NODES; i++) {
                    if (peers[i].id > 0) {
                        display.drawString(40 + peers[i].id * 8, 38, String(peer_slotname[peers[i].id]));
                    }
                }
                display.drawProgressBar(0, 53, 126, 6, 100 * (millis() - cycle_scan_begin) / cfg.cycle_scan);
                display.display();
                display_updated = millis();
            }
        }
      }
    }
    pdLastTime = millis();
  }

// ---------------------- LORA SYNC

    if (main_mode == MODE_LORA_SYNC) {

        if (sys_num_peers == 0 || silent_mode) { // Alone or silent mode, start at will
            lora_next_tx = millis() + cfg.lora_cycle;
            }
        else { // Not alone, sync by slot
            resync_tx_slot(cfg.lora_timing_delay);
        }
        display_updated = millis();
        stats_updated = millis();

        sys_pps = 0;
        sys_ppsc = 0;
        sys_num_peers = 0;
        stats.packets_total = 0;
        stats.packets_received = 0;
        stats.percent_received = 0;
        stats.up_time_begin = millis();

        main_mode = MODE_LORA_RX;
        }

// ---------------------- LORA RX

    if ((main_mode == MODE_LORA_RX) && (millis() > lora_next_tx)) {

        now = millis();

        while (now > lora_next_tx) { // In  case we skipped some beats
            lora_next_tx += cfg.lora_cycle;
            lora_last_tx = lora_next_tx;
        }

        if (silent_mode) {
            sprintf(sys_message, "%s", "SILENT MODE (NO TX)");
        }
        else {
            main_mode = MODE_LORA_TX;
        }
    }

// ---------------------- LORA TX

    if (main_mode == MODE_LORA_TX) {

        if ((curr.host == HOST_NONE) || (curr.gps.fixType < 1)) {
            curr.gps.lat = 0;
            curr.gps.lon = 0;
            curr.gps.alt = 0;
            curr.gps.groundCourse = 0;
            curr.gps.groundSpeed = 0;
        }

        // peerout.tick++;

        // stats.last_tx_begin = millis();
        lora_last_tx = millis();
        lora_send();
        stats.timer_end = millis();
        stats.last_tx_duration = stats.timer_end - lora_last_tx;

        // Drift correction

        if (curr.id > 1) {
            int prev = curr.id - 2;
            if (peers[prev].id > 0) {
                lora_drift = lora_last_tx - peers[prev].updated - cfg.lora_slot_spacing;

                if ((abs(lora_drift) > cfg.lora_antidrift_threshold) && (abs(lora_drift) < (cfg.lora_slot_spacing * 0.5))) {
                    drift_correction = constrain(lora_drift, -cfg.lora_antidrift_correction, cfg.lora_antidrift_correction);
                    lora_next_tx -= drift_correction;
                    sprintf(sys_message, "%s %3d", "TIMING ADJUST", -drift_correction);
                }
            }
        }

        msp_step = 0;
        msp_next_cycle = stats.last_tx_begin + cfg.msp_after_tx_delay;

        // Back to RX

        LoRa.sleep();
        LoRa.receive();
      }
    }
    //if (pd.armState) planeSetWP();
    if (String(planeFC) == "INAV" ) planeSetWP();
    if (cfg.debugFakeWPs) planeFakeWP();
    if (cfg.debugFakePlanes) {
      sendFakePlanes();
      LoRa.sleep();
      LoRa.receive();
      loraTX = 1;
    }

// ---------------------- SERIAL / MSP


    if (((millis() > msp_next_cycle) && (msp_step < 3) && (curr.host != HOST_NONE)) && (main_mode > MODE_LORA_SYNC)) {

        stats.timer_begin = millis();

        switch (msp_step) {

        case 0: // Between TX slots 0 and 1

            if (sys_lora_sent % 8 == 0) {
                msp_set_fcanalog();
                stats.timer_end = millis();
                stats.last_msp_rx_duration = stats.timer_end - stats.timer_begin;
            }
            break;

        case 1: // Between TX slots 1 and 2
            msp_set_state();
            stats.last_msp_rx_duration = millis()- stats.timer_begin;
            break;

        case 2: // Between TX slots 2 and 3
            msp.request(MSP_RAW_GPS, &curr.gps, sizeof(curr.gps));
            stats.last_msp_rx_duration = millis() - stats.timer_begin;
            break;

        case 3: // Between TX slots 3 and 0
            if (curr.host == HOST_INAV) {
                msp_send_peers();
                stats.last_msp_tx_duration = millis() - stats.timer_begin;
            }
            break;

        }

        msp_next_cycle += cfg.lora_slot_spacing;
        msp_step++;
    }


// ---------------------- STATISTICS & IO

    if ((millis() > (cfg.cycle_stats + stats_updated)) && (main_mode > MODE_LORA_SYNC)) {

        sys_pps = sys_ppsc;
        sys_ppsc = 0;
        now = millis();

        // Pausing the timed-out peers + LQ computation

        for (int i = 0; i < LORA_NODES; i++) {

            if (now > (peers[i].lq_updated +  cfg.lora_cycle * 4)) {
                uint16_t diff = peers[i].updated - peers[i].lq_updated;
                peers[i].lq = constrain(peers[i].lq_tick * 4.4 * cfg.lora_cycle / diff, 0, 4);
                peers[i].lq_updated = now;
                peers[i].lq_tick = 0;
            }

            if (peers[i].id > 0 && ((now - peers[i].updated) > cfg.lora_peer_timeout)) {
                peers[i].state = 2;
                // peers[i].id = 0;
                sys_rssi = 0;
            }

        }

        sys_num_peers = count_peers();
        stats.packets_total += sys_num_peers * cfg.cycle_stats / cfg.lora_cycle;
        stats.packets_received += sys_pps;
        stats.percent_received = (stats.packets_received > 0) ? constrain(100 * stats.packets_received / stats.packets_total, 0 ,100) : 0;

        // Screen management

        if (!curr.state && !display_enabled) { // Aircraft is disarmed = Turning on the OLED
            display.displayOn();
            display_enabled = 1;
        }

        else if (curr.state && display_enabled) { // Aircraft is armed = Turning off the OLED
            display.displayOff();
            display_enabled = 0;
        }

    stats_updated = now;
    }

}
