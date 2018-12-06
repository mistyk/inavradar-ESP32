#include <lib/MSP.h>
#include <lib/LoRa.h>
#include <SSD1306.h>
#include <Arduino.h>
#include <esp_system.h>

const uint8_t activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const uint8_t inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};

#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)
#define BAND 868E6 // 915E6

SSD1306 display (0x3c, 4, 15);

#define rxPin 17
#define txPin 23

MSP msp;

int sendInterval = 500;
long sendLastTime = 0;
int displayInterval = 100;
long displayLastTime = 0;

struct planeData {
  String header;
  byte loraAddress;
  char planeName[20];
  bool armState;
  msp_raw_gps_t gps;
};
struct planesData {
  uint8_t waypointNumber;
  long lastUpdate;
  planeData pd;
};
planeData pd;
planeData pdIn;
planesData pds[5];
char planeFC[20];

bool loraRX = 0; // new packet flag
bool loraRXd = 0; // new packet display flag
bool loraTX = 0;
planeData loraMsg;
byte loraAddress = 0x01; // our uniq loraAddress

// ----------------------------------------------------------------------------- String seperator split
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
// ----------------------------------------------------------------------------- LoRa
void sendMessage(planeData *outgoing) {
  while (!LoRa.beginPacket()) {  }
  LoRa.write((uint8_t*)outgoing, sizeof(pd));
  LoRa.endPacket(false);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;
  LoRa.readBytes((uint8_t *)&loraMsg, packetSize);
  Serial.println(loraMsg.planeName);
  if (!loraRX && loraMsg.header == "ADS-RC") { // new plane data
      loraRX = 1;
      pdIn = loraMsg;
  }
}

void initLora() {
	display.drawString (0, 24, "LORA");
  display.display();
  pd.loraAddress = loraAddress;
  pd.header = "ADS-RC";
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    display.drawString (94, 24, "FAIL");
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
  Serial.print("- OK");
  display.drawString (100, 24, "OK");
  display.display();
}
// ----------------------------------------------------------------------------- Display
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
  Serial.print("Display ");
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
  Serial.println("- OK");
}
// ----------------------------------------------------------------------------- MSP and FC
void getPlanetArmed () {
  uint32_t planeModes;
  msp.getActiveModes(&planeModes);
  Serial.print("Arm State: ");
  Serial.println(bitRead(planeModes,0));
}

void getPlaneGPS () {
  if (msp.request(MSP_RAW_GPS, &pd.gps, sizeof(pd.gps))) {
    Serial.println(pd.gps.fixType);
    Serial.println(pd.gps.numSat);
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
  //msp.command(MSP_SET_WP, &wp, sizeof(wp));
}
  void planeFakeWP () {
    msp_set_wp_t wp;
    wp.waypointNumber = 1;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.1006770 * 10000000;
    wp.lon = 8.7613380 * 10000000;
    wp.alt = 500;
    wp.p1 = 0;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));
    wp.waypointNumber = 2;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.1020320 * 10000000;
    wp.lon = 8.7615830 * 10000000;
    wp.alt = 500;
    wp.p1 = 0;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));
    wp.waypointNumber = 3;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.102137 * 10000000;
    wp.lon = 8.762990 * 10000000;
    wp.alt = 500;
    wp.p1 = 0;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));
    wp.waypointNumber = 4;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.100547 * 10000000;
    wp.lon = 8.764052 * 10000000;
    wp.alt = 500;
    wp.p1 = 0;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));

    wp.waypointNumber = 5;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT;
    wp.lat = 50.100306 * 10000000;
    wp.lon = 8.760833 * 10000000;
    wp.alt = 500;
    wp.p1 = 0;
    wp.p2 = 0;
    wp.p3 = 0;
    wp.flag = 0xa5;
    msp.command(MSP_SET_WP, &wp, sizeof(wp));

}

void initMSP () {
  Serial.print("MSP ");
  display.drawString (0, 8, "MSP ");
  display.display();
  delay(200);
  Serial1.begin(115200, SERIAL_8N1, rxPin , txPin);
  msp.begin(Serial1);
  Serial.println("- OK");
  display.drawString (100, 8, "OK");
  display.display();
  Serial.print("FC ");
  display.drawString (0, 16, "FC ");
  display.display();
  delay(2000);
  getPlaneData();
  getPlanetArmed();
  getPlaneGPS();
  Serial.print("- ");
  Serial.println(planeFC);
  display.drawString (100, 16, planeFC);
  display.display();
}
// ----------------------------------------------------------------------------- main init
void setup() {
  Serial.begin(115200);
  initDisplay();
  initLora();
  initMSP();
  delay(2000);
/*
  for (size_t i = 0; i <= 4; i++) {
    pds[i].waypointNumber = i+1;
    pds[i].pd.header= "ADS-RC";
    pds[i].pd.loraAddress= i+1;
    String("Testplane #1").toCharArray(pds[i].pd.planeName,20);
    pds[i].pd.armState= 1;
    pds[i].pd.gps.lat=(50.1006770 * 10000000) + (i*10000);
    pds[i].pd.gps.lon=8.7613380 * 10000000;
    pds[i].pd.gps.alt=500;
    pds[i].pd.gps.groundSpeed=0;
    sendMessage(&pds[i].pd);
    delay(1000);
  }*/
  //planeSetWP();
}
// ----------------------------------------------------------------------------- main loop
void loop() {
  if (loraRX) {
    loraRXd = 1;
    bool found = 0;
    size_t free = 0;
    for (size_t i = 0; i <= 4; i++) {
      if (pds[i].pd.loraAddress == pdIn.loraAddress ) { // update plane
        pds[i].pd = pdIn;
        pds[i].waypointNumber = i+1;
        pds[i].lastUpdate = millis();
        found = 1;
      }
      if (!free && pds[i].waypointNumber == 0) free = i; // find free slot
    }
    if (!found) { // if not there put it in free slot
        pds[free].waypointNumber = free+1;
        pds[free].pd = pdIn;
        pds[free].lastUpdate = millis();
    }
    loraRX = 0;
  }

  if (millis() - displayLastTime > displayInterval) {

    for (size_t i = 0; i <= 4; i++) {
      if (millis() - pds[i].lastUpdate > 500000) { // plane timeout
        pds[i].waypointNumber = 0;
        pds[i].pd.loraAddress = 0;
      }
    }
    drawDisplay();
    loraTX = 0;
    loraRXd = 0;
    displayLastTime = millis();
  }

  if (millis() - sendLastTime > sendInterval) {
    getPlaneGPS();
    loraTX = 1;
    sendMessage(&pd);
    LoRa.receive();

    //planeSetWP();
    planeFakeWP();
    sendLastTime = millis();
  }
}
