struct config {
  uint8_t configVersion;
  char loraHeader[7]; // protocol identifier
  uint8_t loraAddress; // local lora address
  uint32_t loraFrequency; // 433E6, 868E6, 915E6
  uint32_t loraBandwidth; // 250000 bps
  uint8_t loraCodingRate4; // 6?
  uint8_t loraSpreadingFactor; // 7?
  uint16_t intervalSend; // in ms + random
  uint16_t intervalDisplay; // in ms
  uint16_t intervalStatus; // in ms
  uint16_t uavTimeout; // in sec
  uint8_t fcTimeout; // in sec
  uint8_t mspTX; // pin for msp serial TX
  uint8_t mspRX; // pin for msp serial RX
  uint8_t mspPOI; // POI type: 1 (Wayponit), 2 (Plane)
  bool debugOutput;
  bool debugFakeWPs;
  bool debugFakePlanes;
  bool debugFakeMoving;
  int32_t debugGpsLat; // decimal degrees lat * 10000000
  int32_t debugGpsLon; // decimal degrees lon * 10000000
};

struct planeData {
  char header[7];
  uint8_t seqNum;
  char planeName[20];
  int16_t state;
  msp_raw_gps_t gps;
};
struct planesData {
  uint8_t waypointNumber;
  long lastUpdate;
  double distance;
  double rssi;
  planeData pd;
};
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

const uint8_t warnSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00010000,
    B00101000,
    B00101000,
    B01111100,
    B01101100,
    B11111110
};
