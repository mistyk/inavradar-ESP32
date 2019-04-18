#define VERSION "O.0.9"

#define LORA_INIT   0
#define LORA_START  1
#define LORA_RX     2
#define LORA_TX     3

#define LORA_FREQUENCY 433E6 
#define LORA_BANDWIDTH 250000
#define LORA_CODING_RATE 5
#define LORA_SPREADING_FACTOR 7
#define LORA_POWER 20
#define LORA_CYCLE 500 // ms
#define LORA_CYCLE_SLOTSPACING 100 // ms
#define LORA_CYCLE_TIMING_DELAY -10 // ms
#define LORA_CYCLE_ANTICOLLISION 30 // ms

#define SERIAL_PIN_TX 23
#define SERIAL_PIN_RX 17

#define CYCLE_SCAN 4000 // ms
#define CYCLE_DISPLAY 200 // ms
#define CYCLE_MAIN 1000 // ms

#define LORA_MAXPEERS 8

#define LORA_PEER_TIMEOUT 3000 // ms
#define SERIAL_FC_TIMEOUT 4000 // ms

#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)

#define FC_NONE 0
#define FC_INAV 1

char fc_name[2][5]={"None", "iNav"};

struct peer_t {
  uint8_t id;
  uint8_t state;
  char name[6];
  uint8_t tick;
  uint32_t updated;
  int rssi;
  msp_raw_gps_t gps;
};

struct curr_t {
  uint8_t id;
  uint8_t state;
  char name[16];
  uint8_t tick;
  msp_raw_gps_t gps;
  msp_analog_t vbat;
  uint8_t fc;
};

struct peer_air_t {
  uint8_t id;
  char name[3];
  uint8_t tick;
  int32_t  lat; 
  int32_t  lon;
  int16_t  alt; 
  int16_t  speed;  
  int16_t  heading; 
};

struct stats_t {
  float packets_total;
  uint32_t packets_received;
  uint8_t percent_received;
};
