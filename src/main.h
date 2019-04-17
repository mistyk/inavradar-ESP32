#define VERSION "O1"

#define LORA_INIT   0
#define LORA_START  1
#define LORA_RX     2
#define LORA_TX     3

#define LORA_FREQUENCY 433E6 
#define LORA_BANDWIDTH 250000
#define LORA_CODING_RATE 5
#define LORA_SPREADING_FACTOR 7
#define LORA_POWER 20
#define LORA_CYCLE 480 // ms
#define LORA_CYCLE_SLOTSPACING 96

#define SERIAL_PIN_TX 23
#define SERIAL_PIN_RX 17

#define CYCLE_SCAN 4000 // ms
#define CYCLE_DISPLAY 200 // ms
#define CYCLE_MAIN 1000 // ms

#define LORA_MAXPEERS 5
#define LORA_PEER_TIMEOUT 3000 // ms
#define SERIAL_FC_TIMEOUT 3000 // ms

#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)

struct peer_t {
  uint8_t id;
  uint8_t state;
  char name[7];
  uint8_t tick;
  uint32_t updated;
  int16_t rssi;
  msp_raw_gps_t gps;
};

struct peer_air_t {
  uint8_t id;
  char name[7];
  uint8_t tick;
  int32_t  lat; 
  int32_t  lon;
  int16_t  alt; 
  int16_t  speed;  
  int16_t  heading; 
};

