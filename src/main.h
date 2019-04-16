
#define LORA_INIT   0
#define LORA_START  1
#define LORA_RX     2
#define LORA_TX     3


struct config {
  char lora_header[3];
  uint32_t lora_frequency;
  uint32_t lora_bandwidth;
  uint8_t lora_coding_rate;
  uint8_t lora_spreading_factor;
  uint8_t lora_power;
  uint16_t cycle_lora;
  uint16_t cycle_lora_slotspacing;  
  uint16_t cycle_scan;   
  uint16_t cycle_display;
  uint16_t cycle_main;
  uint16_t peer_timeout;
  uint8_t msp_fc_timeout;
  uint8_t msp_tx_pin;
  uint8_t msp_rx_pin;
};

/*
struct peer_t {
  char header[5];
  uint8_t id;
  uint8_t state;
  char name[5];
  uint8_t tick;
  msp_raw_gps_t gps;
};

struct peers_t {
  uint8_t id;
  uint32_t updated;
  int16_t rssi;
  peer_t peer;
};
*/

struct peer_t {
  uint8_t id;
  uint8_t state;
  char name[5];
  char header[3];
  uint8_t tick;
  uint32_t updated;
  int16_t rssi;
  msp_raw_gps_t gps;
};

struct peer_air_t {
  uint8_t id;
  char name[5];
//  char header[3];
  uint8_t tick;
  int32_t  lat; 
  int32_t  lon;
  int16_t  alt; 
// int16_t  speed;  
  int16_t  heading; 
};

