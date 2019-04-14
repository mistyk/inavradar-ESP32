
#define LORA_INIT   0
#define LORA_PICKUP 1
#define LORA_RX     2
#define LORA_TX     3

struct config {
  char lora_header[7];
  uint32_t lora_frequency;
  uint32_t lora_bandwidth;
  uint8_t lora_coding_rate;
  uint8_t lora_spreading_factor;
  uint8_t lora_power;
  uint8_t lora_pickup_delay;
  uint16_t lora_cycle;
  uint8_t lora_cycle_var; 
  uint16_t display_cycle;
  uint16_t main_cycle;
  uint16_t peer_timeout;
  uint8_t msp_fc_timeout;
  uint8_t msp_tx_pin;
  uint8_t msp_rx_pin;
};

struct peer_t {
  char header[5];
  uint8_t id;
  uint8_t tick;
  char name[5];
  uint8_t state;
  msp_raw_gps_t gps;
};

struct peers_t {
  uint8_t id;
  long updated;
  double rssi;
  peer_t peer;
};


