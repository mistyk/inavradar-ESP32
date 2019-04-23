#define VERSION "0.91"

#define MODE_HOST_SCAN   0
#define MODE_LORA_INIT   1
#define MODE_LORA_SYNC   2
#define MODE_LORA_RX     3
#define MODE_LORA_TX     4

#define LORA_NAME_LENGTH 4

#define SERIAL_PIN_TX 23
#define SERIAL_PIN_RX 17

#define LORA_MAXPEERS 5

#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)

#define HOST_NONE 0
#define HOST_INAV 1
#define HOST_BTFL 2

char host_name[3][5]={"NoFC", "iNav", "Beta"};

struct peer_t {
   uint8_t id;
   uint8_t state;
   char name[LORA_NAME_LENGTH + 1];
   uint8_t host;
   uint8_t tick;
   uint32_t updated;
   int rssi;
   msp_raw_gps_t gps;
   };

struct curr_t {
    uint8_t id;
    uint8_t state;
    uint8_t host;
    char name[16];
    uint8_t tick;
    msp_raw_gps_t gps;
    msp_analog_t vbat;
};

struct peer_air_t {
    uint8_t id;
    uint8_t host;
    uint8_t tick;
    int32_t lat;
    int32_t lon;
    int16_t alt;
    int16_t speed;
    int16_t heading;
    char name[LORA_NAME_LENGTH];
};

struct stats_t {
    uint32_t timer_begin;
    uint32_t timer_end;
    float packets_total;
    uint32_t packets_received;
    uint8_t percent_received;
    uint16_t last_tx_begin;
    uint16_t last_tx_duration;
    uint16_t last_rx_duration;
    uint16_t last_msp_tx_duration;
    uint16_t last_msp_rx_duration;
    uint16_t last_oled_duration;
};

struct config_t {
    uint32_t lora_frequency;
    uint32_t lora_bandwidth;
    uint8_t lora_coding_rate;
    uint8_t lora_spreading_factor;
    uint8_t lora_power;
    uint16_t lora_cycle;
    uint16_t lora_slot_spacing;
    int16_t lora_timing_delay;
    uint8_t lora_antidrift_threshold;
    uint8_t lora_antidrift_correction;
    uint16_t lora_peer_timeout;

    uint16_t msp_cycle_delay;
    uint16_t msp_fc_timeout;

    uint16_t cycle_scan;
    uint16_t cycle_display;
    uint16_t cycle_stats;
};
