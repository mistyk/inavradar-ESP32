#define VERSION "1.2"

#define MODE_HOST_SCAN   0
#define MODE_LORA_INIT   1
#define MODE_LORA_SYNC   2
#define MODE_LORA_RX     3
#define MODE_LORA_TX     4

#define LORA_NAME_LENGTH 7

#define SERIAL_PIN_TX 23
#define SERIAL_PIN_RX 17

#define LORA_PERF_MODE 0

#define LORA_NODES_MIN 2
#define LORA_NODES_MAX 4

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
char host_state[2][5]={"", "ARM"};
char peer_slotname[9][3]={"X", "A", "B", "C", "D", "E", "F", "G", "H"};

struct peer_t {
   uint8_t id;
   uint8_t host;
   uint8_t state;
   bool lost;
   uint8_t broadcast;
   uint32_t updated;
   uint32_t lq_updated;
   uint8_t lq_tick;
   uint8_t lq;
   int rssi;
   uint16_t distance;
   int16_t direction;
   int16_t relalt;
   msp_raw_gps_t gps;
   msp_raw_gps_t gpsrec;
   msp_analog_t fcanalog;
   char name[LORA_NAME_LENGTH + 1];
   };

struct curr_t {
    uint8_t id;
    uint8_t state;
    uint8_t host;
    char name[16];
    uint8_t tick;
    msp_raw_gps_t gps;
    msp_fc_version_t fcversion;
    msp_analog_t fcanalog;
};

struct air_type0_t { // 80 bits
    unsigned int id : 3;
    unsigned int type : 3;
    signed int lat : 25; // -9 000 000 to +9 000 000    -90x10e5 to +90x10e5
    signed int lon : 26; // -18 000 000 to +18 000 000   -180x10e5 to +180x10e5
    signed int alt : 14; // -8192m to +8192m
    unsigned int heading : 9; // 0 to 511°
};

struct air_type1_t { // 80 bits
    unsigned int id : 3;
    unsigned int type : 3;
    unsigned int host : 3;
    unsigned int state : 3;
    unsigned int broadcast : 6;
    unsigned int speed : 6; // 64m/s
    char name[LORA_NAME_LENGTH]; // 7 char x 8 bits
    };

struct air_type2_t { // 80 bits
    unsigned int id : 3;
    unsigned int type : 3;
    unsigned int vbat : 8;
    unsigned int mah : 16;
    unsigned int rssi : 10;
    unsigned int temp1 : 20; // Spare
    unsigned int temp2 : 20; // Spare
    };


struct config_t {
    uint32_t lora_frequency;
    uint32_t lora_bandwidth;
    uint8_t lora_coding_rate;
    uint8_t lora_spreading_factor;
    uint8_t lora_power;
    uint8_t lora_nodes_max;
    uint16_t lora_slot_spacing;
    uint16_t lora_cycle;
    int16_t lora_timing_delay;
    uint8_t lora_antidrift_threshold;
    uint8_t lora_antidrift_correction;
    uint16_t lora_peer_timeout;

    uint8_t lora_air_mode;

    uint8_t msp_version;
    uint8_t msp_timeout;
    uint16_t msp_fc_timeout;
    uint16_t msp_after_tx_delay;

    uint16_t cycle_scan;
    uint16_t cycle_display;
    uint16_t cycle_stats;
};

struct system_t {
    uint8_t phase;

    uint32_t now = 0;
    uint32_t now_sec = 0;
    
    uint8_t air_last_received_id = 0;    
    int last_rssi;
    uint8_t pps = 0;
    uint8_t ppsc = 0;
    uint8_t num_peers = 0;
    uint8_t num_peers_active = 0;
    
    uint8_t lora_tick;    
    bool lora_no_tx = 0;
    uint8_t lora_slot = 0;   
    uint32_t lora_last_tx = 0;
    uint32_t lora_last_rx = 0;
    uint32_t lora_next_tx = 0;
    int32_t lora_drift = 0;
    int drift_correction = 0;
     
    uint32_t msp_next_cycle = 0;    
    
    uint8_t display_page = 0;
    bool display_enable = 1;
    uint32_t display_updated = 0;
    
    uint32_t io_button_released = 0;
    bool io_button_pressed = 0;

    uint32_t cycle_scan_begin;    
    uint32_t stats_updated = 0;
    
    char message[20];
};

struct stats_t {
    uint32_t timer_begin;
    uint32_t timer_end;
    float packets_total;
    uint32_t packets_received;
    uint8_t percent_received;
    uint16_t last_tx_duration;
    uint16_t last_rx_duration;
    uint16_t last_msp_0_duration;    
    uint16_t last_msp_1_duration;
    uint16_t last_msp_2_duration;
    uint16_t last_msp_3_duration;        
    uint16_t last_oled_duration;
};

const uint8_t icon_lq_1[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000011
};

const uint8_t icon_lq_2[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00001111,
    B00000000,
    B00000011
};

const uint8_t icon_lq_3[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00111111,
    B00000000,
    B00001111,
    B00000000,
    B00000011
};

const uint8_t icon_lq_4[] PROGMEM = {
    B00000000,
    B11111111,
    B00000000,
    B00111111,
    B00000000,
    B00001111,
    B00000000,
    B00000011
};


