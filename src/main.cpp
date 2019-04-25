#include <Arduino.h>
#include <esp_system.h>
#include <lib/MSP.h>
#include <lib/LoRa.h>
#include <SSD1306.h>
#include <EEPROM.h>
#include <main.h>


// ----------------------------------------------------------------------------- global vars

SSD1306 display(0x3c, 4, 15);

config_t cfg;
MSP msp;

curr_t curr; // Our peer
peer_t peers[LORA_NODES]; // Other peers
air_type0_t air_0;
air_type1_t air_1;
air_type1_t * air_r;

stats_t stats;
int sys_rssi;
uint8_t main_mode;
uint8_t last_received_id = 0;
char sys_message[20];

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
}

int count_peers() {
    int j = 0;
    for (int i = 0; i < LORA_NODES; i++) {
        if (peers[i].id > 0) {
            j++;
        }
    }
    return j;
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
        strcpy(peers[i].name, "----");
    }
}

void pick_id() {
    curr.id = 0;
    for (int i = 0; i < LORA_NODES; i++) {
        if ((peers[i].id == 0) && (curr.id == 0)) {
            curr.id = i + 1;
        }
    }
}

void resync_tx_slot(int16_t delay) {
    bool startnow = 0;
    for (int i = 0; (i < LORA_NODES) && (startnow == 0); i++) { // Resync
        if (peers[i].id > 0) {
            lora_next_tx = peers[i].updated + (curr.id - peers[i].id) * cfg.lora_slot_spacing + cfg.lora_cycle + delay;
            startnow = 1;
        }
    }
}

// -------- LoRa

void lora_send() {

    sys_lora_sent++;

    if (sys_lora_sent % 10 == 0) {
        air_1.id = curr.id;
        air_1.type = 1;
        air_1.host = curr.host;
        air_1.state = curr.state;
        air_1.broadcast = 0;
        air_1.speed = curr.gps.groundSpeed / 100;
        strncpy(air_1.name, curr.name, LORA_NAME_LENGTH);

        while (!LoRa.beginPacket()) {  }
        LoRa.write((uint8_t*)&air_1, sizeof(air_1));
        LoRa.endPacket(false);
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

void lora_receive(int packetSize) {

    if (packetSize == 0) return;

    lora_last_rx = millis();
    lora_last_rx -= (stats.last_tx_duration > 0 ) ? stats.last_tx_duration : 0; // RX time should be the same as TX time

    sys_rssi = LoRa.packetRssi();
    sys_ppsc++;

    LoRa.readBytes((uint8_t *)&air_0, packetSize);

    uint8_t id = air_0.id - 1;
    last_received_id = air_0.id;
    peers[id].id = last_received_id;
    peers[id].lq_tick++;
    peers[id].updated = lora_last_rx;
    peers[id].rssi = sys_rssi;

    if (air_0.type == 1) { // Type 1 packet (Speed + host + state + broadcast + name)

        air_r = (air_type1_t*)&air_0;

        peers[id].host = (*air_r).host;
        peers[id].state = (*air_r).state;
        peers[id].broadcast = (*air_r).broadcast;
        peers[id].gps.groundSpeed = (*air_r).speed * 100;
        strncpy(peers[id].name, (*air_r).name, LORA_NAME_LENGTH);
        peers[id].name[LORA_NAME_LENGTH] = 0;

    }
    else { // Type 0 packet (GPS + heading)

        peers[id].gps.lat = air_0.lat * 100;
        peers[id].gps.lon = air_0.lon * 100;
        peers[id].gps.alt = air_0.alt * 100;
        peers[id].gps.groundCourse = air_0.heading * 10;

        if (peers[id].gps.lat != 0 && peers[id].gps.lon != 0) { // Save the last known coordinates
            peers[id].gpsrec.lat = peers[id].gps.lat;
            peers[id].gpsrec.lon = peers[id].gps.lon;
            peers[id].gpsrec.alt = peers[id].gps.alt;
            peers[id].gpsrec.groundCourse = peers[id].gps.groundCourse;
            peers[id].gpsrec.groundSpeed = peers[id].gps.groundSpeed;
        }
    }

    sys_num_peers = count_peers();

    if ((last_received_id == curr.id) && (main_mode > MODE_LORA_SYNC)) { // Same slot, conflict
        uint32_t cs1 = peers[id].name[0] + peers[id].name[1] * 26 + peers[id].name[2] * 26 * 26 ;
        uint32_t cs2 = curr.name[0] + curr.name[1] * 26 + curr.name[2] * 26 * 26;
        if (cs1 < cs2) { // Pick another slot
            sprintf(sys_message, "%s", "ID CONFLICT");
            pick_id();
            resync_tx_slot(cfg.lora_timing_delay);
        }
    }
}

void lora_init() {

    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(SS, RST, DI0);

    if (!LoRa.begin(cfg.lora_frequency)) {
        display.drawString (94, 9, "FAIL");
        while (1);
    }

    LoRa.sleep();
//  LoRa.setSignalBandwidth(cfg.lora_bandwidth, cfg.lora_frequency);
    LoRa.setSignalBandwidth(cfg.lora_bandwidth);
    LoRa.setCodingRate4(cfg.lora_coding_rate);
    LoRa.setSpreadingFactor(cfg.lora_spreading_factor);
    LoRa.setTxPower(cfg.lora_power, 1);
    LoRa.setOCP(250);
    LoRa.idle();
    LoRa.onReceive(lora_receive);
    LoRa.enableCrc();
}

// ----------------------------------------------------------------------------- Display

void display_init() {
    pinMode(16, OUTPUT);
    pinMode(2, OUTPUT);
    digitalWrite(16, LOW);
    delay(50);
    digitalWrite(16, HIGH);
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
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
//        display.drawString (15, 44, "ID");

        display.drawString(23, 44, String(sys_pps) + "p/s");
        display.drawString (109, 54, "dB");
        display.drawString (60, 23, String(host_name[curr.host]));
//        display.drawString (120,12, "V");

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

        display.drawString(0, 0, "HOST:");
        display.drawString(0, 10, "GPS LAT");
        display.drawString(0, 20, "GPS LON");
        display.drawString(0, 30, "GPS ALT");
        display.drawString(0, 40, "HEADING");
        display.drawString(0, 50, "STATE");

        display.drawString (32, 0, String(curr.name));

        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString (128, 0, String(host_name[curr.host]));
        display.drawString (128, 10, String((float)curr.gps.lat/10000000, 6));
        display.drawString (128, 20, String((float)curr.gps.lon/10000000, 6));
        display.drawString (128, 30, String((float)curr.gps.alt));
        display.drawString (128, 40, String((float)curr.gps.groundCourse / 10));
        display.drawString (128, 50, String(host_state[curr.state]));

    }

    else if (sys_display_page == 2) {

        display.setFont (ArialMT_Plain_10);
        display.setTextAlignment (TEXT_ALIGN_LEFT);

        int j = 0;
        for (int i = 0; i < LORA_NODES ; i++) {
            if (peers[i].id > 0 && j < 3) {
                line = j * 20;
                display.setTextAlignment (TEXT_ALIGN_LEFT);
                display.setFont(ArialMT_Plain_16);
                display.drawString (0, line, String(peer_slotname[peers[i].id]));

                display.setFont(ArialMT_Plain_10);
                display.drawString (30, line, String(peers[i].name));

                if (peers[i].state == 2) { display.drawString (14, line, "L"); }
                else if (peers[i].lq == 0) { display.drawString (14, line, "x"); }
                else if (peers[i].lq == 1) { display.drawXbm(16, line + 3, 8, 8, icon_lq_1); }
                else if (peers[i].lq == 2) { display.drawXbm(16, line + 3, 8, 8, icon_lq_2); }
                else if (peers[i].lq == 3) { display.drawXbm(16, line + 3, 8, 8, icon_lq_3); }
                else if (peers[i].lq == 4) { display.drawXbm(16, line + 3, 8, 8, icon_lq_4); }

//              display.drawString (48, line, String(host_name[peers[i].host]));
                display.drawString (16, line + 10, String(peers[i].rssi) + "db");
//              display.drawString (50, line + 10, String(peers[i].gpsrec.groundCourse) + "°");
                display.drawString (53, line + 10, String(peers[i].gpsrec.alt));
//              display.drawString (13, line + 20, "S:" + String(peers[i].gpsrec.groundSpeed));

                display.setTextAlignment (TEXT_ALIGN_RIGHT);
                display.drawString (128, line, String((float)peers[i].gpsrec.lat / 10000000, 5));
                display.drawString (128, line + 10, String((float)peers[i].gpsrec.lon / 10000000, 5));

                j++;
            }
        }
        if (j == 0) {
            display.drawString (0, 0, "NO PEER DETECTED");
        }
    }
    else if (sys_display_page == 3) {

        display.setFont (ArialMT_Plain_10);
        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawHorizontalLine(0, 11, 128);
        display.drawVerticalLine(0, 9, 6);
        display.drawVerticalLine(64, 11, 4);
        display.drawVerticalLine(127, 9, 6);
        display.drawString (2, 12 , String(cfg.lora_cycle));
//        display.drawString (54, 14 , String(cfg.lora_cycle / 2));
        display.drawString (119, 12 , "0");

        long pos[LORA_NODES];
        now = millis();
        long diff;

        for (int i = 0; i < LORA_NODES ; i++) {
            if (peers[i].id != 0) {
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
    else if (sys_display_page == 4) {
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
    last_received_id = 0;
    sys_message[0] = 0;
    display.display();
}

// -------- MSP and FC


void msp_set_state() {
    uint32_t planeModes;
    msp.getActiveModes(&planeModes);
    curr.state = bitRead(planeModes, 0);
}

void msp_set_name() {
    msp.request(MSP_NAME, &curr.name, sizeof(curr.name));
}

void msp_set_fc() {
    char j[5];
    msp.request(MSP_FC_VARIANT, &j, sizeof(j));

    if (strncmp(j, "INAV", 4) == 0) {
        curr.host = HOST_INAV;
    }
    else if (strncmp(j, "BTFL", 4) == 0) {
        curr.host = HOST_BTFL;
    }
    else {
        curr.host = HOST_NONE;
    }
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

// -------- INTERRUPTS

const byte interruptPin = 0;
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterrupt() {
    portENTER_CRITICAL_ISR(&mux);

    if (io_button_pressed == 0) {
        io_button_pressed = 1;

        if (sys_display_page >= 4) {
            sys_display_page = 0;
        }
        else {
            sys_display_page++;
        }

        io_button_released = millis();
    }
    portEXIT_CRITICAL_ISR(&mux);
}


// -----------------------------

void setup() {

    set_mode(LORA_PERF_MODE);

    display_init();

    display.drawString(0, 0, "RADAR VERSION");
    display.drawString(90, 0, VERSION);
    display.display();

    display.drawString (0, 9, "LORA");
    display.display();
    lora_init();
    display.drawString (90, 9, "OK");
    display.display();

    display.drawString (0, 18, "MSP");
    display.display();
    Serial1.begin(115200, SERIAL_8N1, SERIAL_PIN_RX , SERIAL_PIN_TX);
    msp.begin(Serial1);
    display.drawString (90, 18, "OK");
    display.display();

    reset_peers();

    pinMode(interruptPin, INPUT);
    io_button_pressed = 0;
    attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);

    display.drawString (0, 27, "HOST");
    display.display();

    display_updated = 0;
    cycle_scan_begin = millis();

    curr.host = HOST_NONE;

    main_mode = MODE_HOST_SCAN;
}

// ----------------------------------------------------------------------------- MAIN LOOP

void loop() {

// ---------------------- IO BUTTON

    if ((millis() > io_button_released + 150) && (io_button_pressed == 1)) {
        io_button_pressed = 0;
    }

// ---------------------- HOST SCAN

    if (main_mode == MODE_HOST_SCAN) {
            if ((millis() > (cycle_scan_begin + cfg.msp_fc_timeout)) || (curr.host != HOST_NONE)) {  // End of the host scan

            if (curr.host != HOST_NONE) {
                msp_set_name();
            }
            else {
                for (int i = 0; i < 4; i++) {
                curr.name[i] = (char) random(65, 90);
                curr.name[4] = 0;
                }
            }

            curr.gps.fixType = 0;
            curr.gps.lat = 0;
            curr.gps.lon = 0;
            curr.gps.alt = 0;
            curr.id = 0;

            display.drawString (90, 27, String(host_name[curr.host]));
            display.drawString (0, 38, "SCAN");
            display.display();

            LoRa.sleep();
            LoRa.receive();

            cycle_scan_begin = millis();
            main_mode = MODE_LORA_INIT;

        } else { // Still scanning
            if ((millis() > display_updated + cfg.cycle_display / 2) && display_enabled) {

                delay(cfg.cycle_display / 2);
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
            pick_id();
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

// ---------------------- LORA SYNC

    if (main_mode == MODE_LORA_SYNC) {

        sys_num_peers = count_peers();

        if (sys_num_peers == 0) { // Alone, start at will
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
        }
        main_mode = MODE_LORA_TX;
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
        msp_next_cycle = stats.last_tx_begin + cfg.lora_slot_spacing + cfg.msp_after_tx_delay;

        // Back to RX

        LoRa.sleep();
        LoRa.receive();
        main_mode = MODE_LORA_RX;
    }

// ---------------------- DISPLAY

    if ((millis() > display_updated + cfg.cycle_display) && display_enabled && (main_mode > MODE_LORA_SYNC)) {

        stats.timer_begin = millis();
        display_draw();
        stats.timer_end = millis();
        stats.last_oled_duration = stats.timer_end - stats.timer_begin;
        display_updated = millis();
    }

// ---------------------- SERIAL / MSP


    if (((millis() > msp_next_cycle) && (msp_step < 3) && (curr.host != HOST_NONE)) && (main_mode > MODE_LORA_SYNC)) {

        switch (msp_step) {

        case 0:
            msp_set_state();
            break;

        case 1:
            stats.timer_begin = millis();
            msp.request(MSP_RAW_GPS, &curr.gps, sizeof(curr.gps));
            stats.timer_end = millis();
            stats.last_msp_rx_duration = stats.timer_end - stats.timer_begin;
            break;

        case 2:
            stats.timer_begin = millis();
            msp_send_peers();
            stats.timer_end = millis();
            stats.last_msp_tx_duration = stats.timer_end - stats.timer_begin;
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
