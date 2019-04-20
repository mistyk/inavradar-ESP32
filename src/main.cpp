#include <Arduino.h>
#include <esp_system.h>
#include <lib/MSP.h>
#include <lib/LoRa.h>
#include <SSD1306.h>
#include <EEPROM.h>
#include <main.h>


// ----------------------------------------------------------------------------- global vars

SSD1306 display(0x3c, 4, 15);

curr_t curr; // Our peer
peer_t peers[LORA_MAXPEERS]; // Other peers
peer_air_t incoming; // Received peer
peer_air_t peerout; // Sent peer

MSP msp;
stats_t stats;
int sys_rssi;
uint8_t lora_mode;
uint8_t last_received_id = 0;
char sys_message[20];

// --- Counters

uint8_t sys_pps = 0;
uint8_t sys_ppsc = 0;
int sys_num_peers = 0;

// --- Inputs outputs

uint8_t sys_display_page = 0;
uint32_t io_button_released = 0;
bool io_button_pressed = 0;
bool display_enabled = 1;


// Timing

uint32_t cycle_scan_begin;
uint32_t last_tx_begin;
uint32_t last_tx_end;
uint32_t lora_origin = 0;
uint32_t lora_last_tx = 0;
uint32_t lora_next_tx = 0;
uint32_t lora_last_rx = 0;
int32_t lora_drift = 0;
uint32_t stats_updated = 0;
uint32_t msp_next_cycle = 0;
int msp_step = 0;
uint32_t display_updated = 0;
uint32_t now = 0;


// -------- LoRa

void lora_send(peer_air_t *outgoing) {

    while (!LoRa.beginPacket()) {  }
    LoRa.write((uint8_t*)outgoing, sizeof(peerout));
    LoRa.endPacket(false);

}

void onReceive(int packetSize) {

    if (packetSize == 0) return;
    lora_last_rx = millis();
    LoRa.readBytes((uint8_t *)&incoming, packetSize);
    sys_rssi = LoRa.packetRssi();
    sys_ppsc++;


    uint8_t id = incoming.id - 1;

//  if (String(incoming.name) != String(peers[id].name)) { // Something wrong, id slot / name mismatch
//      peers[incoming.id].state = 3;
//  }

    peers[id].id = incoming.id;
    last_received_id = incoming.id;

    peers[id].host = incoming.host;
    peers[id].updated = lora_last_rx;
    peers[id].rssi = sys_rssi;

    peers[id].gps.lat = incoming.lat;
    peers[id].gps.lon = incoming.lon;
    peers[id].gps.alt = incoming.alt;
    peers[id].gps.groundCourse = incoming.heading;
    peers[id].gps.groundSpeed = incoming.speed;

    peers[id].tick = incoming.tick;

    strncpy(peers[id].name, incoming.name, LORA_NAME_LENGTH);
    peers[id].name[LORA_NAME_LENGTH] = 0;

    stats.timer_end = millis();
    stats.last_rx_duration = stats.timer_end - lora_last_rx;
}

void lora_init() {

    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(SS, RST, DI0);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        display.drawString (94, 9, "FAIL");
        while (1);
    }

    LoRa.sleep();
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setTxPower(LORA_POWER, 1);
    LoRa.setOCP(250);
    LoRa.idle();
    LoRa.onReceive(onReceive);
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

void display_disable() {
    display.displayOff();
    display_enabled = 0;
}

void display_enable() {
    display.displayOn();
    display_enabled = 1;
}

void display_draw() {
    display.clear();

    int j = 0;
    int line;

    if (sys_display_page == 0) {

        display.setFont(ArialMT_Plain_24);
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(26, 0, String(curr.gps.numSat));
        display.drawString(13, 42, String(sys_num_peers));
        // display.drawString(119, 0, String((float)curr.vbat / 10));

        display.setFont(ArialMT_Plain_10);
        display.drawString (126, 23, "_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ ");
        display.drawString(107, 35, String(sys_pps));
        display.drawString(65, 45, String(sys_rssi));
        display.drawString (107, 45, String(stats.percent_received));
        display.drawString (80, 35, String(peerout.tick));

        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawString (60, 21, String(curr.name));
        display.drawString (27, 12, "SAT");
        display.drawString (108, 45, "%E");
        display.drawString (15, 45, "ID");
        display.drawString (28, 45, String(curr.id));
        display.drawString (109, 35, "PS");
        display.drawString (66, 45, "dB");
        display.drawString (0, 21, String(host_name[curr.host]));
//        display.drawString (120,12, "V");

        if (last_received_id > 0) {
            display.drawString (48 + last_received_id * 8, 54, String(last_received_id));
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

        if (curr.gps.fixType == 1) display.drawString (30,3, "2D");
        if (curr.gps.fixType == 2) display.drawString (30,3, "3D");
    }

    else if (sys_display_page == 1) {

        display.setFont (ArialMT_Plain_10);
        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawString (0, 1 , "_____________________");
        display.drawString (3, 4 , ".");
        display.drawString (60, 4 , ".");
        display.drawString (122, 4 , ".");
        display.drawString (0, 14 , String(LORA_CYCLE));
        display.drawString (52, 14 , String(LORA_CYCLE / 2));
        display.drawString (120, 14 , "0");

        long pos[LORA_MAXPEERS];
        now = millis();
        long diff;

        for (int i = 0; i < LORA_MAXPEERS ; i++) {
            if (peers[i].id != 0) {
                diff = now - peers[i].updated;
                if ( diff < LORA_CYCLE) {
                    pos[i] = 120 - 120 * diff / LORA_CYCLE;
                }
            }
            else {
                pos[i] = 0;
            }
        }

        diff = now - lora_last_tx;
        if ( diff < LORA_CYCLE) {
            display.drawString (120 - 120 * diff / LORA_CYCLE, 0, String(curr.id));
        }

        for (int i = 0; i < LORA_MAXPEERS ; i++) {
            if (pos[i] > 0) {
                display.drawString (pos[i], 0, String(peers[i].id));
            }

            if (peers[i].id > 0 && j < 4) {
            line = j * 9 + 24;
                display.setTextAlignment (TEXT_ALIGN_LEFT);
                display.drawString (0, line, String(peers[i].id) + ":");
                display.drawString (16, line, String(peers[i].name));
                display.drawString (56, line, String(host_name[peers[i].host]));
                display.setTextAlignment (TEXT_ALIGN_RIGHT);

                if (lora_last_tx > peers[i].updated) {
                    display.drawString (120, line, String(lora_last_tx - peers[i].updated));
                    display.drawString (128, line, "-");
                }
                else {
                    display.drawString (120, line, String((peers[i].updated) - lora_last_tx));
                    display.drawString (128, line, "+");
                }
            j++;
            }
        }
    }

    else if (sys_display_page == 2) {

        display.setFont (ArialMT_Plain_10);
        display.setTextAlignment (TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "TX TIME");
        display.drawString(0, 10, "RX TIME");
        display.drawString(0, 20, "MSP TIME");
        display.drawString(0, 30, "OLED TIME");
        display.drawString(0, 40, "LORA CYCLE");

        display.drawString(112, 0, "ms");
        display.drawString(112, 10, "ms");
        display.drawString(112, 20, "ms");
        display.drawString(112, 30, "ms");
        display.drawString(112, 40, "ms");

        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString (111, 0, String(stats.last_tx_duration));
        // display.drawString (111, 10, String(stats.last_rx_duration));
        display.drawString (111, 10, "--");
        display.drawString (111, 20, String(stats.last_msp_duration));
        display.drawString (111, 30, String(stats.last_oled_duration));
        display.drawString (111, 40, String(LORA_CYCLE));

        }

    else if (sys_display_page == 3) {
        display.setFont (ArialMT_Plain_10);
        display.setTextAlignment (TEXT_ALIGN_LEFT);


        display.drawString(0, 0, "HOST:");
        display.drawString(0, 10, "GPS LAT");
        display.drawString(0, 20, "GPS LON");
        display.drawString(0, 30, "GPS ALT");
        display.drawString(0, 40, "TICK");

        display.drawString (32, 0, String(curr.name));
        display.drawString (96, 0, String(host_name[curr.host]));

        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString (120, 10, String((float)curr.gps.lat/10000000, 6));
        display.drawString (120, 20, String((float)curr.gps.lon/10000000, 6));
        display.drawString (120, 30, String((float)curr.gps.alt));
        display.drawString (120, 40, String(peerout.tick));
     }
    else if (sys_display_page == 4) {
        display.setFont (ArialMT_Plain_10);
        display.setTextAlignment (TEXT_ALIGN_LEFT);

        int j = 0;
        for (int i = 0; i < LORA_MAXPEERS ; i++) {
            if (peers[i].id > 0 && j < 3) {
                line = j * 20;
                display.setTextAlignment (TEXT_ALIGN_LEFT);
                display.drawString (0, line, String(peers[i].id) + ":");
                display.drawString (13, line, String(peers[i].name));
                display.drawString (13, line + 10, "LQ " + String(peers[i].rssi));
                display.drawString (58, line + 10, String(peers[i].tick));

                display.setTextAlignment (TEXT_ALIGN_RIGHT);
                display.drawString (128, line, String((float)peers[i].gps.lat/10000000,5));
                display.drawString (128, line + 10, String((float)peers[i].gps.lon/10000000,5));

                j++;
            }
        }
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
    if (msp.request(MSP_NAME, &curr.name, sizeof(curr.name))) {
    //
    }
    else {
        for (int i = 0; i < LORA_NAME_LENGTH; i++) {
            curr.name[i] = (char) random(65, 90);
        }
        curr.name[LORA_NAME_LENGTH] = 0;
    }
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
    for (int i = 0; i < LORA_MAXPEERS; i++) {
        if (peers[i].id > 0) {
            radarPos.id = i;
            radarPos.state = peers[i].state;
            radarPos.lat = peers[i].gps.lat;
            radarPos.lon = peers[i].gps.lon;
            radarPos.alt = peers[i].gps.alt;
            radarPos.heading = peers[i].gps.groundSpeed;
            radarPos.speed = peers[i].gps.groundCourse;
            radarPos.tick = peers[i].tick;
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

// -------- SYSTEM

int count_peers() {
    int j = 0;
    for (int i = 0; i < LORA_MAXPEERS; i++) {
        if (peers[i].id > 0) {
            j++;
        }
    }
    return j;
}

// -----------------------------

void setup() {

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

    display.drawString (0, 27, "HOST");
    display.display();
    delay(SERIAL_FC_TIMEOUT);
    msp_set_name();
    msp_set_fc();
    display.drawString (90, 27, String(host_name[curr.host]));
    display.display();

    pinMode(interruptPin, INPUT);
    io_button_pressed = 0;
    attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);

    display.drawString (0, 38, "SCAN");
    display.display();

    for (int i = 0; i < LORA_MAXPEERS; i++) {
        peers[i].id = 0;
    }

    curr.id = 0;
    strncpy(peerout.name, curr.name, LORA_NAME_LENGTH);
    peerout.host = curr.host;
    curr.gps.fixType = 0;
    curr.gps.lat = 0;
    curr.gps.lon = 0;
    curr.gps.alt = 0;
    
    LoRa.sleep();
    LoRa.receive();

    display_updated = 0;
    cycle_scan_begin = millis();

    lora_mode = LORA_INIT;
}

// ----------------------------------------------------------------------------- MAIN LOOP

void loop() {


// ---------------------- IO BUTTON

    if ((millis() > io_button_released + 150) && (io_button_pressed == 1)) {
        io_button_pressed = 0;
    }

// ---------------------- LORA INIT

    if (lora_mode == LORA_INIT) {
        if (millis() > (cycle_scan_begin + CYCLE_SCAN)) {  // End of the scan, set the ID then sync
            for (int i = 0; i < LORA_MAXPEERS; i++) {
                if ((peers[i].id == 0)) {
                    if (curr.id == 0) {
                        curr.id = i + 1;
                        peerout.id = curr.id;
                    }
                }
            }
            lora_mode = LORA_SYNC;
        } else { // Still scanning
            if ((millis() > display_updated + CYCLE_DISPLAY) && display_enabled) {
                for (int i = 0; i < LORA_MAXPEERS; i++) {
                    if (peers[i].id > 0) {
                        display.drawString(40 + peers[i].id * 8, 38, String(peers[i].id));
                    }
                }
                display.drawProgressBar(00, 53, 126, 7, 100 * (millis() - cycle_scan_begin) / CYCLE_SCAN);
                display.display();
                display_updated = millis();
            }
        }
    }

// ---------------------- LORA START

    if (lora_mode == LORA_SYNC) {

        sys_num_peers = count_peers();
        lora_origin = millis();

        if (sys_num_peers == 0) { // Alone, start at will
            lora_next_tx = lora_origin + LORA_CYCLE;
            }
        else { // Not alone, sync by slot

            bool startnow = 0;
            for (int i = 0; (i < LORA_MAXPEERS) && (startnow == 0); i++) {
                if (peers[i].id > 0) {
                   lora_next_tx = peers[i].updated + (curr.id - peers[i].id) * LORA_CYCLE_SLOTSPACING + LORA_CYCLE + LORA_CYCLE_TIMING_DELAY;
                   startnow = 1;
                }
            }

        }
        display_updated = millis();
        stats_updated = millis();
        sys_pps = 0;
        sys_ppsc = 0;
        sys_num_peers = 0;
        stats.packets_total = 0;
        stats.packets_received = 0;
        stats.percent_received = 0;
        peerout.tick = 0;
        lora_mode = LORA_RX;
        }

// ---------------------- LORA RX

    if ((lora_mode == LORA_RX) && (millis() > lora_next_tx)) {
        now = millis();

        while (now > lora_next_tx) { // In  case we skipped some beats
           lora_next_tx += LORA_CYCLE;
        }
        lora_mode = LORA_TX;
    }

// ---------------------- LORA TX

    if (lora_mode == LORA_TX) {

        if ((curr.host != HOST_NONE) && (curr.gps.fixType > 0)) {
            peerout.lat = curr.gps.lat;
            peerout.lon = curr.gps.lon;
            peerout.alt = curr.gps.alt;
            peerout.speed = curr.gps.groundSpeed;
            peerout.heading = curr.gps.groundCourse;
        }
        else {
            peerout.lat = 0;
            peerout.lon = 0;
            peerout.alt = 0;
            peerout.speed = 0;
            peerout.heading = 0;
        }
        
        peerout.tick++;

        stats.last_tx_begin = millis();
        lora_send(&peerout);
        lora_last_tx = millis();
        stats.last_tx_duration = lora_last_tx - stats.last_tx_begin;

//        msp_step = 0;
//        stats.last_msp_duration = 0;
        //msp_next_cycle = lora_last_tx + LORA_CYCLE_SLOTSPACING / 2; // Begin sending MSP half a slot later

        // Drift correction

        if (peers[curr.id].id > 1 && peers[curr.id - 1].id > 0) {
            lora_drift = lora_last_tx - peers[curr.id - 1].updated - LORA_CYCLE_SLOTSPACING;

            if ((abs(lora_drift) > LORA_CYCLE_ANTIDRIFT_THRESHOLD) && (abs(lora_drift) < LORA_CYCLE_SLOTSPACING * 0.5)) {
                lora_next_tx += constrain(lora_drift, -LORA_CYCLE_ANTIDRIFT_CORRECTION, LORA_CYCLE_ANTIDRIFT_CORRECTION);
            }
        }


        if (curr.host != HOST_NONE) {
            stats.timer_begin = millis();
            msp_send_peers();
            msp.request(MSP_RAW_GPS, &curr.gps, sizeof(curr.gps));
            msp_set_state();
            stats.timer_end = millis();
            stats.last_msp_duration = stats.timer_end - stats.timer_begin;
        }

        // Back to RX

        LoRa.sleep();
        LoRa.receive();
        lora_mode = LORA_RX;
    }

// ---------------------- DISPLAY

    if ((millis() > display_updated + CYCLE_DISPLAY) && display_enabled && (lora_mode > LORA_SYNC)) {

        stats.timer_begin = millis();
        display_draw();
        stats.timer_end = millis();
        stats.last_oled_duration = stats.timer_end - stats.timer_begin;
        display_updated = millis();
    }

// ---------------------- SERIAL / MSP


/*
    if ((millis() > msp_next_cycle && msp_step < 3 && (curr.host != HOST_NONE)) && (lora_mode > LORA_SYNC)) {
        stats.timer_begin = millis();

        switch (msp_step) {

        case 0:
            msp_send_peers();
            break;

        case 1:
            msp.request(MSP_RAW_GPS, &curr.gps, sizeof(curr.gps));
            break;

        case 2:
            msp_set_state();
//          if (display_enabled == 1) {
//              msp.request(MSP_ANALOG, &curr.vbat, sizeof(curr.vbat));
//          }
            break;

        }

        stats.timer_end = millis();
        stats.last_msp_duration += stats.timer_end - stats.timer_begin;

        msp_next_cycle += MSP_CYCLE_DELAY;
        msp_step++;
    }
*/


// ---------------------- STATISTICS & IO

    if ((millis() > (CYCLE_STATS + stats_updated)) && (lora_mode > LORA_SYNC)) {

        sys_pps = sys_ppsc;
        sys_ppsc = 0;
        now = millis();

        // Pruning the timed-out peers

        for (int i = 0; i < LORA_MAXPEERS; i++) {
            if (peers[i].id > 0 && ((now - peers[i].updated) > LORA_PEER_TIMEOUT)) {
                peers[i].state = 0;
                peers[i].id = 0;
                sys_rssi = 0;
            }
        }

        sys_num_peers = count_peers();
        stats.packets_total += sys_num_peers * CYCLE_STATS / LORA_CYCLE;
        stats.packets_received += sys_pps;
        stats.percent_received = (stats.packets_received > 0) ? constrain(100 * stats.packets_received / stats.packets_total, 0 ,100) : 0;

        // Screen management

        if ((curr.state == 0) && (display_enabled == 0)) { // Aircraft is disarmed = Turning on the OLED
            display_enable();
        }

        else if ((curr.state == 1) && (display_enabled == 1)) { // Aircraft is armed = Turning off the OLED
            display_disable();
        }

    stats_updated = millis();
    }

}
