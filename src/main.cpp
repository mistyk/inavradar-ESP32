#include <Arduino.h>
#include <esp_system.h>
#include <lib/MSP.h>
#include <lib/LoRa.h>
#include <SSD1306.h>
#include <EEPROM.h>
#include <main.h>

#define SCK 5 // GPIO5 - SX1278's SCK
#define MISO 19 // GPIO19 - SX1278's MISO
#define MOSI 27 // GPIO27 - SX1278's MOSI
#define SS 18 // GPIO18 - SX1278's CS
#define RST 14 // GPIO14 - SX1278's RESET
#define DI0 26 // GPIO26 - SX1278's IRQ (interrupt request)

#define VERSION "Z1"
#define LORA_MAXPEERS 5

// ----------------------------------------------------------------------------- global vars
config cfg;
MSP msp;

// Stream *serialConsole[1];

int sys_display_page = 0;
SSD1306 display(0x3c, 4, 15);

long sys_pickup_updated = 0;

long lora_last_tx = 0;
long main_updated = 0;
long currentUpdateTime = 0;

msp_analog_t sys_curr_vbat;
char sys_curr_fc[5];

peer_t curr; // Our peer
peer_t peers[5]; // Other peers
peer_air_t incoming; // Received
peer_air_t j; // Sent

int sys_tx_ticker  = 0;
int sys_num_peers = -1;

String sys_rssi = "0";

uint8_t sys_pps = 0;
uint8_t sys_ppsc = 0;

bool io_button_state = 1; // Not needed ?
bool io_button_pressed = 0;
long io_button_released = 0;

bool display_enabled = 1;
long display_updated = 0;

long cycle_scan_begin; 
uint8_t scan_last_peers = -1;

int lora_mode = LORA_INIT;

int last_received_id = 0;
char last_received_name[5];

// ----------------------------------------------------------------------------- EEPROM / config

void config_init() {

    String("R01").toCharArray(cfg.lora_header, 3); // protocol identifier
    cfg.lora_frequency = 433E6; // 433E6, 868E6, 915E6
    cfg.lora_bandwidth =  250000;// KHz
    cfg.lora_coding_rate = 5; // Error correction rate
    cfg.lora_spreading_factor = 8;
    cfg.lora_power = 20; //17 PA OFF, 18-20 PA ON

    cfg.cycle_lora = 480; // ms
    cfg.cycle_lora_slotspacing = 96; // ms
    cfg.cycle_scan = 4000; // ms
    cfg.cycle_display = 150; // ms
    cfg.cycle_main = 1000; // ms

    cfg.peer_timeout = 3; // sec
    cfg.msp_fc_timeout = 2; // sec

    cfg.msp_tx_pin = 23; // pin for MSP serial TX
    cfg.msp_rx_pin = 17; // pin for MSP serial RX
}


// ----------------------------------------------------------------------------- LoRa

void lora_send(peer_air_t *outgoing) {

    while (!LoRa.beginPacket()) {  }
    LoRa.write((uint8_t*)outgoing, sizeof(outgoing));
    LoRa.endPacket(false);

}



void onReceive(int packetSize) {
    if (packetSize == 0) return;
    LoRa.readBytes((uint8_t *)&incoming, packetSize);

  //  if (String(incoming.header) == String(cfg.lora_header)) { // New data from a peer

        sys_rssi = LoRa.packetRssi();
        sys_ppsc++;
/*
        if (String(incoming.name) == peers[incoming.id].name) { // Known peer
            peers[incoming.id].state = 1;
        }
        else { // Something wrong, id slot / name mismatch
            peers[incoming.id].state = 1; // TODO, for now overwrite
        }
*/
        uint8_t id = incoming.id - 1;
        peers[id].id = incoming.id;
        last_received_id = incoming.id;
        
        peers[id].name[0] = incoming.name[0];
        peers[id].name[1] = incoming.name[1];
        peers[id].name[2] = incoming.name[2];
        peers[id].name[3] = incoming.name[3];
        peers[id].name[4] = incoming.name[4];
/*        
        peers[id].header[0] = incoming.header[0];
        peers[id].header[1] = incoming.header[1];        
        peers[id].header[2] = incoming.header[2];
*/        
        peers[id].tick = incoming.tick;
        peers[id].updated = millis();
        // peers[id].rssi = sys_rssi;
        peers[id].gps.lat = incoming.lat;
        peers[id].gps.lon = incoming.lon;
        peers[id].gps.alt = incoming.alt;

        peers[id].gps.lat = 0;
        peers[id].gps.lon = 0;
        peers[id].gps.alt = 0;

//      peers[id].gps.groundSpeed = incoming.speed;
        peers[id].gps.groundCourse = incoming.heading;
 //   }
}


/*
void onReceive(int packetSize) {
    if (packetSize == 0) return;
    LoRa.readBytes((uint8_t *)&incoming, packetSize);

    //if (String(incoming.header) == String(cfg.lora_header)) { // new plane data

        sys_rssi = String(LoRa.packetRssi());
        sys_ppsc++;


        uint8_t id = incoming.id - 1;
        peers[id].id = incoming.id;

        peers[id].rssi = LoRa.packetRssi();
        peers[id].state = 1;
        peers[id].updated = millis();
//}
}

*/


void lora_init() {
    display.drawString (0, 16, "LORA");
    display.display();

    String(cfg.lora_header).toCharArray(curr.header,5);
    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(SS, RST, DI0);

    if (!LoRa.begin(cfg.lora_frequency)) {
        display.drawString (94, 8, "FAIL");
        while (1);
    }

    LoRa.sleep();
    LoRa.setSignalBandwidth(cfg.lora_bandwidth);
    LoRa.setCodingRate4(cfg.lora_coding_rate);
    LoRa.setSpreadingFactor(cfg.lora_spreading_factor);
    LoRa.setTxPower(cfg.lora_power, 1);
//    LoRa.setTxPower(cfg.lora_power);
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
    display.clear();
    display.setFont (ArialMT_Plain_24);
    display.setTextAlignment (TEXT_ALIGN_LEFT);
    display.drawString (20, 20, "Armed");
    display.display();
    delay(250);
    display.clear();
    display.display();
    display_enabled = 0;
}

void display_enable() {
    display.clear();
    display.setFont (ArialMT_Plain_24);
    display.setTextAlignment (TEXT_ALIGN_LEFT);
    display.drawString (20,20, "Disarmed");
    display.display();
    delay(250);
    display_enabled = 1;
}

void display_draw() {
    display.clear();
    if (sys_display_page == 0) {
        display.setFont (ArialMT_Plain_24);
        display.setTextAlignment (TEXT_ALIGN_RIGHT);
        display.drawString (30,5, String(curr.gps.numSat));
        display.drawString (30,30, String(sys_num_peers));
        display.drawString (114,5, String((float)sys_curr_vbat.vbat/10));
        display.setFont (ArialMT_Plain_10);
        display.drawString (110,30, String(sys_pps));
        display.drawString (110,42, sys_rssi);

display.drawString (60,54, String(sys_tx_ticker));

display.drawString (85 + last_received_id * 8, 54, String(last_received_id));


        display.setTextAlignment (TEXT_ALIGN_LEFT);
        if (curr.gps.fixType == 1) display.drawString (33,7, "2D");
        if (curr.gps.fixType == 2) display.drawString (33,7, "3D");
        display.drawString (33,16, "SAT");
        display.drawString (33,34, "ID");
        display.drawString (49,34, String(curr.id));
        display.drawString (33,42, "UAV");
        display.drawString (116,16, "V");
        display.drawString (112,30, "Hz");
        display.drawString (112,42, "dB");

        if (String(sys_curr_fc) != String("NOFC")) {
            display.drawString (0,54, String(sys_curr_fc));
        }
//    display.drawString (84, 54, display_lora_tx ? "TX" : "  ");
//    display.drawString (106, 54, display_lora_rx ? "RX" : "  ");
    }

    else if (sys_display_page == 1) {
        if (sys_num_peers == 0) {
            display.drawString (0, 0, "No UAV detected");
            }
        else {
            display.setFont (ArialMT_Plain_10);
            for (int i = 0; i < 3 ; i++) {
                if (peers[i].id != 0) {
                    display.setTextAlignment (TEXT_ALIGN_LEFT);
                    display.drawString (0, i * 16, String(peers[i].id) + ":");
                    display.drawString (20, i * 16, peers[i].name);
                    display.drawString (0, 8 + i * 16, "RSSI " + String(peers[i].rssi));
                    display.setTextAlignment (TEXT_ALIGN_RIGHT);
                    display.drawString (128, i * 16, String((float)peers[i].gps.lat/10000000, 6));
                    display.drawString (128, 8 + i * 16, String((float)peers[i].gps.lon/10000000, 6));
                }
            }
        }
    }

    else if (sys_display_page == 2) {
        display.setFont (ArialMT_Plain_10);
        for (size_t i = 0; i <=1 ; i++) {
            if (peers[i].id != 0) {
                display.setTextAlignment (TEXT_ALIGN_LEFT);
                display.drawString (0, i*16, peers[i+3].name);
                display.drawString (0, 8 + i * 16, "RSSI " + String(peers[i].rssi));
                display.setTextAlignment (TEXT_ALIGN_RIGHT);
                display.drawString (65, i * 16, String((float)peers[i+3].gps.lat/10000000, 6));
                display.drawString (65, 8 + i * 16, String((float)peers[i+3].gps.lon/10000000, 6));
            }
        }
    }
    display.display();
}




// ----------------------------------------------------------------------------- MSP and FC



void msp_set_curr_state() {
    uint32_t planeModes;
    msp.getActiveModes(&planeModes);
    curr.state = bitRead(planeModes, 0);
}

void msp_set_curr_gps() {
    msp.request(MSP_RAW_GPS, &curr.gps, sizeof(curr.gps));
}

void msp_set_curr_vbat() {
    msp.request(MSP_ANALOG, &sys_curr_vbat, sizeof(sys_curr_vbat));
}


void msp_set_curr_name() {
    /*
    char buff[16];

    msp.request(MSP_NAME, &buff, sizeof(curr.name));
    
    if (String(buff) != "") {
        for (int i = 0; i < 5; i++) {
            curr.name[i] = buff[i]; // ------
        }
    } else {
*/
    for (int i = 0; i < 5; i++) {
            curr.name[i] = (char) random(65, 90);
        }
  //  }
}

void msp_set_curr_fc() {
    if (!msp.request(MSP_FC_VARIANT, &sys_curr_fc, sizeof(sys_curr_fc))) {
        String("NOFC").toCharArray(sys_curr_fc, 5);
    }
}



void msp_send_peers() {
    msp_radar_pos_t radarPos;
    for (int i = 0; i < LORA_MAXPEERS; i++) {
        if (peers[i].id != 0) {
            radarPos.id = i;
            radarPos.state = peers[i].state;
            radarPos.lat = peers[i].gps.lat;
            radarPos.lon = peers[i].gps.lon;
            radarPos.alt = peers[i].gps.alt;
            radarPos.speed = peers[i].gps.groundSpeed;
            radarPos.heading = peers[i].gps.groundCourse / 10;
            msp.command(MSP_SET_RADAR_POS, &radarPos, sizeof(radarPos));
        }
    }
}

void msp_init() {

    display.drawString (0, 24, "MSP ");
    display.display();
    delay(100);
    Serial1.begin(115200, SERIAL_8N1, cfg.msp_rx_pin , cfg.msp_tx_pin);
    msp.begin(Serial1);

    display.drawString (100, 24, "OK");
    display.display();
    display.drawString (0, 32, "FC ");
    display.display();

    delay(cfg.msp_fc_timeout * 1000);
    msp_set_curr_name();
    msp_set_curr_fc();
//    msp_set_curr_gps();

    display.drawString (100, 32, sys_curr_fc);
    display.display();
}

// ----------------------------------------------------------------------------- main init

const byte interruptPin = 0;
volatile int interruptCounter = 0;
int numberOfInterrupts = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR handleInterrupt() {
    portENTER_CRITICAL_ISR(&mux);

    if (io_button_pressed == 0) {
        io_button_pressed = 1;

        if (sys_display_page >= 1) {
            sys_display_page = 0;
        }
        else {
            sys_display_page++;
        }

        io_button_released = millis();
    }
    portEXIT_CRITICAL_ISR(&mux);
}

void setup() {

    config_init();

    display_init();
    display.drawString(0, 8, "FIRMWARE");
    display.drawString(100, 8, VERSION);
    display.display();

    lora_init();

    display.drawString (100, 16, "OK");
    display.display();    
    
    msp_init();
    delay(500);

    pinMode(interruptPin, INPUT);
    io_button_pressed = 0;
    attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);

   
    display.drawString (0, 40, "SCAN");
    display.display();

    for (int i = 0; i < LORA_MAXPEERS; i++) {
        peers[i].id = 0;
    }

    curr.id = 0;    
    
    lora_last_tx = millis();
    main_updated = millis();

    display_updated = millis();
    cycle_scan_begin = millis();

    LoRa.sleep();
    LoRa.receive();
    
    lora_mode = LORA_INIT;
    
}

// ----------------------------------------------------------------------------- MAIN LOOP

void loop() {


// ---------------------- IO BUTTON

    if (((millis() - io_button_released) > 150) && (io_button_pressed == 1)) {
        io_button_pressed = 0;
    }

// ---------------------- LORA INIT

    if (lora_mode == LORA_INIT) {

        if (millis() > (cycle_scan_begin + cfg.cycle_scan)) {

            for (int i = 0; i < LORA_MAXPEERS; i++) {
                if ((peers[i].id == 0)) {
                    if (curr.id == 0) {
                        curr.id = i + 1;
                    }
                } else {
                    sys_num_peers++;
                }
            }
            lora_mode = LORA_START;
        } else {
           // delay (20);
            
            sys_num_peers = 0;
            for (int i = 0; i < LORA_MAXPEERS; i++) {
                if (peers[i].id > 0) {
                sys_num_peers++;
                }
            }
            
            if (sys_num_peers > scan_last_peers) {
                display.drawString (40 + sys_num_peers * 8, 40, String(sys_num_peers));
                display.display();
                scan_last_peers = sys_num_peers;
            }
            
        }
    }

// ---------------------- LORA START

    if (lora_mode == LORA_START) {

        if (sys_num_peers == 0 ) { // Alone, start at will
            lora_last_tx = millis();
        }
        else { // Not alone, sync by slot

        int gogogo = 0;
            for (int i = 0; (i < LORA_MAXPEERS) && (gogogo == 0); i++) {
                if (peers[i].id > 0) {
                lora_last_tx = peers[i].updated + (curr.id - peers[i].id) * cfg.cycle_lora_slotspacing;
                gogogo = 1;
                }
            }

        }
        display_updated = millis();
        main_updated = millis();
        lora_mode = LORA_RX;

        }


// ---------------------- LORA RX

        if ((lora_mode == LORA_RX) && (millis() > (lora_last_tx + cfg.cycle_lora))) {
            lora_mode = LORA_TX;
        }


// ---------------------- LORA TX

    if (lora_mode == LORA_TX) {



//      if (curr.gps.fixType > 0) {

        if (String(sys_curr_fc) == "INAV" ) {
            msp_set_curr_gps();
        }
//        else {
 //           curr.gps.fixType = 0;
  //      }


    if (sys_tx_ticker < 255) {
        sys_tx_ticker++;
    }
    else {
        sys_tx_ticker = 0;
    }

    j.id = curr.id;
    j.name[0] = curr.name[0];
    j.name[1] = curr.name[1];
    j.name[2] = curr.name[2];
    j.name[3] = curr.name[3];
    j.name[4] = curr.name[4];
/*    
    j.header[0] = cfg.lora_header[0];
    j.header[1] = cfg.lora_header[1];
    j.header[2] = cfg.lora_header[2];
*/    
    j.tick = sys_tx_ticker;
    j.lat = curr.gps.lat;
    j.lon = curr.gps.lon;
    j.alt = curr.gps.alt;
    j.heading = curr.gps.groundCourse;
 
           lora_send(&j);
           lora_last_tx = millis();
           lora_mode = LORA_RX;
//      }

        if (String(sys_curr_fc) == "INAV" ) {
            msp_send_peers();
        }

        LoRa.sleep();
        LoRa.receive();
        lora_mode = LORA_RX;
    }


    if (((millis() - display_updated) > cfg.cycle_display) && display_enabled && (lora_mode > LORA_START)) {
        display_draw();
        display_updated = millis();
    }


    if ((millis() > (cfg.cycle_main + main_updated)) && (lora_mode > LORA_START)) {
        sys_num_peers = 0;
        sys_pps = sys_ppsc;
        sys_ppsc = 0;

        for (int i = 0; i < (LORA_MAXPEERS - 1); i++) {
            if (peers[i].id > 0) {
                sys_num_peers++;
            }

            if (peers[i].id > 0 && ((millis() - peers[i].updated) > cfg.peer_timeout * 1000)) { // Peer timeout
                peers[i].state = 0;
                // msp_send_peers();
                peers[i].id = 0;
                sys_rssi = "0";
                String("").toCharArray(peers[i].name, 5);
            }
        }

        if (String(sys_curr_fc) == "INAV" ) {
            msp_set_curr_state();
            msp_set_curr_vbat();
        }

        if ((curr.state == 0) && (display_enabled == 0)) { // Aircraft is disarmed = Turning on the OLED
            display_enable();
        }

        else if ((curr.state == 1) && (display_enabled == 1)) { // Aircraft is armed = Turning off the OLED
            display_disable();
        }

    main_updated = millis();
    }


}
