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
peer_t incoming; // Incoming peer
peers_t peers[5]; // Other peers

int sys_msp_ticker  = 0;
int sys_num_peers = 0;

String sys_rssi = "0";

uint8_t sys_pps = 0;
uint8_t sys_ppsc = 0;

bool io_button_state = 1; // Not needed ?
bool io_button_pressed = 0;
long io_button_released = 0;

bool display_enabled = 1;
long display_updated = 0;

uint8_t lora_mode = 0; // 0 init, 1 pickup, 2 RX, 3 TX
uint16_t lora_cycle_eff;

// ----------------------------------------------------------------------------- EEPROM / config

void config_init() {

    String("IRP2").toCharArray(cfg.lora_header,5); // protocol identifier
    cfg.lora_frequency = 433E6; // 433E6, 868E6, 915E6
    cfg.lora_bandwidth =  250000;// KHz
    cfg.lora_coding_rate = 5; // Error correction rate
    cfg.lora_spreading_factor = 7; 
    cfg.lora_power = 17; //17 PA OFF, 18-20 PA ON
    cfg.lora_pickup_delay = 5; // sec
    cfg.lora_cycle = 500; // ms
    cfg.lora_cycle_var = 50; // ms
    cfg.display_cycle = 250; // ms
    cfg.main_cycle = 1000; // ms
    cfg.peer_timeout = 3; // 5 sec ************************
    cfg.msp_fc_timeout = 5; // sec
    cfg.msp_tx_pin = 23; // pin for MSP serial TX
    cfg.msp_rx_pin = 17; // pin for MSP serial RX
}


// ----------------------------------------------------------------------------- LoRa

void lora_send(peer_t *outgoing) {
    if (sys_msp_ticker < 255) {
        sys_msp_ticker++;
    }
    else {
        sys_msp_ticker = 0;
    }
    
   
    curr.tick = sys_msp_ticker;
    curr.id = 1;
    String("TEST9").toCharArray(curr.name, 5);

    while (!LoRa.beginPacket()) {  }
    LoRa.write((uint8_t*)outgoing, sizeof(curr));
    LoRa.endPacket(false);

}

void onReceive(int packetSize) {
    if (packetSize == 0) return;
    LoRa.readBytes((uint8_t *)&incoming, packetSize);

    if (String(incoming.header) == String(cfg.lora_header)) { // New data from a peer

        sys_rssi = String(LoRa.packetRssi());
        sys_ppsc++;


//        uint8_t id = incoming.id - 1;
        uint8_t id = incoming.id;
        peers[id].id = incoming.id;
        peers[id].updated = millis();
        peers[id].rssi = LoRa.packetRssi();
        peers[id].peer = incoming;
        peers[id].peer.state = 1;

    }
}

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
    LoRa.setOCP(200);
    LoRa.idle();
    LoRa.onReceive(onReceive);
    LoRa.enableCrc();
    LoRa.receive();
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

display.drawString (60,54, String(sys_msp_ticker)); 

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

        if (String(sys_curr_fc) != String("NO FC")) {
            display.drawString (0,54, sys_curr_fc);
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
            for (size_t i = 0; i <=2 ; i++) {
                if (peers[i].id != 0) {
                    display.setTextAlignment (TEXT_ALIGN_LEFT);
                    display.drawString (0, i*16, peers[i].peer.name);
                    display.drawString (0, 8 + i * 16, "RSSI " + String(peers[i].rssi));
                    display.setTextAlignment (TEXT_ALIGN_RIGHT);
                    display.drawString (128, i * 16, String((float)peers[i].peer.gps.lat/10000000, 6));
                    display.drawString (128, 8 + i * 16, String((float)peers[i].peer.gps.lon/10000000, 6));
                }
            }
        }
    }
    
    else if (sys_display_page == 2) {
        display.setFont (ArialMT_Plain_10);
        for (size_t i = 0; i <=1 ; i++) {
            if (peers[i].id != 0) {
                display.setTextAlignment (TEXT_ALIGN_LEFT);
                display.drawString (0, i*16, peers[i+3].peer.name);
                display.drawString (0, 8 + i * 16, "RSSI " + String(peers[i].rssi));
                display.setTextAlignment (TEXT_ALIGN_RIGHT);
                display.drawString (65, i * 16, String((float)peers[i+3].peer.gps.lat/10000000, 6));
                display.drawString (65, 8 + i * 16, String((float)peers[i+3].peer.gps.lon/10000000, 6));
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
    char buff[16];
    if (!msp.request(MSP_NAME, &buff, sizeof(curr.name))) {
        for (size_t i = 0; i < 5; i++) {
            curr.name[i] = (char) random(65, 90);
        }
    } else {
        for (size_t i = 0; i < 5; i++) {
            curr.name[i] = buff[i]; // ------ 
        }            
    }
}
    
void msp_set_curr_fc() {    
    if (!msp.request(MSP_FC_VARIANT, &sys_curr_fc, sizeof(sys_curr_fc))) {
        String("NoFC ").toCharArray(sys_curr_fc, 5);
    }
}



void msp_send_peers() {
    msp_radar_pos_t radarPos;
    for (size_t i = 0; i <= 4; i++) {
        if (peers[i].id != 0) {
            radarPos.id = i;
            radarPos.state = peers[i].peer.state;
            radarPos.lat = peers[i].peer.gps.lat;
            radarPos.lon = peers[i].peer.gps.lon;
            radarPos.alt = peers[i].peer.gps.alt;
            radarPos.speed = peers[i].peer.gps.groundSpeed;
            radarPos.heading = peers[i].peer.gps.groundCourse / 10;
            msp.command(MSP_SET_RADAR_POS, &radarPos, sizeof(radarPos));
        }
    }
}

void msp_init() {

    display.drawString (0, 24, "MSP ");
    display.display();
    delay(100);
    Serial1.begin(57600, SERIAL_8N1, cfg.msp_rx_pin , cfg.msp_tx_pin);
    msp.begin(Serial1);

    display.drawString (100, 24, "OK");
    display.display();
    display.drawString (0, 32, "FC ");
    display.display();

    delay(cfg.msp_fc_timeout * 1000);
    msp_set_curr_name();
    msp_set_curr_fc();
    msp_set_curr_gps();

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
//  Serial.begin(115200);
//  serialConsole[0] = &Serial;

    config_init();

    display_init();
    display.drawString(0, 8, "FIRMWARE");
    display.drawString(100, 8, VERSION);
    display.display();
    
    lora_init();

    display.drawString (100, 16, "OK");
    display.display();

    delay(500);

    msp_init();
    delay(500);

    pinMode(interruptPin, INPUT);
    io_button_pressed = 0;
    attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
    
    for (size_t i = 0; i <= 4; i++) {
        peers[i].id = 0;
    }
    
    curr.id = 0;
    lora_mode = LORA_RX;
    lora_cycle_eff  = cfg.lora_cycle;
    
    lora_last_tx = millis();
    main_updated = millis();
    display_updated = millis();

}

// ----------------------------------------------------------------------------- MAIN LOOP

void loop() {

// ---------------------- IO BUTTON

    if ((millis() - io_button_released) > 150 && io_button_pressed == 1) {
        io_button_pressed = 0;
    }

// ---------------------- LORA INIT    
/*    
    if (lora_mode == LORA_INIT && curr.id == 0) {
       lora_mode = LORA_PICKUP;
       sys_pickup_updated = millis();
    }
*/    
// ---------------------- LORA PICKUP 
/*
    if ((lora_mode == LORA_PICKUP) && (millis() - sys_pickup_updated > cfg.lora_pickup_delay * 1000)) {
        sys_num_peers = 0;
        for (int i = 0; i < sizeof(peers); i++) {
           if (peers[i].id == 0 && curr.id == 0) {
               curr.id = i + 1;
               lora_mode = LORA_RX;
            }
            if (peers[i].id != 0) {
                sys_num_peers++;
            }
        }
        if (peers[0].id == 1) {
            lora_last_tx = peers[0].updated;
        }
    }
*/
// ---------------------- LORA RX
    
    
        if ((millis() - lora_last_tx) > lora_cycle_eff) {
            lora_mode = LORA_TX;
            lora_cycle_eff  = cfg.lora_cycle - random(0, cfg.lora_cycle_var);
            lora_last_tx = millis();
        }


// ---------------------- LORA TX    
    
    if (lora_mode == LORA_TX) {
        
        
     // msp_set_curr_gps();
        
   //     if (curr.gps.fixType > 0) {
           lora_send(&curr);

   //     }
        
        if (String(sys_curr_fc) == "INAV" ) {
            msp_send_peers();
        }

        LoRa.sleep();
        LoRa.receive();
        
        lora_mode = LORA_RX;
    }

    
    if ((display_enabled && ((millis() - display_updated)) > cfg.display_cycle)) {
    
        display_draw();
        display_updated = millis();
    }
    
    
    if ((millis() - main_updated) > cfg.main_cycle) {
        sys_num_peers = 0;
        sys_pps = sys_ppsc;
        sys_ppsc = 0;
        
        for (size_t i = 0; i <= 4; i++) {
            if (peers[i].id != 0) { 
                sys_num_peers++;
            }

            if (peers[i].id != 0 && ((millis() - peers[i].updated) > cfg.peer_timeout * 1000)) { // Peer timeout
                peers[i].peer.state = 0;
                // msp_send_peers();
                peers[i].id = 0;
                sys_rssi = "0";
                String("").toCharArray(peers[i].peer.name, 5);
            }
        }

        if (String(sys_curr_fc) == "INAV" ) {
            msp_set_curr_state();
            msp_set_curr_vbat();
        }

        if ((curr.state == 0) && (display_enabled == 0)) {
            display_enable();
        }
        
        else if (curr.state == 1 && (display_enabled == 1)) { // Aircraft is armed = Turning off the OLED
            display_disable();
        }

    main_updated = millis();
    }


}
