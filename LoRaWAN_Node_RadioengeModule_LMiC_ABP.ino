/* 
 *   
 *  Project:          IoT Energy Meter with C/C++, Java/Spring, TypeScript/Angular and Dart/Flutter;
 *  About:            End-to-end implementation of a LoRaWAN network for monitoring electrical quantities;
 *  Version:          1.0;
 *  Backend Mote:     ATmega328P/ESP32/ESP8266/ESP8285/STM32;
 *  Radios:           RFM95w and LoRaWAN EndDevice Radioenge Module: RD49C;
 *  Sensors:          Peacefair PZEM-004T 3.0 Version TTL-RTU kWh Meter;
 *  Backend API:      Java with Framework: Spring Boot;
 *  LoRaWAN Stack - Customized for the Frequency Plan adopted in Brazil for LoRaWAN --> [AU915]:
 *  MCCI Arduino LoRaWAN Library (LMiC: LoRaWAN-MAC-in-C) version 3.0.99;
 *  
 *  Or:
 *  LoRaWAN Stack - Implementation approved by the National Telecommunications Agency of Brazil
 *  and inserted by the company Radioenge:
 *  LoRaMac-node Library (LoRaWAN L2 1.0.3 - Released / API via AT commands);
 *  
 *  Activation mode:  Activation by Personalization (ABP) or Over-the-Air Activation (OTAA);
 *  Author:           Adail dos Santos Silva
 *  E-mail:           adail101@hotmail.com
 *  WhatsApp:         +55 89 9 9433-7661
 *  
 *  This project was conceived using the MCCI Arduino LoRaWAN Library (LMiC: LoRaWAN-MAC-in-C) version 3.0.99,
 *  This RD49C module in turn has the copyright belonging to the manufacturer, the company Radioenge do Brasil.
 *  All remaining implementation is authored by the creator of the LoRaWAN Electricity Meter project.
 *  
 *  WARNINGS:
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of
 *  this software and associated documentation files (the “Software”), to deal in
 *  the Software without restriction, including without limitation the rights to
 *  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 *  the Software, and to permit persons to whom the Software is furnished to do so,
 *  subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 *  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 *  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 *  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *  
 */


/*
 *  Radioenge Module - LMiC (LoRaWAN-MAC-in-C) Example
 */

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

/* Includes */
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/* Definitions */

// Pinout - IOs:
#define RX_1                    PB7
#define TX_1                    PB6

// Pinout - GPIOs:
#define GPIO0                   PA3
#define GPIO1                   PA2
#define GPIO2                   PA15
#define GPIO3                   PA10
#define GPIO4                   PA9
#define GPIO5                   PA8
#define GPIO6                   PB15
#define GPIO7                   PA0
#define GPIO8                   PA1
#define GPIO9                   PB11

// Pinout - LEDs:
#define LED_RED                 PB5
#define LED_GREEN               PB8
#define LED_YELLOW              PB9

// Pinout - Radio LoRa:
#define RADIO_RESET_PORT        PB14
#define RADIO_MOSI_PORT         PA7
#define RADIO_MISO_PORT         PA6
#define RADIO_SCLK_PORT         PA5
#define RADIO_NSS_PORT          PA4

#define RADIO_DIO_0_PORT        PB10
#define RADIO_DIO_1_PORT        PB2
#define RADIO_DIO_2_PORT        PB1
#define RADIO_DIO_3_PORT        PB0

#define RADIO_RXTX1_PORT        PB13
#define RADIO_RXTX2_PORT        PB12


/* Credentials */
// CHIRPSTACK - CS (8 at 15 + 65 channels):
/* little-endian - LSB */ // af c2 cf 56 1e a5 f4 0e 7d da 4b 82 89 c6 bf 59
/* big-endian - MSB */    // 59 bf c6 89 82 4b da 7d 0e f4 a5 1e 56 cf c2 af
static const u1_t PROGMEM APPSKEY[16] = {0x59, 0xBF, 0xC6, 0x89, 0x82, 0x4B, 0xDA, 0x7D, 0x0E, 0xF4, 0xA5, 0x1E, 0x56, 0xCF, 0xC2, 0xAF};

/* little-endian - LSB */ // 99 f2 cd 7c e8 5a 57 a2 e4 6f 6a 48 0e 39 55 a9
/* big-endian - MSB */    // a9 55 39 0e 48 6a 6f e4 a2 57 5a e8 7c cd f2 99
static const PROGMEM u1_t NWKSKEY[16] = {0xA9, 0x55, 0x39, 0x0E, 0x48, 0x6A, 0x6F, 0xE4, 0xA2, 0x57, 0x5A, 0xE8, 0x7C, 0xCD, 0xF2, 0x99};

/* little-endian - LSB */ // 1c e7 c4 23
/* big-endian - MSB */    // 23 c4 e7 1c
static const u4_t DEVADDR = 0x23c4e71c; // <-- Change this address for every node!

/* 
 *  These callbacks are only used in over-the-air activation, so they are
 *  left empty here (we cannot leave them out completely unless
 *  DISABLE_JOIN is set in config.h, otherwise the linker will complain).
 */
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}


/* Data to Send */
static uint8_t mydata[] = "AdailSilva";

/* Instances */
/* Jobs */
static osjob_t sendjob;

/* Schedule TX every this many seconds (might become longer due to duty cycle limitations). */
const unsigned TX_INTERVAL = 10;

/* Pin mapping. */
const lmic_pinmap lmic_pins = {
  .nss = RADIO_NSS_PORT,
  .rxtx = RADIO_RXTX1_PORT,
  .rst = RADIO_RESET_PORT,
  .dio = {RADIO_DIO_0_PORT, RADIO_DIO_1_PORT, RADIO_DIO_2_PORT},
  .rxtx_rx_active = 1,
};

/*****************************
 _____           _      
/  __ \         | |     
| /  \/ ___   __| | ___ 
| |    / _ \ / _` |/ _ \
| \__/\ (_) | (_| |  __/
 \____/\___/ \__,_|\___|
*****************************/

/* Functions. */

/* Log Functions */
void printHex2(unsigned key)
{
    key &= 0xff;

    if (key < 16)
    {
        Serial.print('0');
    }
    
    Serial.print(key, HEX);
}

void showNetworkInformations()
{
    u4_t netid = 0;
    devaddr_t devaddr = 0;
    u1_t nwkKey[16];
    u1_t artKey[16];
    LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);

//    Serial.println(F("_____________________________________________________________________________"));
    Serial.println();
    Serial.println(" [INFO] NetID   (MSB)       : " + String(netid, HEX));
    Serial.println(" [INFO] DevAddr (MSB)       : " + String(devaddr, HEX));

    Serial.print(" [INFO] NwkSKey (MSB)       : "); // Not used (F());
    
    for (size_t i = 0; i < sizeof(nwkKey); ++i)
    {
        if (i != 0)
            Serial.print("-");
        printHex2(nwkKey[i]);
    }

    Serial.println("");        
    Serial.print(" [INFO] AppSKey (MSB)       : ");

    for (size_t i = 0; i < sizeof(artKey); ++i)
    {
        if (i != 0)
            Serial.print("-");
        printHex2(artKey[i]);
    }
    
    Serial.println();
//    Serial.println(F("_____________________________________________________________________________"));
    Serial.println();
}

void onEvent (ev_t ev) {
//    Serial.print(os_getTime());
//    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack;"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" byte(s) of payload."));
            }
            /* Schedule next transmission. */
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            /* Data received in ping slot. */
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    /* Check if there is not a current TX/RX job running. */
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        /* Prepare upstream data transmission at the next possible time. */
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    /* Next TX is scheduled after TX_COMPLETE event. */
}

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/

void setup() {
    Serial.begin(9600);
    delay(100);
    Serial.println(F("Starting..."));

    /* For Pinoccio Scout boards. */
    #ifdef VCC_ENABLE
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    /* All LED pins as Output. */
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    /* Turn leds off. */
    digitalWrite(LED_RED, HIGH); 
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, HIGH);

    pinMode(RADIO_RXTX2_PORT, OUTPUT);
    digitalWrite(RADIO_RXTX2_PORT, LOW);

    /* LMIC init. */
    os_init();
    
    /* Reset the MAC state. Session and pending data transfers will be discarded. */
    LMIC_reset();

    /* 
     *  Set static session parameters. Instead of dynamically establishing a session
     *  by joining the network, precomputed session parameters are be provided.
     */
    #ifdef PROGMEM
    /* 
     *  On AVR, these values are stored in flash and only copied to RAM
     *  once. Copy them to a temporary buffer here, LMIC_setSession will
     *  copy them into a buffer of its own again.
     */
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
//    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
    #else
    /* If not running an AVR with PROGMEM, just use the arrays directly. */
//    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
    #endif

//    LMIC_selectSubBand(1);

    /* Channels Control to AU915 (8 at 15 + 65 channels): */
    for (u1_t b = 0; b < 8; ++b)
    {
        LMIC_disableSubBand(b);
    }

    for (u1_t channel = 0; channel < 72; ++channel)
    {
        LMIC_disableChannel(channel);
    }

    LMIC_enableChannel(8);
    LMIC_enableChannel(9);
    LMIC_enableChannel(10);
    LMIC_enableChannel(11);
    LMIC_enableChannel(12);
    LMIC_enableChannel(13);
    LMIC_enableChannel(14);
    LMIC_enableChannel(15);
//    LMIC_enableChannel(65); /* Test */

    /* Disable Adaptive Data Rate. */    
    LMIC_setAdrMode(0);
    
    /* Disable link check validation. */
    LMIC_setLinkCheckMode(0);

    /* TTN uses SF9 for its RX2 window. */
    LMIC.dn2Dr = DR_SF9;

    /* Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library). */
    LMIC_setDrTxpow(DR_SF7,14);

    /* 
     *  Use with Arduino Pro Mini ATmega328P 3.3V 8MHz;
     *  Let LMIC compensate for +/- 1% clock error.
     */
    LMIC_setClockError (MAX_CLOCK_ERROR * 10 / 100);

    showNetworkInformations();

    /* Start job. */
    do_send(&sendjob);
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop() {
    unsigned long now;
    now = millis();
    if ((now & 512) != 0) {
      digitalWrite(LED_GREEN, HIGH);
    }
    else {
      digitalWrite(LED_GREEN, LOW);
    }
    
    os_runloop_once();
}
