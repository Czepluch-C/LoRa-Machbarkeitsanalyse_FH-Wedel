/*
Created by Christian Czepluch

*/

#include "ESP32_LoRaWAN.h"

#define button 17

#define pin7 36
#define pin8 37
#define pin9 38
#define pin10 39
#define pin11 12
#define pin12 13

#define LED 25

#define LoRa_PACKET_LENGTH 32


#define RF_FREQUENCY 868100000  // Hz
#define TX_OUTPUT_POWER 14      // dBm
#define LORA_BANDWIDTH \
    0  // [0: 125 kHz,
       //  1: 250 kHz,
       //  2: 500 kHz,
       //  3: Reserved]
//#define LORA_SPREADING_FACTOR 7  // [SF7..SF12]
#define LORA_CODINGRATE \
    1                           // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

int sf = 7;
// LoRa license
uint32_t license[4] = {0xD5397DF0, 0x8573F814, 0x7A38C73D, 0x48E68607};

byte LoRaPacket[LoRa_PACKET_LENGTH] = {0};

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);

int counter = 1;

void setupTxRadio(void) {
    // RadioEvents.TxDone = OnTxDone;
    // RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, sf, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON,
                      3000);
}

void sendPackage() { Radio.Send(LoRaPacket, sizeof(LoRaPacket)); }

void setup() {
    Serial.begin(115200);

    // put your setup code here, to run once:
    pinMode(button, INPUT_PULLUP);

    pinMode(pin7, INPUT_PULLUP);
    pinMode(pin8, INPUT_PULLUP);
    pinMode(pin9, INPUT_PULLUP);
    pinMode(pin10, INPUT_PULLUP);
    pinMode(pin11, INPUT_PULLUP);
    pinMode(pin12, INPUT_PULLUP);

    pinMode(LED, OUTPUT);
    delay(1000);

    bool isSF7 = digitalRead(pin7) == HIGH;
    bool isSF8 = digitalRead(pin8) == HIGH;
    bool isSF9 = digitalRead(pin9) == HIGH;
    bool isSF10 = digitalRead(pin10) == HIGH;
    bool isSF11 = digitalRead(pin11) == HIGH;
    bool isSF12 = digitalRead(pin12) == HIGH;

    Serial.print("isSF 7: ");
    Serial.println(isSF7);
    Serial.print("isSF 8: ");
    Serial.println(isSF8);
    Serial.print("isSF 9: ");
    Serial.println(isSF9);
    Serial.print("isSF 10: ");
    Serial.println(isSF10);
    Serial.print("isSF 11: ");
    Serial.println(isSF11);
    Serial.print("isSF 12: ");
    Serial.println(isSF12);



    if (digitalRead(pin7) == HIGH) {
        sf = 7;
    } else if (digitalRead(pin9) == HIGH) {
        sf = 9;
    } else if (digitalRead(pin12) == HIGH) {
        sf = 12;
    }
    Serial.print("SF: ");
    Serial.print(sf);
    Serial.println(" selected");

    LoRaPacket[0] = 1;
    LoRaPacket[1] = 1;
    LoRaPacket[2] = 1;
    LoRaPacket[3] = sf;
    LoRaPacket[4] = sf;
    LoRaPacket[5] = sf;


    // for LoRa
    SPI.begin(SCK, MISO, MOSI, SS);
    Mcu.init(SS, RST_LoRa, DIO0, DIO1, license);
    setupTxRadio();
}

void loop() {
    if (digitalRead(button) == LOW) {
        LoRaPacket[6] = counter;
        LoRaPacket[7] = counter;
        LoRaPacket[8] = counter;

        digitalWrite(LED, HIGH);
        Serial.print("Sending on SF ");
        Serial.print(sf);
        Serial.println(" ...");

        sendPackage();

        counter++;

        Serial.println("Sending complete");

        delay(5000);
        digitalWrite(LED, LOW);
        Serial.println("WaitTime is over");
        Serial.println();
    }
}
