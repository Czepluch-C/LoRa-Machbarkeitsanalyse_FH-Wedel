/******************************************************************************
 *
 * Copyright (c) 2015 Thomas Telkamp
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Changes for creating a dual channel gateway with the Raspberry Pi+ LoRa(TM) Expansion Board
 * of Uputronics, see also: store.uputronics.com/index.php?route=product/product&product_id=68
 *******************************************************************************/

/*
Modified by Christian Czepluch


*/

// Raspberry PI pin mapping
// Pin number in this global_conf.json are Wiring Pi number (wPi colunm)
// issue a `gpio readall` on PI command line to see mapping
// +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
// | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
// +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
// |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
// |   2 |   8 |   SDA.1 |   IN | 1 |  3 || 4  |   |      | 5V      |     |     |
// |   3 |   9 |   SCL.1 |   IN | 1 |  5 || 6  |   |      | 0v      |     |     |
// |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | ALT5 | TxD     | 15  | 14  |
// |     |     |      0v |      |   |  9 || 10 | 1 | ALT5 | RxD     | 16  | 15  |
// |  17 |   0 | GPIO. 0 |  OUT | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
// |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
// |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
// |     |     |    3.3v |      |   | 17 || 18 | 1 | IN   | GPIO. 5 | 5   | 24  |
// |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
// |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
// |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
// |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
// |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
// |   5 |  21 | GPIO.21 |  OUT | 0 | 29 || 30 |   |      | 0v      |     |     |
// |   6 |  22 | GPIO.22 |  OUT | 0 | 31 || 32 | 1 | IN   | GPIO.26 | 26  | 12  |
// |  13 |  23 | GPIO.23 |  OUT | 0 | 33 || 34 |   |      | 0v      |     |     |
// |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
// |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
// |     |     |      0v |      |   | 39 || 40 | 0 | OUT  | GPIO.29 | 29  | 21  |
// +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
// | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
// +-----+-----+---------+------+---+---Pi 3---+---+------+---------+-----+-----+
// For Uputronics Raspberry Pi+ LoRa(TM) Expansion Board
// pins configuration in file global_conf.json
//
//
//  "pin_nss": 10,
//  "pin_dio0": 6,
//  "pin_nss2": 11,
//  "pin_dio0_2": 27,
//  "pin_rst": 0,
//  "pin_NetworkLED": 22,
//  "pin_InternetLED": 23,
//  "pin_ActivityLED_0": 21,
//  "pin_ActivityLED_1": 29,
//


//#include <arpa/inet.h>
//#include <net/if.h>
//#include <netdb.h>
//#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
//#include <rapidjson/stringbuffer.h>
//#include <rapidjson/writer.h>
//#include <stdlib.h>
//#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

//#include <cstdint>
//#include <cstdio>
//#include <cstdlib>
//#include <cstring>
//#include <iostream>
//#include <string>
//#include <vector>

//#include "base64.h"

// using namespace std;

// using namespace rapidjson;

#define BASE64_MAX_LENGTH 341

static const int SPI_CHANNEL = 0;
static const int SPI_CHANNEL_2 = 1;

bool sx1272 = true;
typedef unsigned char byte;

typedef enum SpreadingFactors { SF7 = 7, SF8, SF9, SF10, SF11, SF12 } SpreadingFactor_t;


/*******************************************************************************
 *
 * Default values, configure them in global_conf.json
 *
 *******************************************************************************/

// uputronics - Raspberry connections
// Put them in global_conf.json
int ssPin = 0x06;
int dio0 = 0x07;
int ssPin_2 = 0x06;
int dio0_2 = 0x07;
int RST = 0x03;
int Led1 = 0x04;
int NetworkLED = 22;
int InternetLED = 23;
int ActivityLED_0 = 21;
int ActivityLED_1 = 29;


// Set spreading factor (SF7 - SF12), &nd  center frequency
// Overwritten by the ones set in global_conf.json
SpreadingFactor_t sf = SF7;
uint16_t bw = 125;
uint32_t freq = 868100000;  // in Mhz! (868.1)
// uint32_t freq_2 = 868100000;  // in Mhz! (868.3)

// #############################################
// #############################################

#define REG_FIFO 0x00
#define REG_FIFO_ADDR_PTR 0x0D
#define REG_FIFO_TX_BASE_AD 0x0E
#define REG_FIFO_RX_BASE_AD 0x0F
#define REG_RX_NB_BYTES 0x13
#define REG_OPMODE 0x01
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_MODEM_CONFIG 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_MODEM_CONFIG3 0x26
#define REG_SYMB_TIMEOUT_LSB 0x1F
#define REG_PKT_SNR_VALUE 0x19
#define REG_PAYLOAD_LENGTH 0x22
#define REG_IRQ_FLAGS_MASK 0x11
#define REG_MAX_PAYLOAD_LENGTH 0x23
#define REG_HOP_PERIOD 0x24
#define REG_SYNC_WORD 0x39
#define REG_VERSION 0x42

#define SX72_MODE_RX_CONTINUOS 0x85
#define SX72_MODE_TX 0x83
#define SX72_MODE_SLEEP 0x80
#define SX72_MODE_STANDBY 0x81


#define PAYLOAD_LENGTH 0x40

// LOW NOISE AMPLIFIER
#define REG_LNA 0x0C
#define LNA_MAX_GAIN 0x23
#define LNA_OFF_GAIN 0x00
#define LNA_LOW_GAIN 0x20

// CONF REG
#define REG1 0x0A
#define REG2 0x84

#define SX72_MC2_FSK 0x00
#define SX72_MC2_SF7 0x70
#define SX72_MC2_SF8 0x80
#define SX72_MC2_SF9 0x90
#define SX72_MC2_SF10 0xA0
#define SX72_MC2_SF11 0xB0
#define SX72_MC2_SF12 0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE 0x01  // mandated for SF11 and SF12

// FRF
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08

#define FRF_MSB 0xD9  // 868.1 Mhz
#define FRF_MID 0x06
#define FRF_LSB 0x66


void Die(const char* s) {
    perror(s);
    exit(1);
}

void SelectReceiver(byte CE) {
    if (CE == 0) {
        digitalWrite(ssPin, LOW);
    } else {
        digitalWrite(ssPin_2, LOW);
    }
}

void UnselectReceiver(byte CE) {
    if (CE == 0) {
        digitalWrite(ssPin, HIGH);
    } else {
        digitalWrite(ssPin_2, HIGH);
    }
}

uint8_t ReadRegister(uint8_t addr, byte CE) {
    uint8_t spibuf[2];

    SelectReceiver(CE);
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CE, spibuf, 2);
    UnselectReceiver(CE);

    return spibuf[1];
}

void WriteRegister(uint8_t addr, uint8_t value, byte CE) {
    uint8_t spibuf[2];

    SelectReceiver(CE);
    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    wiringPiSPIDataRW(CE, spibuf, 2);

    UnselectReceiver(CE);
}

bool ReceivePkt(char* payload, uint8_t* p_length, byte CE) {
    // clear rxDone
    WriteRegister(REG_IRQ_FLAGS, 0x40, CE);

    int irqflags = ReadRegister(REG_IRQ_FLAGS, CE);

    //  payload crc: 0x20
    if ((irqflags & 0x20) == 0x20) {
        printf("CRC error\n");
        WriteRegister(REG_IRQ_FLAGS, 0x20, CE);
        return false;

    } else {
        uint8_t currentAddr = ReadRegister(REG_FIFO_RX_CURRENT_ADDR, CE);
        uint8_t receivedCount = ReadRegister(REG_RX_NB_BYTES, CE);
        *p_length = receivedCount;

        WriteRegister(REG_FIFO_ADDR_PTR, currentAddr, CE);

        for (int i = 0; i < receivedCount; i++) {
            payload[i] = ReadRegister(REG_FIFO, CE);
        }
    }
    return true;
}

char* PinName(int pin, char* buff) {
    strcpy(buff, "unused");
    if (pin != 0xff) {
        sprintf(buff, "%d", pin);
    }
    return buff;
}

void SetupLoRa(byte CE) {
    char buff[16];
    if (CE == 0) {
        printf("Trying to detect module CE0 with ");
        printf("NSS=%s ", PinName(ssPin, buff));
        printf("DIO0=%s ", PinName(dio0, buff));
        printf("Reset=%s ", PinName(RST, buff));
        printf("Led1=%s\n", PinName(Led1, buff));
    } else {
        printf("Trying to detect module CE1 with ");
        printf("NSS=%s ", PinName(ssPin_2, buff));
        printf("DIO0=%s ", PinName(dio0_2, buff));
        printf("Reset=%s ", PinName(RST, buff));
        printf("Led1=%s\n", PinName(Led1, buff));
    }

    // check basic
    if (ssPin == 0xff || dio0 == 0xff) {
        Die("Bad pin configuration ssPin and dio0 need at least to be defined");
    }


    uint8_t version = ReadRegister(REG_VERSION, CE);

    if (version == 0x22) {
        // sx1272
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    } else {
        // sx1276?
        version = ReadRegister(REG_VERSION, CE);
        if (version == 0x12) {
            // sx1276
            if (CE == 0) {
                printf("SX1276 detected on CE0, starting.\n");
            } else {
                printf("SX1276 detected on CE1, starting.\n");
            }
            sx1272 = false;
        } else {
            printf("Transceiver version 0x%02X\n", version);
            Die("Unrecognized transceiver");
        }
    }

    WriteRegister(REG_OPMODE, SX72_MODE_SLEEP, CE);

    // set frequency
    uint64_t frf;
    if (CE == 0) {
        frf = ((uint64_t)freq << 19) / 32000000;
    } else {
        // frf = ((uint64_t)freq_2 << 19) / 32000000;
    }
    WriteRegister(REG_FRF_MSB, (uint8_t)(frf >> 16), CE);
    WriteRegister(REG_FRF_MID, (uint8_t)(frf >> 8), CE);
    WriteRegister(REG_FRF_LSB, (uint8_t)(frf >> 0), CE);

    WriteRegister(REG_SYNC_WORD, 0x12, CE);  // Private sync word: 0x12    LoRaWAN public sync word: 0x34

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            WriteRegister(REG_MODEM_CONFIG, 0x0B, CE);
        } else {
            WriteRegister(REG_MODEM_CONFIG, 0x0A, CE);
        }
        WriteRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04, CE);
    } else {
        if (sf == SF11 || sf == SF12) {
            WriteRegister(REG_MODEM_CONFIG3, 0x0C, CE);
        } else {
            WriteRegister(REG_MODEM_CONFIG3, 0x04, CE);
        }
        WriteRegister(REG_MODEM_CONFIG, 0x72, CE);
        WriteRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04, CE);
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        WriteRegister(REG_SYMB_TIMEOUT_LSB, 0x05, CE);
    } else {
        WriteRegister(REG_SYMB_TIMEOUT_LSB, 0x08, CE);
    }
    WriteRegister(REG_MAX_PAYLOAD_LENGTH, 0x80, CE);
    WriteRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH, CE);
    WriteRegister(REG_HOP_PERIOD, 0xFF, CE);
    WriteRegister(REG_FIFO_ADDR_PTR, ReadRegister(REG_FIFO_RX_BASE_AD, CE), CE);

    // Set Continous Receive Mode
    WriteRegister(REG_LNA, LNA_MAX_GAIN, CE);  // max lna gain
    WriteRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS, CE);
}

void printMessage(char* message, int length) {
    // printf("len:%u/", len);
    printf("Message: ");
    for (int i = 0; i < length; i++) {
        printf("%d/", message[i]);
    }
}

bool Receivepacket(byte CE) {
    long int SNR;
    int rssicorr, dio_port;
    bool ret = false;

    if (CE == 0) {
        dio_port = dio0;
    } else {
        dio_port = dio0_2;
    }

    if (digitalRead(dio_port) == 1) {
        char message[256];
        uint8_t length = 0;
        if (ReceivePkt(message, &length, CE)) {
            // OK got one
            ret = true;

            uint8_t value = ReadRegister(REG_PKT_SNR_VALUE, CE);

            if (CE == 0) {
                digitalWrite(ActivityLED_0, HIGH);
            } else {
                digitalWrite(ActivityLED_1, HIGH);
            }

            if (value & 0x80) {  // The SNR sign bit is 1
                // Invert and divide by 4
                value = ((~value + 1) & 0xFF) >> 2;
                SNR = -value;
            } else {
                // Divide by 4
                SNR = (value & 0xFF) >> 2;
            }

            rssicorr = sx1272 ? 139 : 157;

            printf("Packet RSSI: %d, ", ReadRegister(0x1A, CE) - rssicorr);
            printf("RSSI: %d, ", ReadRegister(0x1B, CE) - rssicorr);
            printf("SNR: %li, ", SNR);
            printf("Length: %i, ", (int)length);
            printMessage(message, (int)length);
            printf("\n");
        }
    }
    return ret;
}


int main(int argc, char* argv[]) {
    setbuf(stdout, NULL);  // flushes the stdout so it's easier to read the output with python
    unsigned int led0_timer, led1_timer;
    bool changedFreqAndSF = false;

    if (argc == 2) {
        char help1[] = "--h";
        char help2[] = "--help";
        char help3[] = "-h";
        char help4[] = "-help";
        bool help = 0 == strcmp(argv[1], help1) || 0 == strcmp(argv[1], help2) ||
                    0 == strcmp(argv[1], help3) || 0 == strcmp(argv[1], help4);

        if (help) {
            printf("Help for LoRa Receiver:\n\n");
            printf("-h                     Help\n");
            printf("-f [freq]  -sf [SF]    Frequency in Hz [868000000..870000000] and\n");
            printf("                       Spreading Factor [7..12]\n\n");
            printf("Leave blank for 868.1 MHz and SF7\n");
            printf("\n");
            return 0;
        } else {
            return 1;
        }

    } else if (argc == 5) {
        char frequencyStr[] = "-f";
        char spreadingFactorStr[] = "-sf";
        bool change = 0 == strcmp(argv[1], frequencyStr) && 0 == strcmp(argv[3], spreadingFactorStr);
        if (change) {
            uint32_t thisFreq = atoi(argv[2]);
            uint32_t thisSF = atoi(argv[4]);
            if (thisFreq < 868000000 || thisFreq > 870000000 || thisSF < 7 || thisSF > 12) {
                printf("Wrong Frequency or wrong SF\nUse -h for help\n");
                return -1;
            }
            freq = thisFreq;
            sf = static_cast<SpreadingFactor_t>(thisSF);
            // printf("Freq: %i, SF: %i", thisFreq, thisSF);+
            changedFreqAndSF = true;
        } else {
            return -1;
        }
    }

    if (argc == 1 || changedFreqAndSF) {
        printf("Argc: %i  ", argc);
        printf("Argv: ");
        for (int i = 0; i < argc; i++) {
            printf("%s  ", argv[i]);
        }
        printf("\n");


        // LoadConfiguration("global_conf.json");
        // PrintConfiguration();

        // Init WiringPI
        wiringPiSetup();
        pinMode(ssPin, OUTPUT);
        pinMode(ssPin_2, OUTPUT);
        pinMode(dio0, INPUT);
        pinMode(dio0_2, INPUT);
        pinMode(RST, OUTPUT);
        pinMode(NetworkLED, OUTPUT);
        pinMode(ActivityLED_0, OUTPUT);
        pinMode(ActivityLED_1, OUTPUT);
        pinMode(InternetLED, OUTPUT);

        // Init SPI
        wiringPiSPISetup(SPI_CHANNEL, 500000);

        // Setup LORA
        digitalWrite(RST, HIGH);
        delay(100);
        digitalWrite(RST, LOW);
        delay(100);
        SetupLoRa(0);


        printf("Listening at SF%i on %.6lf Mhz.\n", sf, (double)freq / 1000000);
        printf("-----------------------------------\n");

        while (1) {
            // Packet received ?
            if (Receivepacket(0)) {
                // Led ON
                if (ActivityLED_0 != 0xff) {
                    digitalWrite(ActivityLED_0, 1);
                }

                // start our Led blink timer, LED as been lit in Receivepacket
                led0_timer = millis();
            }
            if (Receivepacket(1)) {
                // Led ON
                if (ActivityLED_1 != 0xff) {
                    digitalWrite(ActivityLED_1, 1);
                }

                // start our Led blink timer, LED as been lit in Receivepacket
                led1_timer = millis();
            }

            // Led timer in progress ?
            if (led0_timer) {
                // Led timer expiration, Blink duration is 250ms
                if (millis() - led0_timer >= 250) {
                    // Stop Led timer
                    led0_timer = 0;

                    // Led OFF
                    if (ActivityLED_0 != 0xff) {
                        digitalWrite(ActivityLED_0, 0);
                    }
                }
            }
            if (led1_timer) {
                // Led timer expiration, Blink duration is 250ms
                if (millis() - led1_timer >= 250) {
                    // Stop Led timer
                    led1_timer = 0;

                    // Led OFF
                    if (ActivityLED_1 != 0xff) {
                        digitalWrite(ActivityLED_1, 0);
                    }
                }
            }


            // Let some time to the OS
            delay(1);
        }

        return (0);

    } else {
        return 1;
    }
}