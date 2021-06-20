/*
Created by Christian Czepluch

*/

// complete protocolID 1 code

#include <AES.h>
#include <Adafruit_SGP30.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ESP32_LoRaWAN.h>
#include <Wire.h>

/** Protocol_ID of this kind of devices, which sends this kind of data
 *  0..63
 */
#define PROTOCOL_ID 1

/** ID of this particular sender
 *  different for every device which uses Protocol=1
 *  0..511 -> for Protocol=1
 */
#define SENDER_ID 3

/** can be changed for each device, verifies that this is an authorized device
 *  0..4095 -> for Protocol=1
 */
#define VERIFICATION_ID 2134


#define INTERVAL 450E3  // Interval of sending Messages in milli Sec

/**
 * Sensor 1 / gruen1:   CO2: 0x8ECD     TVOC: 0x959C
 * Sensor 2 / rot2:     CO2: 0x919F     TVOC: 0x94DA
 */

#define CO2_BASE 0x9955   // calibration base for CO2 Sensor, different for every Sensor
#define TVOC_BASE 0x976B  // calibration base for TVOC Sensor, different for every Sensor

/**
 * defines the Interval every 'PIR_INTERVAL' cycles the motion sensor is active
 * 0 -> never active
 * 1 -> always active
 * 2 -> every second cycle
 * ...
 */
//#define PIR_INTERVAL 2
#define WAIT_TILL_MOVEMENT 12E3  // time to wait for a motion to happen

// multiplies the voltage of the battery to get better results
#define VOLTAGE_MULTIPLIER 1.05



// the other 'defines' shouldn't be changed unless you know what you're doing
// make sure to set the AES Key after the 'defines'

#define DHT_11_PIN 13       // Pin for temp / hum sensor
#define PIR_Pin 2           // Pin for motion sensor can change for sensor use 2 or 38
#define LED_PIN 23          // Pin for motion LED
#define ADC_PIN 39          // Pin for Reading of battery charge
#define R1 1E6              // 1. resistor for voltage divider to measure batterycharge
#define R2 2E6              // 2. resistor for voltage divider to measure batterycharge
#define VOLTAGE_FULL 4.15   // the voltage of a full battery
#define VOLTAGE_EMPTY 3.35  // the voltage of an empty battery
#define BOARD_POWER_PIN 32  // controls Power for the sensors
#define PIR_POWER_PIN 33    // controls Power for the motion sensor


// can only send data multiples of 16 -1, so 15 is the smallest
#define MEASUREMENT_DATA_LENGTH 15

// this should be MEASUREMENT_DATA_LENGTH + Padding + AES_IV_Length
#define LoRa_PACKET_LENGTH 32

// Key size of AES Key
#define AES_KEY_SIZE 16


#define RF_FREQUENCY 868100000  // Hz
#define TX_OUTPUT_POWER 14      // dBm
#define LORA_BANDWIDTH \
    0                            // [0: 125 kHz,
                                 //  1: 250 kHz,
                                 //  2: 500 kHz,
                                 //  3: Reserved]
#define LORA_SPREADING_FACTOR 7  // [SF7..SF12]
#define LORA_CODINGRATE \
    1                           // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define mS_TO_uS_FACTOR 1000


/** type that contains all the Data thats going to be sent via LoRa at ProtocolID=1
 *  all ranges are for Protocol=1
 *  the values are the one measured for ex. temperature 32.3°C
 *  transmit value 0 means that the measurement wasn't measured
 */
typedef struct measurements {
    uint8_t protocolID;       // ID of Protocoll 0..63 6bit
    uint16_t senderID;        // ID of sender 0..511 9bit
    uint16_t verificationID;  // ID for verification 0..4095 12bit

    // counts LoRa Packets after each restart 0..1,048,575 20bit ≈ 10 years for 1 packet every 5 min
    uint32_t counter;
    uint8_t batteryCharge;  // Battery Charge % 0..127 7bit
    float temperature;      // Temperature °C
    float humidity;         // Humidity %rH

    /** if there was movement during the measurement period 0..3 2bit
     * 0 [00]: no measurement
     * 1 [01]: sensor was running but no movement
     * 2 [10]: no measurement
     * 3 [11]: sensor was running and movement
     */
    bool movement;
    uint16_t tvoc;      // TVOC ug/m^3
    uint16_t co2;       // CO2 ppm
    uint8_t water;      // watersensor for flood 0..6 3bit
    float brightness;   // brightness lux
    float soundLevel;   // sound level dB
    float airPressure;  // Air Pressure hPa
} measurements;

// type that can store if sensors are active
typedef struct activeSensor {
    bool batteryCharge;
    bool temperature;
    bool humidity;
    bool movement;
    bool tvoc;
    bool co2;
    bool water;
    bool brightness;
    bool soundLevel;
    bool airPressure;
} activeSensor;


// AES Key, must be the same, that the Gateway uses
const byte AES_KEY2[AES_KEY_SIZE] = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
                                     0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};
const byte AES_KEY[AES_KEY_SIZE] = {142, 194, 164, 255, 161, 160, 178, 156, 20, 236, 200, 109, 60, 5, 82, 54};

// encrypted LoRa-packet thats going to be send
byte LoRaPacket[LoRa_PACKET_LENGTH] = {0};

// contains the data of the measurement typedef, ready to be encrypted
byte measurementData[MEASUREMENT_DATA_LENGTH] = {0};

// contains all the measurements
measurements measurement;

// contains if sensors on this device are active
activeSensor isActive;

DHT dht11(DHT_11_PIN, DHT11);  // DHT11 Sensor (Temperature and Humidity)

Adafruit_SGP30 sgp;  // SGP30 Sensor (eCO2 and TVOC)

uint32_t startTime;

int64_t lastLoop = -INTERVAL / 4;
RTC_DATA_ATTR unsigned long counter = 1;  // counter, that incremets for every Package, must be >= 1
bool wasDetected = false;                 // for PIR
bool SgpWasFound = false;


static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);


// LoRa license
uint32_t license[4] = {0xD5397DF0, 0x8573F814, 0x7A38C73D, 0x48E68607};


/** Function that returns random Byte
 * AES Library Function seems broken
 * @return returns a random Number as Byte between 0..255
 */
byte randByte() { return (byte)random(256); }

/** Generate IV (once)
 * needs to be repeated before every encryption
 * @param iv provide byte Arr of size N_BLOCK, new IV is returned
 */
void genIV(byte* iv) {
    Serial.println("gen_iv()");
    for (int i = 0; i < N_BLOCK; i++) {
        iv[i] = randByte();
    }
    // Serial.println(encrypt(strdup(plaintext.c_str()), plaintext.length(), aesIV));
}

/** encryption Method
 *
 * This function encrypts a byte msg and returns the cipherMsg
 * @param msg byte pointer for the message that needs to be encrypted
 * @param msgLen the length of the message
 * @param encMsg pointer to the encrypted message with the length of 'cipherLen'
 *               needs to have the right length already
 * @param cipherLen the length of the new enrypted message, needs to be provided
 * @param iv provide byte Arr of size N_BLOCK, new IV is returned
 * @return nothing is returned, the encMsg is returned as pointer
 */
void encrypt(byte* msg, uint16_t msgLen, byte* encMsg, int cipherLen, byte* iv) {
    genIV(iv);

    // for some reason 'do_aes_encrypt' changes the IV
    // copy can be used for enc
    byte thisIV[N_BLOCK] = {0};
    memcpy(thisIV, iv, N_BLOCK);

    // byte encrypted[cipherLen];
    AES thisAES;
    thisAES.setPadMode(paddingMode::CMS);

    thisAES.do_aes_encrypt(msg, msgLen, encMsg, AES_KEY, AES_KEY_SIZE, thisIV);  // does encryption

    /**
    Serial.print("encrypt2 Cipher: ");
    for (int i = 0; i < cipherLen; i++) {
        // uint8_t intVal = (int8_t)encrypted[i];
        // encMsg[i] = intVal;
        Serial.print(encMsg[i]);
        Serial.print("/");
    }
    Serial.println("");
    */
}

// calls the functions to encode the measurementData and writes it to the LoRa packet
void doAES() {
    AES aes;

    /** CMS is the same as PKCS#5 and PKCS#7
     *  BlockSize is in this Case N_BLOCK (16)
     *  every Pack has Num of Pad Bytes added to the end
     *  N_BLOCK -1 Bytes: ..01
     *  N_BLOCK -4 Byte: ..04 04 04 04
     *  N_BLOCK Bytes: 16 16... -> a new Packet is added which is filled with the Num of N_BLOCK (16)
     */
    aes.setPadMode(paddingMode::CMS);

    int cipherLen =
        aes.get_padded_len(MEASUREMENT_DATA_LENGTH);  // gets the length of encrypted Data after padding
    byte encrypted[cipherLen] = {0};                  // stores the encrypted data
    byte iv[N_BLOCK] = {0};                           // creates an empty IV
    encrypt(measurementData, MEASUREMENT_DATA_LENGTH, encrypted, cipherLen, iv);  // encrypts data

    memcpy(LoRaPacket, iv, N_BLOCK);
    memcpy(LoRaPacket + N_BLOCK, encrypted, cipherLen);
}

// initializes the measurement with 0 or with the constants
void initMeasurement() {
    measurement.protocolID = PROTOCOL_ID;
    measurement.senderID = SENDER_ID;
    measurement.verificationID = VERIFICATION_ID;
    measurement.counter = counter;
    measurement.batteryCharge = 0;
    isActive.batteryCharge = true;
    measurement.temperature = 0;
    isActive.temperature = true;
    measurement.humidity = 0;
    isActive.humidity = true;
    measurement.movement = 0;
    isActive.movement = false;
    measurement.tvoc = 0;
    isActive.tvoc = true;
    measurement.co2 = 0;
    isActive.co2 = true;
    // measurement.water = 0;
    isActive.water = false;
    // measurement.brightness = 0;
    isActive.brightness = false;
    // measurement.soundLevel = 0;
    isActive.soundLevel = false;
    // measurement.airPressure = 0;
    isActive.airPressure = false;
}

// encodes the measurement struct into an array with 15 bytes
void encodeMeasurement() {
    // ProtocolID  6bit
    measurementData[0] = (byte)(measurement.protocolID << 2);

    // Sender ID 9bit
    measurementData[0] += (byte)(measurement.senderID >> 6);
    measurementData[1] = (byte)((measurement.senderID % (1 << 7)) << 1);

    // Verification ID 12bit
    measurementData[1] += (byte)(measurement.verificationID >> 11);             // 1. 1bit
    measurementData[2] = (byte)((measurement.verificationID >> 3) % (1 << 8));  // 2. 8bits
    measurementData[3] = (byte)((measurement.verificationID % (1 << 3)) << 5);  // 3. 3bits

    // counter 20bit
    measurementData[3] += (byte)(measurement.counter >> 15);
    measurementData[4] = (byte)((measurement.counter >> 7) % (1 << 8));
    measurementData[5] = (byte)((measurement.counter % (1 << 7)) << 1);

    // battery charge 7bit
    uint8_t thisBatteryCharge = 0;
    if (isActive.batteryCharge) {
        thisBatteryCharge = measurement.batteryCharge + 1;
    }
    measurementData[5] += (byte)(thisBatteryCharge >> 6);
    measurementData[6] = (byte)((thisBatteryCharge % (1 << 6)) << 2);

    // Temperature 10bit
    uint16_t thisTemperature = 0;
    if (isActive.temperature) {
        float thisTemperatureF = measurement.temperature;
        if (thisTemperatureF < -35.0) {
            thisTemperature = 0;
        } else if (thisTemperatureF >= 67.2) {
            thisTemperature = 1022;  // highest allowed value
        } else {
            thisTemperatureF += 35.0;                      // start value
            thisTemperatureF *= 10;                        // for precision
            thisTemperatureF += 0.5;                       // for rounding
            thisTemperature = (uint16_t)thisTemperatureF;  // convert to integer
        }
        thisTemperature += 1;  // for active
    }
    measurementData[6] += (byte)(thisTemperature >> 8);
    measurementData[7] = (byte)(thisTemperature % (1 << 8));

    // Humidity 8bit
    uint8_t thisHumidity = 0;
    if (isActive.humidity) {
        float thisHumidityF = measurement.humidity;
        if (thisHumidityF < 0) {
            thisHumidityF = 0;
        } else if (thisHumidityF > 100) {
            thisHumidityF = 200;
        }
        thisHumidityF *= 2;
        thisHumidityF += 0.5;
        thisHumidity = (uint8_t)thisHumidityF;
        thisHumidity += 1;  // for active
    }
    measurementData[8] = thisHumidity;

    // Movement 2bit
    uint8_t thisMovement = 0;
    if (isActive.movement) {
        if (measurement.movement) {
            thisMovement = 3;
        } else {
            thisMovement = 1;
        }
    }
    measurementData[9] = thisMovement << 6;
    Serial.print("Measurement Arr Mov: ");
    Serial.println(measurementData[9]);

    // TVOC 9bit
    uint32_t thisTVOC = 0;
    if (isActive.tvoc) {
        float thisTVOC_F = (float)measurement.tvoc;
        if (thisTVOC_F < 0) {
            thisTVOC = 0;
        } else if (thisTVOC_F > 25800) {
            thisTVOC = 25800;
        }

        /*
        limits and resolution are in ug/m^3
          sendVal | lowLimit  | UpLimit  |  resolution
        ----------+-----------+----------+------------
          0 - 150 |        0  |    3000  |          20
        151 - 351 |     3001  |   10000  |          35
        352 - 510 |    10001  |   25800  |         100
        */
        if (thisTVOC_F < 3000) {
            thisTVOC_F = thisTVOC_F / 20;
        } else if (thisTVOC_F < 10000) {
            thisTVOC_F = 150 + ((thisTVOC_F - 3000) / 35);
        } else if (thisTVOC_F < 25800) {
            thisTVOC_F = 351 + ((thisTVOC_F - 10000) / 100);
        }
        thisTVOC_F += 0.5;  // for Rounding
        thisTVOC = (uint16_t)thisTVOC_F;
        thisTVOC += 1;  // for active
    }
    measurementData[9] += (byte)(thisTVOC >> 3);
    measurementData[10] = (byte)((thisTVOC % (1 << 3)) << 5);

    // CO2 10bit
    uint16_t thisCO2 = 0;
    if (isActive.co2) {
        thisCO2 = measurement.co2;
        if (thisCO2 < 400) {
            thisCO2 = 400;  // should not happen
        } else if (thisCO2 > 10620) {
            thisCO2 = 10620;
        }
        thisCO2 -= 400;
        thisCO2 += 5;   // for rounding
        thisCO2 /= 10;  // scale down
        thisCO2 += 1;   // for activation
    }
    measurementData[10] += (byte)(thisCO2 >> 5);
    measurementData[11] = (byte)((thisCO2 % (1 << 5)) << 3);

    // Water 3bit
    uint8_t thisWater = 0;
    if (isActive.water) {
        // do Watercode here
    }
    measurementData[11] += thisWater;

    // Brightness 9bit
    uint16_t thisBrigthness = 0;
    if (isActive.brightness) {
        float thisBrigthnessF = measurement.brightness;
        if (thisBrigthnessF < 0) {
            thisBrigthnessF = 0;
        } else if (thisBrigthnessF > 4080) {
            thisBrigthnessF = 4080;  // 1020
        }
        thisBrigthnessF /= 2;  //  scale down
        thisBrigthnessF += 0.5;
        thisBrigthness = (uint16_t)thisBrigthnessF;
        thisBrigthness += 1;
        Serial.print("thisBrigthness: ");
        Serial.println(thisBrigthness);
    }
    measurementData[12] = (byte)(thisBrigthness >> 1);
    measurementData[13] = (byte)((thisBrigthness % (1 << 1)) << 7);

    // Sound level 8bit
    uint8_t thisSoundLevel = 0;
    if (isActive.soundLevel) {
        float thisSoundLevelF = measurement.soundLevel;
        if (thisSoundLevelF < 0) {
            thisSoundLevel = 0;
        } else if (thisSoundLevelF > 254) {
            thisSoundLevelF = 254;
        }
        thisSoundLevelF += 0.5;
        thisSoundLevel = (uint8_t)thisSoundLevelF;
        thisSoundLevel += 1;
    }
    measurementData[13] += (thisSoundLevel >> 1);
    measurementData[14] = ((thisSoundLevel % (1 << 1)) << 7);

    // Air Pressure 7bit
    uint8_t thisAirPressure = 0;
    if (isActive.airPressure) {
        float thisAirPressureF = measurement.airPressure;
        if (thisAirPressureF < 950) {
            thisAirPressureF = 950;
        } else if (thisAirPressureF > 1074) {
            thisAirPressureF = 1074;
        }
        thisAirPressureF -= 950;  // starts measuring at 950 hPa
        thisAirPressureF += 0.5;
        thisAirPressure = (uint8_t)thisAirPressureF;
        thisAirPressure += 1;
    }
    measurementData[14] += thisAirPressure;
}


/* return absolute humidity [mg/m^3] with approximation formula
 * @param temperature [°C]
 * @param humidity [%RH]
 */
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity =
        216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) /
                  (273.15f + temperature));                                                     // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);  // [mg/m^3]
    return absoluteHumidityScaled;
}


/**
 * Function that converts the measured TVOC Values of the SGP30
 * from ppb (Parts per billion) to ug/m^3 micro gramm per cubic meter
 * molar mass of TVOC is 110 g/mol
 * @param tvoc_ppb TVOC measured in ppb
 * @return return TVOC measured in ug/m^3
 */
uint32_t convertTVOCto_ug(float tvoc_ppb) {
    uint16_t M_gas_mix = 110;
    float temp;       // temperature in Kelvin
    float R = 8.314;  // universal gas constant
    float pres;       // air Pressure in kPa
    if (isActive.temperature) {
        temp = measurement.temperature;
    } else {
        temp = 22.0;  // as Default Value
    }
    temp += 273.15;  // convert to Kelvin
    if (isActive.airPressure) {
        pres = measurement.airPressure / 10;  // to convert hPa to kPa
    } else {
        pres = 101.3;  // default air Pressure on sea level
    }

    float Vm = (R * temp) / pres;  // molar volume

    // tvoc in ug/m^3; +0.5 for rounding
    uint32_t tvoc_ug = (int)((M_gas_mix / Vm) * tvoc_ppb) + 0.5;
    return tvoc_ug;
}


// function that incerements the counter
// should be called after each send
void incCounter() {
    if (counter >= 1048575) {
        // if the counter reaches the 20bit border
        // should be in about 10 years if a message is send every 5 minutes
        counter = 1;
    } else {
        counter += 1;
    }
}

// reads the batteryCharge and writes it
void readBatteryCharge() {
    uint16_t anRead = analogRead(ADC_PIN) * VOLTAGE_MULTIPLIER;
    float voltage2 = 1.0303 * 3.3 * anRead / 4095;     // voltage at voltage divider V2
    float batteryVoltage = voltage2 * (R1 + R2) / R2;  // calculation of voltage divider V1
    Serial.print("Voltage: ");
    Serial.println(batteryVoltage);
    float charge_F = ((batteryVoltage - VOLTAGE_EMPTY) * 100) / (VOLTAGE_FULL - VOLTAGE_EMPTY);
    if (charge_F < 0) {
        charge_F = 0;
    } else if (charge_F > 100) {
        charge_F = 100;
    }
    uint8_t charge = (float)(charge_F + 0.5);
    measurement.batteryCharge = charge;

    Serial.print("Battery: ");
    Serial.print(charge);
    Serial.print(" anRead: ");
    Serial.print(anRead);
    Serial.print("  Brigthness: ");
    Serial.println(measurement.brightness);
}

// reads the Temperature and the Humidity
void readTempAndHum() {
    measurement.temperature = dht11.readTemperature();
    measurement.humidity = dht11.readHumidity();

    if (isnan(measurement.temperature) || isnan(measurement.humidity)) {
        Serial.println("Failed to read from DHT sensor!");
        isActive.temperature = false;
        isActive.humidity = false;
        return;
    }
    Serial.print("Temperatur: ");
    Serial.print(measurement.temperature);
    Serial.print("°C  Luftfeuchtigkeit: ");
    Serial.print(measurement.humidity);
    Serial.println("%");
}

// reads the movement
void readMovement() {
    if (counter == 1) {
        startTimer(60);
    }
    isActive.movement = true;
    bool movement = false;
    uint32_t thisCounter = 0;
    uint32_t starterTime = millis();
    Serial.println("scanning movement...");
    while (!movement and millis() - starterTime < WAIT_TILL_MOVEMENT) {
        movement = digitalRead(PIR_Pin) == HIGH;
        thisCounter++;
        delay(500);
    }
    Serial.print("Mov Counter: ");
    Serial.println(thisCounter);
    if (movement) {
        // digitalWrite(LED_PIN, HIGH);
        measurement.movement = true;
        // Serial.println("Movement");
    } else {
        // digitalWrite(LED_PIN, LOW);
        measurement.movement = false;
        // Serial.println("NOOOO Movement");
    }
    // digitalWrite(PIR_POWER_PIN, LOW);
}


// reads the CO2 and TVOC values
void readTvocAndCo2() {
    isActive.tvoc = true;
    isActive.co2 = true;

    if (isActive.temperature && isActive.humidity) {
        float temperature = measurement.temperature;  // [°C]
        float humidity = measurement.humidity;        // [%RH]
        sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));
    }
    if (!sgp.IAQmeasure()) {
        Serial.println("Measurement failed");
        isActive.tvoc = false;
        isActive.co2 = false;
        return;
    }
    measurement.tvoc = convertTVOCto_ug(sgp.TVOC);
    measurement.co2 = sgp.eCO2;
    Serial.print("eCO2: ");
    Serial.print(measurement.co2);
    Serial.print(" ppm\tTVOC: ");
    Serial.print(measurement.tvoc);
    Serial.println(" ug/m^3");
    sgp.softReset();
}

// reads all the Sensors
void readAllSensorData() {
    Serial.println("reading Sensors...");
    // readBatteryCharge();  // must be done before the other sensors go live
    readTempAndHum();
    readTvocAndCo2();

    // turns off Power for sensors
    digitalWrite(BOARD_POWER_PIN, LOW);
}

// sets all the values, that need to be reread and recomputed to 0
// generates a new IV
void resetBeforeSend() {
    memset(LoRaPacket, 0, LoRa_PACKET_LENGTH);
    memset(measurementData, 0, MEASUREMENT_DATA_LENGTH);
}

/** Function that prints the values of an array with a specified Seperator
 * @param toPrint Array that should be printed
 * @param size size of array
 * @param sep seperator that should be used between elements
 */
void printArr(byte* toPrint, int size, char sep) {
    for (int i = 0; i < size; i++) {
        Serial.print(toPrint[i]);
        Serial.print(sep);
    }
    Serial.println("");
}

/** Function, that delays the code for 'time' Seconds
 * and prints every 'interval' the left amount of wait time
 * @param time number of seconds to wait
 */
void startTimer(unsigned int time) {
    unsigned int interval = 5;
    unsigned int h = time / interval;
    for (int i = 0; i < h; i++) {
        delay(interval * 1000);
        Serial.print((i + 1) * interval);
        Serial.print("/");
        Serial.println(time);
    }
    uint32_t leftOnTheClock = time - (h * interval);
    if (leftOnTheClock > 0) {
        delay(leftOnTheClock * 1000);
    }
}

void setupTxRadio(void) {
    // RadioEvents.TxDone = OnTxDone;
    // RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON,
                      3000);
}

void sendPackage() { Radio.Send(LoRaPacket, sizeof(LoRaPacket)); }


void setup() {
    // put your setup code here, to run once:
    startTime = millis();
    Wire.begin();
    Serial.begin(115200);
    while (!Serial)
        ;  // wait for serial port


    randomSeed(ESP.getEfuseMac());
    initMeasurement();

    // for Battery Reading
    adcAttachPin(ADC_PIN);
    analogSetClockDiv(255);  // 1338mS

    Serial.println("Hi, init...");
    pinMode(PIR_Pin, INPUT_PULLDOWN);  // Pin Mode for motion sensor

    readBatteryCharge();  // must be done before the other sensors go live to get more accurate readings


    // activate the Power for the the sensors
    pinMode(BOARD_POWER_PIN, OUTPUT);
    digitalWrite(BOARD_POWER_PIN, HIGH);

    delay(500);


    // for SGP30 CO2 TVOC sensor
    if (!sgp.begin()) {
        Serial.println("Sensor not found :(");
        SgpWasFound = false;
    } else {
        SgpWasFound = true;
    }
    Serial.print("Found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);

    // Sets baseline for CO2 and TVOC
    // different for every sensor
    sgp.setIAQBaseline(CO2_BASE, TVOC_BASE);
    uint32_t SGP_startTime = millis();

    // for Temperature / Humidity Sensor
    dht11.begin();
    delay(3000);
    readMovement();

    if (SgpWasFound) {
        long additionalSgpWaitTime_Sec = (16E3 - (millis() - SGP_startTime)) / 1000;
        if (additionalSgpWaitTime_Sec > 0) {
            Serial.print("Additional Wait Time after PIR for SGP30 [sec]: ");
            Serial.println(additionalSgpWaitTime_Sec);
            startTimer(additionalSgpWaitTime_Sec);  // for SGP30 Sensor
        }
        readAllSensorData();
    } else {
        readTempAndHum();
    }

    // lastLoop = millis();
    encodeMeasurement();
    doAES();

    // for LoRa
    SPI.begin(SCK, MISO, MOSI, SS);
    Mcu.init(SS, RST_LoRa, DIO0, DIO1, license);
    setupTxRadio();

    Serial.print("Measurement: ");
    printArr(measurementData, MEASUREMENT_DATA_LENGTH, '/');
    Serial.print("LoRa: ");
    printArr(LoRaPacket, LoRa_PACKET_LENGTH, '/');
    Serial.println("Sending...");
    sendPackage();
    Serial.println("Sending complete");

    delay(1000);  // waiting for completion of LoRa sending
    incCounter();
    int64_t timeToSleep = INTERVAL - (millis() - startTime);
    if (timeToSleep < 0) {
        Serial.print("too short for Sleep");
        setup();
    }
    Serial.print("can go sleeping");
    esp_sleep_enable_timer_wakeup(timeToSleep * mS_TO_uS_FACTOR);
    esp_deep_sleep_start();
}

void loop() {
    // will never be executed
}
