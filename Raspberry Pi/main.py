"""
Created by Christian Czepluch
"""


from datetime import datetime, timedelta
import paho.mqtt.client as mqtt
import time
import json
import pyowm
from influxdb import InfluxDBClient
import LoRa_Packet as LP
import decipher as dcy
import decoder as dec
import owm as owm
import threading


SETTINGS_NAME = "settings.json"

"""
The following Settings are overwritten by the 'SETTINGS_NAME' config file
"""
MQTT_IP = "localhost"  # ip of MQTT Broker
MQTT_PORT = 1883  # Port of MQTT Broker
IGNORE_TIME = 10  # ignores tuples within this amount of seconds
KEY = b'\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30\x30'
INFLUX_HOST = "localhost"
INFLUX_PORT = 8086
INFLUX_USERNAME = "pi"
INFLUX_PW = 'pi'
INFLUX_DATA_BASE = 'TestDB'
INFLUX_MEASUREMENTS_MEASUREMENT = "testLuftqualitaet"
INFLUX_META_MEASURMENT = "testMeta"
ext_Weather_API_Key = ""
ext_Weather_Location = "Wedel,DE"
ext_Weather_Table = "Wetter"
ext_Weather_enable = True

SETTINGS = {
    "AES-Key": [142, 194, 164, 255, 161, 160, 178, 156, 20,
                236, 200, 109, 60, 5, 82, 54],
    "AES-Key2": [48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48],
    "MQTT_IP": "localhost",
    "MQTT_Port": 1883,
    "IgnoreTime": 10,
    "Influx": {
        "IP": "localhost",
        "Port": 8086,
        "username": "pi",
        "pw": "pi",
        "DB": "TestDB",
        "measurementTable": "testLuftqualitaet",
        "metaTable": "testMeta"
    },
    "ExternalWeather": {
        "API-Key": "",
        "Location": "Wedel,DE",
        "WeatherTable": "Wetter",
        "enable": True
    },
    "Protocol": {
        "1": {
            "Sender": {
                "1": {
                    "verificationID": 2134,
                    "name": "HS1",
                    "interval": 450,
                    "lastMessage": {
                        "time": 1618146382133,
                        "counter": 10
                    }
                },
                "2": {
                    "verificationID": 2134,
                    "name": "HS2",
                    "interval": 450,
                    "lastMessage": {
                        "time": 1618146382133,
                        "counter": 10
                    }
                },
                "3": {
                    "verificationID": 2134,
                    "name": "HS3",
                    "interval": 450,
                    "lastMessage": {
                        "time": 1618146382133,
                        "counter": 0
                    }
                }
            }
        }
    }
}


def importSettings():
    """
    Imports all the Settings from the settings.json File
    """
    global SETTINGS
    global MQTT_IP
    global MQTT_PORT
    global IGNORE_TIME
    global KEY
    global INFLUX_HOST
    global INFLUX_PORT
    global INFLUX_USERNAME
    global INFLUX_PW
    global INFLUX_DATA_BASE
    global INFLUX_MEASUREMENTS_MEASUREMENT
    global INFLUX_META_MEASURMENT
    global ext_Weather_API_Key
    global ext_Weather_Location
    global ext_Weather_Table
    global ext_Weather_enable
    try:
        with open(SETTINGS_NAME) as json_File:
            SETTINGS = json.load(json_File)
        MQTT_IP = SETTINGS["MQTT_IP"]
        MQTT_PORT = SETTINGS["MQTT_Port"]
        IGNORE_TIME = SETTINGS["IgnoreTime"]
        KEY = SETTINGS["AES-Key"]
        INFLUX_HOST = SETTINGS["Influx"]["IP"]
        INFLUX_PORT = SETTINGS["Influx"]["Port"]
        INFLUX_USERNAME = SETTINGS["Influx"]["username"]
        INFLUX_PW = SETTINGS["Influx"]["pw"]
        INFLUX_DATA_BASE = SETTINGS["Influx"]["DB"]
        INFLUX_MEASUREMENTS_MEASUREMENT = SETTINGS["Influx"]["measurementTable"]
        INFLUX_META_MEASURMENT = SETTINGS["Influx"]["metaTable"]
        ext_Weather_API_Key = SETTINGS["ExternalWeather"]["API-Key"]
        ext_Weather_Location = SETTINGS["ExternalWeather"]["Location"]
        ext_Weather_Table = SETTINGS["ExternalWeather"]["WeatherTable"]
        ext_Weather_enable = SETTINGS["ExternalWeather"]["enable"]

    except:
        errMsg = "\n    ERROR: please provide a '" + SETTINGS_NAME + "' file\n"
        exit(errMsg)


def saveSettings():
    """
    saves all the settings in case they where changed and writes them to the
    settings.json file
    """
    with open(SETTINGS_NAME, "w") as outfile:
        json.dump(SETTINGS, outfile, indent=4)
        print("Settings are saved")


def nowStr():
    now = datetime.now()
    return now.strftime("%Y-%m-%dT%H:%M:%SZ")


def StrArr_TO_IntARR(arr):
    # converts String Arrays to Int Arrays
    # returns empfty Array if there was a conversion error

    arrL = len(arr)
    newArr = [None] * arrL

    for i in range(0, arrL):
        try:
            newArr[i] = int(arr[i])
        except:
            print("conversion Error")
            return []
    return newArr


def sendToInflux(dataSet):
    """
    sends dataSet to the InfluxDB
    """
    print("Sending...")
    print(dataSet)
    influxClient.write_points(dataSet, time_precision="s")
    print("Sending Complete")


def sendInfluxMeta(thirdParty, tableName, loraPacket, decodedPacket=None):
    """
    generates MetaData for Influx and sends it
    """
    dictEntry = {
        "measurement": tableName,
        "tags": {},
        "fields": {}
    }
    dictEntry = loraPacket.genInfluxMeta(dictEntry)
    dictEntry["tags"]["Dritt_Sender"] = thirdParty

    if decodedPacket is not None:
        dictEntry = decodedPacket.genInfluxMeta(dictEntry)

    sendToInflux([dictEntry])


def updateSettings(decPacket):
    # updates the Settings.json file with the last send time and the last counter
    SETTINGS = decPacket.updateSettings()
    saveSettings()


def newLoRaPacket(packet):
    """
    handels new LoRa Packets
    """
    thirdPartyPacket = False  # determins if the Packet is from a third Party
    dictPacket = {
        "packet": packet.loraConcat,
        "time": packet.time
    }
    recentPackets.append(dictPacket)

    if packet.isRightLength:
        try:
            # if the Packet is the right size, it gets deciphered
            data = dcy.decipherC(bytearray(KEY), packet)
        except:
            print("Cannot decipher the LoRa Packet")
            thirdPartyPacket = True

        if not thirdPartyPacket and data.decipherble:
            thirdPartyPacket = False   # for now its not a third Party
        else:
            thirdPartyPacket = True
    else:
        thirdPartyPacket = True

    if not thirdPartyPacket:
        thisData = dec.getDecodedObj(
            data.decipheredData_int, packet.time, SETTINGS)  # data gets decoded
        thirdPartyPacket = thisData.thirdParty

        if not thirdPartyPacket:
            # if this is a valid LoRa Packet from a knwon node
            thisData.printComplete()
            sendToInflux(thisData.genInfluxMeasurements(
                INFLUX_MEASUREMENTS_MEASUREMENT))
            sendInfluxMeta(thirdPartyPacket,
                           INFLUX_META_MEASURMENT, packet, thisData)
            updateSettings(thisData)

    if thirdPartyPacket:
        print("third Party Packet")
        sendInfluxMeta(thirdPartyPacket, INFLUX_META_MEASURMENT, packet)


def delOldRecentPackets():
    """
    deletes old Recent Packets from cache
    """

    lengthOfRecent = len(recentPackets)
    now = time.time()
    i = 0
    while i < lengthOfRecent:
        #print(now - recentPackets[i]["time"])
        if now - recentPackets[i]["time"] > IGNORE_TIME:
            recentPackets.pop(i)
            i -= 1
            lengthOfRecent -= 1
        i += 1


def receivesPacket(packet):
    """ 
    handels incoming LoRa Packet when they arrive via MQTT
    """
    lengthOfRecent = len(recentPackets)

    if lengthOfRecent == 0:
        newLoRaPacket(packet)
    else:
        delOldRecentPackets()  # deletes old packets from cache
        lengthOfRecent = len(recentPackets)
        found = False

        for i in range(lengthOfRecent):
            # checks cache for duplicates Packets
            # these can occur if two LoRa Receivers receive the same packet and transfer them via MQTT
            if recentPackets[i]["packet"] == packet.loraConcat:
                found = True
                print("Old Packet")

        if not found:
            # first occurance of LoRa Packet within IGNORE_TIME
            newLoRaPacket(packet)


def on_message(client, userdata, msg):
    msg.payload = msg.payload.decode("utf-8")
    value = str(msg.payload)
    print("+------------------------------------------------------------------------+")
    print("|                                                                        |")
    print(nowStr() + " >>  " + msg.topic + ":  " + value + "\n")
    # print(msg.topic + " " + value)
    try:
        loraPacket = LP.LoRa_Packet(value)
    except:
        print("wrong value")
        loraPacket = None

    if loraPacket is not None:
        receivesPacket(loraPacket)

    # print(json.dumps(data, indent=4))

    print("|                                                                        |")
    print("+------------------------------------------------------------------------+")
    print("\n")


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc) + "\n")

    client.subscribe("LoRaPacket")


"""
def startExtWeather():
    return WeatherBackground(ext_Weather_API_Key, ext_Weather_Location, INFLUX_HOST, INFLUX_PORT,
                             INFLUX_USERNAME, INFLUX_PW, INFLUX_DATA_BASE, ext_Weather_Table)
"""

if __name__ == "__main__":

    isConnected = False  # connection to MQTT Broker
    mqttTimeOut = 5  # Seconds before Programm quits
    recentPackets = []

    importSettings()
    print("HI")
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    influxClient = InfluxDBClient(host=INFLUX_HOST, port=INFLUX_PORT,
                                  username=INFLUX_USERNAME, password=INFLUX_PW,
                                  ssl=False, verify_ssl=False)
    influxClient.switch_database(INFLUX_DATA_BASE)

    client.connect(MQTT_IP, MQTT_PORT, 60)
    if ext_Weather_enable:
        w = owm.WeatherBackground(ext_Weather_API_Key, ext_Weather_Location,
                                  INFLUX_HOST, INFLUX_PORT, INFLUX_USERNAME, INFLUX_PW,
                                  INFLUX_DATA_BASE, ext_Weather_Table, interval=600)

    client.loop_forever()
