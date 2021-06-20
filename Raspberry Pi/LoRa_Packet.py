"""
Created by Christian Czepluch
"""


import time
from datetime import datetime, timedelta
import decipher as dc


def nowStr():
    now = datetime.now()
    return now.strftime("%Y-%m-%d %H:%M:%S")


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


def timeToGMEStr(time):
    return datetime.utcfromtimestamp(time).strftime("%Y-%m-%dT%H:%M:%SZ")


class LoRa_Packet:
    def __init__(self, msg):
        __packets = msg.replace(" ", "").split(",")
        __packetsLen = len(__packets)
        error = False

        if __packetsLen != 5:
            raise ValueError("some Value is wrong for LoRaPacket")

        if __packets[0].startswith("PacketRSSI:"):
            try:
                __thisPRssi = __packets[0].split(":")
                self.pRssi = int(__thisPRssi[1])
            except:
                raise ValueError("some Value is wrong for LoRaPacket")
        else:
            raise ValueError("some Value is wrong for LoRaPacket")

        if __packets[1].startswith("RSSI:"):
            try:
                __thisRssi = __packets[1].split(":")
                self.rssi = int(__thisRssi[1])
            except:
                raise ValueError("some Value is wrong for LoRaPacket")
        else:
            raise ValueError("some Value is wrong for LoRaPacket")

        if __packets[2].startswith("SNR:"):
            try:
                __thisSNR = __packets[2].split(":")
                self.snr = int(__thisSNR[1])
            except:
                raise ValueError("some Value is wrong for LoRaPacket")
        else:
            raise ValueError("some Value is wrong for LoRaPacket")

        if __packets[3].startswith("Length:"):
            try:
                __thisLength = __packets[3].split(":")
                self.packetLength = int(__thisLength[1])
            except:
                raise ValueError("some Value is wrong for LoRaPacket")
        else:
            raise ValueError("some Value is wrong for LoRaPacket")

        self.loraConcat = __packets[4].split(":")[1]
        self.loraMsg = __packets[4].split(":")[1].split("/")
        self.loraMsg.pop(-1)
        __loraIntArr = StrArr_TO_IntARR(self.loraMsg)
        if __loraIntArr is not None:
            self.loraPackage = __loraIntArr
        else:
            raise ValueError("some Value is wrong for LoRaPacket")

        self.time = time.time()
        self.timeStr = timeToGMEStr(self.time)
        self.isRightLength = self.checkRightLength()

        if self.isRightLength:
            self.IV_int = self.loraPackage[0:16]
            self.data_int = self.loraPackage[16:]
        else:
            self.IV_int = []
            self.data_int = []

    def checkRightLength(self):
        """
        checks if the packet consists of at least 32 Byte (IV + 16 Byte Data)
        or is a multiple of 16
        """
        if self.packetLength < 32 or self.packetLength % 16 != 0:
            return False
        else:
            return True

    def __str__(self):
        thisString = "Time: " + self.timeStr
        thisString += "  Packet RSSI: " + str(self.pRssi)
        thisString += "  RSSI: " + str(self.rssi)
        thisString += "  SNR: " + str(self.snr)
        thisString += "  Length: " + str(self.packetLength)
        thisString += "\nPacket: " + str(self.loraPackage)
        thisString += "\nIV:  " + str(self.IV_int)
        thisString += "\ndataInt: " + str(self.data_int)
        return thisString

    def genInfluxMeta(self, dictEntry):
        # dictEntry has to have already a bunch of data, just adding addiotonal info
        dictEntry["time"] = self.timeStr
        dictEntry["fields"] = {
            "RSSI": self.rssi,
            "Packet_RSSI": self.pRssi,
            "SNR": self.snr,
            "Paketlaenge": self.packetLength
        }
        return dictEntry
