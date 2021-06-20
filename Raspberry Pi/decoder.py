"""
Created by Christian Czepluch
"""


import time
from datetime import datetime

# holds all Decoder Classes


def getDecodedObj(data, time, settings):
    # this function returns the right decoded Obj for the right ProtocolID
    # data is the IntArr after it is deciphered
    dataLen = len(data)
    thirdParty = True
    if dataLen > 1:
        protocolID = data[0] >> 2

        if protocolID == 1 and dataLen == 15:
            thirdParty = False
            return decodeID_1(data, protocolID, dataLen, thirdParty, time, settings)
        elif protocolID == 2 and dataLen == 15:
            # do Stuff for ProtocolID 2 and return right Class
            return decoder(data, protocolID, dataLen, thirdParty, time, settings)
        else:
            return decoder(data, -1, dataLen, thirdParty, time, settings)

    else:
        return(decoder(data, -1, thirdParty, time, settings))


def timeToGMEStr(time):
    return datetime.utcfromtimestamp(time).strftime("%Y-%m-%dT%H:%M:%SZ")


class decoder:
    def __init__(self, data, protocolID, dataLen, thirdParty, time, settings):
        self.rawData = []
        self.protocolID = protocolID
        self.thirdParty = thirdParty
        self.settings = settings
        self.dataLen = dataLen
        self.time = time


class decodeID_1(decoder):

    def __init__(self, msg, protocolID, dataLen, thirdParty, time, settings):
        super().__init__(msg, protocolID, dataLen, thirdParty, time, settings)

        # to make sure that the msg always has the desired length
        assert dataLen == 15

        #
        #
        # self.protocolID = msg[0] >> 2  # first 6bits are ProtocolID

        #
        # SenderID  9bit
        sndr1 = msg[0] % (1 << 2)  # 1. part  last 2bits of element
        sndr2 = msg[1] >> 1  # 2. part first 7bits of elemnt
        self.sender = (sndr1 << 7) + sndr2

        #
        # VerificationID  12bit
        ver1 = (msg[1] % (1 << 1))  # last 1bit are part of element
        ver2 = (msg[2])  # 2. part whole element
        ver3 = msg[3] >> 5  # 3. last part first 3bits of element
        self.verificationID = (ver1 << 11) + (ver2 << 3) + ver3

        #
        # Counter  20bit
        cnt1 = msg[3] % (1 << 5)  # 1. part  last 5bits of element
        cnt2 = msg[4]  # 2. part
        cnt3 = msg[5] >> 1  # 3. part  first 7bits of element
        self.counter = (cnt1 << 15) + (cnt2 << 7) + cnt3

        try:
            __lastSender = self.settings["Protocol"][str(
                self.protocolID)]["Sender"][str(self.sender)]
            if __lastSender["verificationID"] == self.verificationID and (self.counter > __lastSender["lastMessage"]["counter"] or self.counter < 5):
                # checks if the counter is new or < 5
                self.thirdParty = False
                self.duration = self.time - __lastSender["lastMessage"]["time"]
                self.name = __lastSender["name"]
            else:
                self.thirdParty = True
        except:
            self.thirdParty = True

        if not self.thirdParty:

            #
            # Battery charge  7bit
            btr1 = msg[5] % (1 << 1)  # 1. part  last 1bit of element
            btr2 = msg[6] >> 2  # 2. part  first 6bits of element

            # if 0 then value wasn't sent
            if btr1 == btr2 == 0:
                self.batteryCharge = None
            else:
                self.batteryCharge = round((btr1 << 6) + btr2 - 1)

            #
            # Temperature  10bit
            temp1 = msg[6] % (1 << 2)  # 1. part  last 2bits of element
            temp2 = msg[7]  # 2. part  whole element

            if temp1 == temp2 == 0:
                self.temperature = None
            else:
                thisTemp = (temp1 << 8) + temp2 - 1
                thisTemp = -35 + (thisTemp / 10)
                self.temperature = float(round(thisTemp, 1))

            #
            # Humidity  8bit
            hum = msg[8]  # whole element

            if hum == 0:
                self.humidity = None
            else:
                thisHum = (hum - 1) / 2
                if thisHum > 100:
                    self.humidity = None
                else:
                    self.humidity = float(round(thisHum, 1))

            #
            # movement 2bits are used
            #
            # the last bit defines if this value was transmitted
            # the first bit defines if there was movement
            # [00] 0 sensor deactive = 'None'
            # [01] 1 sensor active, no movement = 0
            # [11] 3 sensor active, movement = 1
            # [10] 2 invalid = 'None'
            mov = msg[9] >> 6  # 1. part  first 2bit of element

            if mov in (0, 2):
                self.movement = None
            elif mov == 1:
                self.movement = 0
            else:
                __interval = self.settings["Protocol"][str(
                    self.protocolID)]["Sender"][str(self.sender)]["interval"]
                if __interval > self.duration:
                    __interval = self.duration
                self.movement = round(__interval)

            #
            # TVOC total volatile Compund  9bit
            tv1 = msg[9] % (1 << 6)  # 1. part  last 6bits of element
            tv2 = msg[10] >> 5  # 2. part  first 3bits of element
            thisTVOC = (tv1 << 3) + tv2
            if thisTVOC == 0:
                self.TVOC = None
            else:
                thisTVOC -= 1  # to remove the isActive part

                """
                limits and resolution are in ug/m^3

                  sendVal | lowLimit  | UpLimit  |  resolution
                ----------+-----------+----------+------------
                  0 - 150 |        0  |    3000  |          20
                151 - 351 |     3001  |   10000  |          35
                352 - 510 |    10001  |   25800  |         100
                """

                if 0 <= thisTVOC <= 150:
                    self.TVOC = int(thisTVOC * 20)
                elif 150 < thisTVOC <= 351:
                    self.TVOC = 3000 + int((thisTVOC - 150) * 35)
                elif 351 < thisTVOC <= 510:
                    self.TVOC = 10000 + int((thisTVOC - 351) * 100)
                else:
                    # should never execute
                    self.TVOC = None

            #
            # CO2eq  10bit
            co21 = msg[10] % (1 << 5)  # 1. part  last 5bits of element
            co22 = msg[11] >> 3  # 2. part  first 5bits of element
            thisCO2 = (co21 << 5) + co22
            if thisCO2 == 0:
                self.CO2 = None
            else:
                self.CO2 = int(400 + ((thisCO2 - 1) * 10))

            #
            # Water  3bit
            thisWater = (msg[11] % (1 << 3))  # last 3 bits of element
            if thisWater == 0:
                self.water = None
            else:
                self.water = thisWater - 1

            #
            # Brightness  9bit
            lu1 = msg[12]  # 1. part  whole element
            lu2 = msg[13] >> 7  # 2. part  first 1bit of element
            thisLux = (lu1 << 1) + lu2
            if thisLux == 0:
                self.brightness = None
            else:
                self.brightness = (thisLux - 1) * 2

            #
            # Sound level  8bit
            snd1 = msg[13] % (1 << 7)  # 1. part  last 7bits of element
            snd2 = msg[14] >> 7  # 2. part  first 1bit of element
            thisSound = (snd1 << 1) + snd2
            if thisSound == 0:
                self.soundLevel = None
            else:
                self.soundLevel = (thisSound - 1)

            # atm Preassure  7bit
            thisPressure = msg[14] % (1 << 7)  # 1. part  last 7bits of element
            if thisPressure == 0:
                self.airPressure = None
            else:
                self.airPressure = 950 + (thisPressure - 1)

    # End init

    def updateSettings(self):
        self.settings["Protocol"][str(self.protocolID)]["Sender"][str(
            self.sender)]["lastMessage"]["time"] = self.time
        self.settings["Protocol"][str(self.protocolID)]["Sender"][str(
            self.sender)]["lastMessage"]["counter"] = self.counter
        return self.settings

    def genInfluxMeasurements(self, tableName):
        # generates a dict with all the elements that need to be stored in the Influx DB
        #__senderName = self.settings["Protocol"][str(self.protocolID)]["Sender"][str(self.sender)]["name"]
        dictEntry = {
            "measurement": tableName,
            "tags": {
                "Raum": self.name
            },
            "time": timeToGMEStr(self.time),
            "fields": {}
        }

        if self.temperature is not None:
            dictEntry["fields"]["Temperatur"] = self.temperature
        if self.humidity is not None:
            dictEntry["fields"]["Luftfeuchtigkeit"] = self.humidity
        if self.movement is not None:
            dictEntry["fields"]["Bewegung"] = self.movement
        if self.CO2 is not None:
            dictEntry["fields"]["CO2"] = self.CO2
        if self.TVOC is not None:
            dictEntry["fields"]["TVOC"] = self.TVOC
        if self.water is not None:
            dictEntry["fields"]["Wasserstand"] = self.water
        if self.brightness is not None:
            dictEntry["fields"]["Helligkeit"] = self.brightness
        if self.soundLevel is not None:
            dictEntry["fields"]["Lautstaerke"] = self.soundLevel
        if self.airPressure is not None:
            dictEntry["fields"]["Luftdruck"] = self.airPressure

        return [dictEntry]

    def genInfluxMeta(self, dictEntry):
        # dictEntry has to have already a bunch of data, just adding addiotonal info
        dictEntry["tags"]["Protocol_ID"] = self.protocolID
        dictEntry["tags"]["Sender_Nummer"] = self.sender
        dictEntry["tags"]["Raum"] = self.name
        if self.batteryCharge is not None:
            dictEntry["fields"]["Akkustand"] = self.batteryCharge
        return dictEntry

    def printComplete(self):
        #print("raw Msg: " + str(self.rawMSG))
        print("Protocol: " + str(self.protocolID))
        print("Sender Number: " + str(self.sender))
        print("Sender Name: " + str(self.name))
        print("VerifikationsID: " + str(self.verificationID))
        print("Counter: " + str(self.counter))
        if self.batteryCharge is not None:
            print("Akkustand: " + str(self.batteryCharge) + " %")
        if self.temperature is not None:
            print("Temperatur: " + str(self.temperature) + " °C")
        if self.humidity is not None:
            print("Luftfeuchtigkeit: " + str(self.humidity) + " %H")
        if self.airPressure is not None:
            print("Luftdruck: " + str(self.airPressure) + " hPa")
        if self.CO2 is not None:
            print("CO2: " + str(self.CO2) + " ppm")
        if self.TVOC is not None:
            print("TVOC: " + str(self.TVOC) + " ug/m^3")
        if self.movement is not None:
            if self.movement < 0.0001:
                print("Bewegung: NEIN")
            else:
                print("Bewegung: JA")
        if self.brightness is not None:
            print("Helligkeit: " + str(self.brightness) + " lux")
        if self.soundLevel is not None:
            print("Lautstärke: " + str(self.soundLevel) + " dB")
        if self.water is not None:
            print("Wassermelderstärke: " + str(self.water))
        print("")

    def strMSG(self):
        return str("raw Msg: " + str(self.rawMSG))

    def strProtocol(self):
        return str("Protocol: " + str(self.protocolID))

    def strVerificationID(self):
        return str("VerifikationsID: " + str(self.verificationID))

    def strSender(self):
        return str("Sender Nummer: " + str(self.sender))

    def strCounter(self):
        return str("Counter: " + str(self.counter))

    def strBatteryCharge(self):
        if self.batteryCharge is not None:
            return str("Akkustand: " + str(self.batteryCharge) + " %")
        else:
            return ""

    def strTemperature(self):
        return str("Temperatur: " + str(self.temperature) + " °C")

    def strHumidity(self):
        return str("Luftfeuchtigkeit: " + str(self.humidity) + " %H")

    def strAirPressure(self):
        return str("Luftdruck: " + str(self.airPressure) + " hPa")

    def strCO2(self):
        return str("CO2: " + str(self.CO2) + " ppm")

    def strTVOC(self):
        return str("TVOC: " + str(self.TVOC) + " ug/m^3")

    def strBrightness(self):
        return str("Helligkeit: " + str(self.brightness) + " lux")
