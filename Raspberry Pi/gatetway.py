"""
Created by Christian Czepluch
"""

from subprocess import Popen, PIPE
import subprocess
import datetime
import paho.mqtt.client as mqtt
import time


IP = "localhost"  # ip of MQTT Broker
PORT = 1883  # Port of MQTT Broker
isConnected = False  # connection to MQTT Broker
mqttTimeOut = 5  # Seconds before Programm quits


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc) + "\n")
    global isConnected
    isConnected = True


def sendMQTT(topic, message):
    # sends single MQTT data
    client.publish(topic, message)


def sendMQTTs(topics, data):
    # sends only data if topic has the same length
    # returns -1 if error

    dataL = len(data)
    topicL = len(topics)

    if topicL != dataL:
        return -1
    for i in range(0, dataL):
        sendMQTT(topics[i], data[i])


def nowStr():
    now = datetime.datetime.now()
    return now.strftime("%Y-%m-%d %H:%M:%S")


def run(command):
    process = Popen(command, stdout=PIPE, shell=False)
    while True:
        line = process.stdout.readline().rstrip()
        if not line:
            break
        yield line


if __name__ == "__main__":
    # execute only if run as a script
    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect(IP, PORT, 60)
    client.loop_start()

    # f = open("logfile.log", "w")  # can read all messages
    #f.write("Starting Receiver..")
    # f.close()
    startTimeMqtt = time.time()

    while not isConnected and (time.time() - startTimeMqtt) < mqttTimeOut:
        print(isConnected)
        # wait for connect of MQTT
        time.sleep(0.2)

    if not isConnected:
        print("Couldn't connect to MQTT-Broker in time")
        exit

    for line in run(["./Receiver", "-f", "868100000", "-sf", "7"]):
        #f = open("logfile.log", "a")
        line = line.decode("utf-8")
        line = line.rstrip()
        dateStr = nowStr() + " >>  " + line
        #f.write(dateStr + "\n")
        # f.close()

        print(dateStr)
        # sends complete LoRa packet to Chanel "LoRaPacket"
        if line.startswith("Packet"):
            sendMQTT("LoRaPacket", line)

    client.loop_stop()
