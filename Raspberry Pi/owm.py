"""
Created by Christian Czepluch
"""


import pyowm
import time
from datetime import datetime
import threading
from influxdb import InfluxDBClient


def nowStr():
    now = datetime.now()
    return now.strftime("%Y-%m-%dT%H:%M:%SZ")


def nowGMEStr():
    now = time.time()
    return timeToGMEStr(now)


def timeToGMEStr(time):
    return datetime.utcfromtimestamp(time).strftime("%Y-%m-%dT%H:%M:%SZ")


class WeatherBackground(object):
    """ Threading example class
    The run() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self, owmKey, city, influx_host, influx_port, influx_username,
                 influx_pw, influx_db, influx_measurement, interval=30):
        """
        :param owmKey API Key for Open Weather Map
        :param city City for WeatherData
        :param influx_host IP Address of Influx
        :param influx_port Port of Influx
        :param influx_username username for InfluxDB
        :param influx_pw Password for user for InfluxDB
        :param influx_db Name of Database where Weather is stored in
        :param influx_measurement name of Table
        :param interval interval of queries

        """
        self.interval = interval
        self.owmKey = owmKey
        self.city = city
        self.host = influx_host
        self.port = influx_port
        self.username = influx_username
        self.pw = influx_pw
        self.db = influx_db
        self.measurement = influx_measurement

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def run(self):
        """ Method that runs forever """
        while True:
            try:
                influxClient = InfluxDBClient(host=self.host, port=self.port,
                                              username=self.username, password=self.pw,
                                              ssl=False, verify_ssl=False)

                influxClient.switch_database(self.db)
                owm = pyowm.OWM(self.owmKey)
                mgr = owm.weather_manager()
                data = {}
                # try:
                observation = mgr.weather_at_place(self.city)
                w = observation.weather
                print("got weather")
                data = {
                    "measurement": self.measurement,
                    "tags": {
                        "Quelle": "OWM",
                        "Ort": self.city
                    },
                    "time": nowGMEStr(),
                    "fields": {}
                }
                fields = {
                    "Temperatur": float(round(w.temperature(unit="celsius")["temp"], 2)),
                    "Luftfeuchtigkeit": int(w.humidity),
                    "Luftdruck": int(w.barometric_pressure(unit="hPa")["press"]),
                    "Wind": float(round(w.wind()["speed"], 2)),
                    "Wolken": int(w.clouds)
                }
                rain = w.rain
                if "1h" in rain:
                    fields["Regen"] = float(round(rain["1h"], 2))
                data["fields"] = fields
                influxClient.write_points([data], time_precision="s")
                """
                except:
                    print("ERR could not fetch weather data")
                """
                print("finished")
                influxClient.close()
            except:
                print("ERR could not fetch weather data")

            time.sleep(self.interval)
