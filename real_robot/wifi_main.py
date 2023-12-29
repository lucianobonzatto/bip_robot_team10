from files.WiFiInterface import WiFiInterface
from files.definitions import *
from machine import Pin, ADC
import time

wifi_interface = WiFiInterface(wifi_ssid, wifi_password, server_ip, server_port)
wifi_interface.run()
analog_read = ADC(Pin(33))

for i in range(6000):
    x = "A1 " + str(analog_read.read()) +";A2 100;loop 1;"
    wifi_interface.send_data(x.encode())
    data = wifi_interface.receive_data()
    if data != None:
        print("Dados Recebidos:", data)
    time.sleep_ms(100)
