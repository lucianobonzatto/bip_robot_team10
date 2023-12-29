from machine import Pin, ADC
import time
from files.WiFiInterface import WiFiInterface

# Utilização
wifi_ssid = "Vodafone-D25918"
wifi_password = "s98S4c8kqn"
server_ip = "192.168.1.134"
server_port = 4224

wifi_interface = WiFiInterface(wifi_ssid, wifi_password, server_ip, server_port)
wifi_interface.run()
analog_read = ADC(Pin(33))

for i in range(6000):
  x = "A1 " + str(analog_read.read()) +";A2 100;loop 1;"
  wifi_interface.send_data(x.encode())
  time.sleep_ms(100)




