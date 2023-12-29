import network
import socket
import time

# Replace these with your Wi-Fi credentials
wifi_ssid = "BIPES"
wifi_password = "#bipBremen#"

# Connect to Wi-Fi
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(wifi_ssid, wifi_password)

while not wifi.isconnected():
  time.sleep(1)

print("Connected to Wi-Fi")

# UDP server configuration
server_ip = "192.168.137.1"
server_port = 4224 #12345

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('',4224))
# Define the message to be sent

#message = "End"

from machine import Pin, ADC
analogread = ADC(Pin(33))
#analogread.attend(ADC.ATTN_11DB)

#try:
for i in range(6000):
  x = "A1 " + str(analogread.read()) +";A2 100;loop 1;"
  print(x)
  sock.sendto(x.encode(), (server_ip, server_port))
  time.sleep_ms(100)  
  #print("ok")

#finally:
#  sock.close()



