from machine import Pin
import network
import socket
import time

class WiFiInterface:
    def __init__(self, ssid, password, server_ip, server_port):
        self.ssid = ssid
        self.password = password
        self.server_ip = server_ip
        self.server_port = server_port
        self.wifi = None
        self.sock = None
        self.led = Pin(2, Pin.OUT)

    def connect_to_wifi(self):
        self.wifi = network.WLAN(network.STA_IF)
        self.wifi.active(True)
        self.wifi.connect(self.ssid, self.password)

        while not self.wifi.isconnected():
            self.led.value(not self.led.value())
            time.sleep(1)

    def create_udp_server(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.server_port))

    def send_data(self, data):
        self.sock.sendto(data, (self.server_ip, self.server_port))

    def run(self):
        self.connect_to_wifi()
        print("Connected to Wi-Fi")
        print(self.wifi.ifconfig())
        self.create_udp_server()
