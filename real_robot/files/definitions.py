# This file contains definitions of pins and constants used in your code
BOARD_LED = 2    # ESP32 onboard LED

# Encoders: supply GND and 3V3
ENC1_A = 27  # green  Right motor encoder
ENC1_B = 14  # blue 
ENC2_A = 19  # green  Left motor encoder
ENC2_B = 23  # blue
PULSES_PER_TURN = 96  # Number of encoder pulses per revolution of the wheel

# MOTOR DRIVER: supply to GND and Vin

EN_LEFT = 26         # A-1A  Left enable
POSITIVE_LEFT = 17   # B-1B  Left Motor -
NEGATIVE_LEFT = 16   # B-1A  Left Motor +

EN_RIGHT = 25         # A-1B  Right enable
POSITIVE_RIGHT = 2   # B-1B  Right Motor -
NEGATIVE_RIGHT = 4   # B-1A  Right Motor +

SOLENOID_PIN = 13  # connect to Magnet board (supply the board with GND and +5V)
TOUCHSW_pin = 12   # switch it to ground
DELTA_T_ms = 40    # Cycle time in milliseconds

# IR line sensor: Connect to GND and 3V3
IR1 = 33    # Blue   IR1 IO33 - I've switched IR1 and IR2 because connections are inverted in my robot!
IR2 = 32    # Yellow IR2 IO32 - I've switched IR1 and IR2 because connections are inverted in my robot!
IR3 = 39    # Orange IR3 IO39
IR4 = 36    # Green  IR4 IO36
IR5 = 34    # White  IR5 IO34

# Network credentials - Replace by the correct ones!
wifi_ssid = "Vodafone-D25918"
wifi_password = "s98S4c8kqn"
server_ip = "192.168.1.134"
server_port = 4224

# Serial pins
TX = 10     # UART-TX pin
RX = 9      # UART-RX pin

# Robot Size
Wheel_Radius = 0.0205
Whell_Distance = 0.0565

