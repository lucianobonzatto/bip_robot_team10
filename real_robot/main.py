# MIT License (MIT)
# Copyright (c) 2023 Felipe Martins
# https://opensource.org/licenses/MIT
#
# MicroPython example-code for the ESP32-based RobotAtFactory Lite robot.
# You can modify and build on this code to complete your assignment.

from machine import Pin, PWM, UART, ADC, Timer
from files.rotary_irq_esp import RotaryIRQ
from files.WiFiInterface import WiFiInterface
from files.robot import My_Robot
from files.definitions import *
import time

frequency = 15000  # PWM frequency
# uart = UART(2, baudrate=115200, tx=TX, rx=RX)
led_board = Pin(BOARD_LED, Pin.OUT)				# Define ESP32 onboard LED
cycle_count = 0
delta_t = DELTA_T_ms / 1000

#######################################################

## L9110 motor driver
# from files.dcmotor import L9110 as DCMotor
# pinA1A = PWM(Pin(ML_A), frequency)
# pinA1B = PWM(Pin(ML_B), frequency)
# pinB1A = PWM(Pin(MR_A), frequency)
# pinB1B = PWM(Pin(MR_B), frequency)
# left_motor = DCMotor(pinA1A, pinA1B, 0, 1023)
# right_motor = DCMotor(pinB1A, pinB1B, 0, 1023)

#######################################################
## L298 motor driver

from files.dcmotor import L298 as DCMotor
enable_l = PWM(Pin(EN_LEFT), frequency)
left_p = Pin(POSITIVE_LEFT, Pin.OUT)
left_n = Pin(NEGATIVE_LEFT, Pin.OUT)

enable_r = PWM(Pin(EN_RIGHT), frequency)
right_p = Pin(POSITIVE_RIGHT, Pin.OUT)
right_n = Pin(NEGATIVE_RIGHT, Pin.OUT)


left_motor = DCMotor(left_p, left_n, enable_l, 0, 1023)
right_motor = DCMotor(right_p, right_n, enable_r, 0, 1023)

left_motor.stop()
right_motor.stop()
#######################################################

## encoder driver
encoder_right = RotaryIRQ(pin_num_clk=ENC1_A, 
              pin_num_dt=ENC1_B, 
              min_val=0, 
              max_val=10000, 
              reverse=False, 
              range_mode=RotaryIRQ.RANGE_WRAP)	# Right wheel encoder: 966 ppr
encoder_left = RotaryIRQ(pin_num_clk=ENC2_A, 
              pin_num_dt=ENC2_B, 
              min_val=0, 
              max_val=10000, 
              reverse=True, 
              range_mode=RotaryIRQ.RANGE_WRAP)	# Left wheel encoder: 968 ppr
encoders = [encoder_right, encoder_left]

## switch driver
touchsw = Pin(TOUCHSW_pin, Pin.IN, Pin.PULL_UP)	# define touch switch pin with internal pull-up

## line sensor driver
IR1_pin = ADC(Pin(IR1))		# define ADC for line sensor pin IR1
IR2_pin = ADC(Pin(IR2))		# define ADC for line sensor pin IR2
IR3_pin = ADC(Pin(IR3))		# define ADC for line sensor pin IR3
IR4_pin = ADC(Pin(IR4))		# define ADC for line sensor pin IR4
IR5_pin = ADC(Pin(IR5))		# define ADC for line sensor pin IR5

IR1_pin.atten(ADC.ATTN_11DB)
IR2_pin.atten(ADC.ATTN_11DB)
IR3_pin.atten(ADC.ATTN_11DB)
IR4_pin.atten(ADC.ATTN_11DB)
IR5_pin.atten(ADC.ATTN_11DB)
analog_pins = [IR1_pin, IR2_pin, IR3_pin, IR4_pin, IR5_pin]

## solenoid driver
solenoid = PWM(Pin(SOLENOID_PIN), frequency)	# define PWM for solenoid pin
solenoid.duty(0)	# Turn off magnet solenoid

#######################################################

my_robot = My_Robot(analog_pins, left_motor, right_motor, encoders, touchsw, solenoid)
#wifi_interface = WiFiInterface(wifi_ssid, wifi_password, server_ip, server_port)
#wifi_interface.run()

#for i in range(6000):
#    x = "A1 " + str(analog_read.read()) +";A2 100;loop 1;"
#    wifi_interface.send_data(x.encode())
#    data = wifi_interface.receive_data()
#    if data != None:
#        print("Dados Recebidos:", data)
#    time.sleep_ms(100)
    
#######################################################

#print("Click the switch to start.")
#while touchsw.value() == True:
#    # Switch value is False when clicked
#    time.sleep(0.1)

print("Starting...")

    
while True:
    time.sleep(delta_t)
    cycle_count += 1
    led_board.value(not led_board.value())
    data = my_robot.update()
    data = data + "A1 " + str(cycle_count) + ";loop 1;"
    #wifi_interface.send_data(data.encode())
    
    line_sensor_values = my_robot.readIRSensors()
    touch_sw_value = my_robot.readSwitch()
    my_robot.writeSolenoid(True)
    print(touch_sw_value)
    print(line_sensor_values)
    print(data)
    
    # my_robot.test_motors()
    my_robot.follow_line()
    