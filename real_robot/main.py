# MIT License (MIT)
# Copyright (c) 2023 Felipe Martins
# https://opensource.org/licenses/MIT
#
# MicroPython example-code for the ESP32-based RobotAtFactory Lite robot.
# You can modify and build on this code to complete your assignment.

from machine import Pin, PWM, UART, ADC, Timer
from time import sleep
from rotary_irq_esp import RotaryIRQ
import math

# Import other functions and definitions used in this code:
from utils import *
from definitions import *

# Initializing peripherals, variables and pins:
#######################################################
# ## To use the L9110 motor driver, uncomment this part
from dcmotor import L9110 as DCMotor
pinA1A = PWM(Pin(ML_A), frequency)
pinA1B = PWM(Pin(ML_B), frequency)
pinB1A = PWM(Pin(MR_A), frequency)
pinB1B = PWM(Pin(MR_B), frequency)
left_motor = DCMotor(pinA1A, pinA1B, 0, 1023)
right_motor = DCMotor(pinB1A, pinB1B, 0, 1023)

left_motor.stop()
right_motor.stop()
#######################################################

# uart = UART(2, baudrate=115200, tx=TX, rx=RX)
led_board = Pin(BOARD_LED, Pin.OUT)				# Define ESP32 onboard LED
frequency = 15000  # PWM frequency
cycle_count = 0
delta_t = DELTA_T_ms / 1000

solenoid = PWM(Pin(SOLENOID_PIN), frequency)	# define PWM for solenoid pin
solenoid.duty(0)	# Turn off magnet solenoid
solenoid_state = False

IR1_pin = ADC(Pin(IR1))		# define ADC for line sensor pin IR1
IR2_pin = ADC(Pin(IR2))		# define ADC for line sensor pin IR2
IR3_pin = ADC(Pin(IR3))		# define ADC for line sensor pin IR3
IR4_pin = ADC(Pin(IR4))		# define ADC for line sensor pin IR4
IR5_pin = ADC(Pin(IR5))		# define ADC for line sensor pin IR5
analog_pins = [IR1_pin, IR2_pin, IR3_pin, IR4_pin, IR5_pin]

touchsw = Pin(TOUCHSW_pin, Pin.IN, Pin.PULL_UP)	# define touch switch pin with internal pull-up

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

flag = False
motor_test = False

def timerCallback(timer):
    ''' Timer callback: executed every periodic timer interval
    '''
    global oldEncoderValues
    global left_encoder_value
    global right_encoder_value
    global cycle_count
    
    encoderValues = [encoder_left.value(), encoder_right.value()]
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, PULSES_PER_TURN, delta_t)
    oldEncoderValues = [left_encoder_value, right_encoder_value]
    left_encoder_value = encoder_left.value()
    right_encoder_value = encoder_right.value()
    
    # Print line sensor values
    print(line_sensor_values, left_encoder_value, right_encoder_value, cycle_count, wl, wr)

    led_board.value(not led_board.value())		# Invert board LED
    cycle_count = 0

print("Click the switch to start.")
while touchsw.value() == True:
    # Switch value is False when clicked
    sleep(0.1)
    
print("Starting...")
sleep(1)

# Initiate Timer
timer = Timer(1)
timer.init(period=DELTA_T_ms, mode=Timer.PERIODIC, callback = timerCallback)


while True:
    cycle_count += 1
    
    # Update sensor readings
    line_sensor_values = readIRSensors(analog_pins)
    touch_sw_value = touchsw.value()
    
    # Turn solenoid ON/OFF according to the value of solenoid_state
    if led_board.value():
        solenoid.duty(1000)
        solenoid_state = True
    else:
        solenoid.duty(0)
        solenoid_state = False

    # Click the touch switch to turn motor test ON/OFF
    if not touchsw.value() and not flag:
        flag = True
        motor_test = True    
    if touchsw.value() and flag:
        flag = False
        
    # Run motor test sequence if motor_test is true
    if motor_test:
        # Stop the timer to execute motor test
        timer.deinit()
        print("\nMotor test. \n*** Hold the touch switch to cancel. ***\n")
        print("Initiating in... ", end='')
        sleep(1)
        print("3, ", end="")
        sleep(1)
        print("2, ", end="")
        sleep(1)
        print("1, ", end="")
        sleep(1)
        print("0.")
        if touchsw.value():
            test_motors(left_motor, right_motor, encoder_right, encoder_left)  # Test the motors
            left_motor.stop()
            right_motor.stop()
            print("Test finished. \n")
        else:
            print("Motor test aborted. \n")
            sleep(2)
        motor_test = False
        # Restart the timer
        timer.init(period=DELTA_T_ms, mode=Timer.PERIODIC, callback = timerCallback)


    
