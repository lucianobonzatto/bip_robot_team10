# In this file you can place the functions you write for your robot
import math

def readIRSensors(analog_pins):
    '''
    Read the IR sensors using the AD converter and return a list with inverted values:
    white = 1; 4096 = black
    '''
    return [4096 - value.read() for value in analog_pins]


def get_wheels_speed(encoderValues, oldEncoderValues, PULSES_PER_TURN, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    # Calculate the change in angular position of the wheels:
    ang_diff_l = 2*math.pi*(encoderValues[0] - oldEncoderValues[0])/PULSES_PER_TURN
    ang_diff_r = 2*math.pi*(encoderValues[1] - oldEncoderValues[1])/PULSES_PER_TURN

    # Calculate the angular speeds:
    wl = ang_diff_l/delta_t
    wr = ang_diff_r/delta_t

    return wl, wr


def test_motors(motor1, motor2, encoder1, encoder2):
    '''
    Sequence of movements to test the motors at different speeds and directions.
    '''
    from time import sleep

    print("Left motor")
    print(encoder2.value(), encoder1.value())
    motor1.forward(60)
    sleep(1)
    motor1.stop()
    print(encoder2.value(), encoder1.value())

    sleep(1)
    print(encoder2.value(), encoder1.value())
    motor1.backwards(90)
    sleep(1)
    motor1.stop()
    print(encoder2.value(), encoder1.value())

    sleep(2)

    print("Right motor")
    print(encoder2.value(), encoder1.value())
    motor2.forward(60)
    sleep(1)
    motor2.stop()
    print(encoder2.value(), encoder1.value())

    sleep(1)
    print(encoder2.value(), encoder1.value())
    motor2.backwards(90)
    sleep(1)
    motor2.stop()
    print(encoder2.value(), encoder1.value())

    sleep(2)
    print(encoder2.value(), encoder1.value())


