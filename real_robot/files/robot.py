from files.definitions import *
from files.PID_class import PID
from time import sleep
import math

class My_Robot:
    def __init__(self, ground_sensors, left_motor, right_motor, encoders, touchsw, solenoid):
        # actuators
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.solenoid = solenoid
        self.solenoid.duty(0)
        self.solenoid_state = False

        # sensors
        self.touchsw = touchsw
        self.ground_sensors = ground_sensors
        self.encoders = encoders

        # values
        self.old_encoders_values = [None, None]
        self.angle_tolerance = 0.05
        self.position_tolerance = 0.05

        # robot information
        self.delta_t = DELTA_T_ms / 1000
        self.whell_size = Wheel_Radius
        self.whell_distance = Whell_Distance
        self.odometry_position = [0.0, 0.0, 0.0]
        self.pulses_per_turn = PULSES_PER_TURN

        # controllers
        self.linear_controller = PID(0.1, 0.01, 0.01, self.delta_t, 0.12)
        self.angular_controller = PID(0.1, 0.01, 0.01, self.delta_t, 0.01)

        self.goto = [[0.1, 0.0], [0.1, 0.1]]
        self.index = 0

    def update(self):
        # read sensors
        encoderValues = [self.encoders[0].value(), self.encoders[1].value()]
        if(self.old_encoders_values[0] == None or self.old_encoders_values[1] == None):
            self.old_encoders_values = encoderValues
        
        data = "E1 " + str(encoderValues[0]) + ";" + \
            "E2 " + str(encoderValues[1]) + ";" + \
            "E3 " + str(self.old_encoders_values[0]) + ";" + \
            "E4 " + str(self.old_encoders_values[1]) + ";"

        # calc odometry
        data = data + self._update_position(encoderValues)
        self.old_encoders_values = encoderValues

        # print(self.old_encoders_values[0], self.old_encoders_values[1])
        # print(encoderValues[0], encoderValues[1])
        return data
        
    def followLine(self):
        line_sensor_values = self.readIRSensors()
        gs_extreme_left = line_sensor_values[0]
        gs_right = line_sensor_values[1].getValue()
        gs_center = line_sensor_values[2].getValue()
        gs_left = line_sensor_values[3].getValue()
        gs_extreme_right = line_sensor_values[4]
        
        base_speed = 0.5
        turn_speed = 0.2
        
        if(gs_extreme_left > LINE_THRESHOLD and gs_extreme_right > LINE_THRESHOLD):
            return True
        
        if gs_center > LINE_THRESHOLD:
            u = base_speed
            w = 0
        elif gs_left > LINE_THRESHOLD:
            u = base_speed
            w = turn_speed
        elif gs_right > LINE_THRESHOLD:
            u = base_speed
            w = -turn_speed
        else:
            u = 0
            w = 0
            
        self.sendVelocity(u, w)
        return False
        
    def rotateTo(self, target_angle):
        

        current_angle = self.odometry_position[2]
        angle_diff = target_angle - current_angle
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        if abs(angle_diff) < self.angle_tolerance:
            self.sendVelocity(0, 0)
            return True

        angle_velocity = self.angular_controller.update(angle_diff)
        self.sendVelocity(0, angle_velocity)
        return False
            
    def moveTo(self, target):
        wl = 0
        wr = 0
        x_err = target[0] - self.odometry_position[0]
        y_err = target[1] - self.odometry_position[1]
        dist_err = math.sqrt(x_err**2 + y_err**2)
        
        phi_d = math.arctan2(y_err,x_err)
        phi_err = phi_d - self.odometry_position[2]
        phi_err = math.arctan2(math.sin(phi_err), math.cos(phi_err))
        
        if dist_err > self.position_tolerance or phi_err > self.angle_tolerance:
            new_u = self.linear_controller.update(dist_err)
            new_w = self.angular_controller.update(phi_err)
            self.sendVelocity(new_u, new_w)
        else:
            return True
        return False

    def sendVelocity(self, u, w):
        wr, wl = self._calc_wheels_speed_to(u, w)
        wr = self._convert_speed_to_pwm(wr)
        wl = self._convert_speed_to_pwm(wl)
        
        if wl > 0:
            self.left_motor.forward(abs(wl))
        elif wl < 0:
            self.left_motor.backwards(abs(wl))
        else:
            self.left_motor.stop()
            
        if wr > 0:
            self.right_motor.forward(abs(wr))
        elif wr < 0:
            self.right_motor.backwards(abs(wr))
        else:
            self.right_motor.stop()
        
    def readSwitch(self):
        '''
        Read the Switch state:
        pressed = True; unpressed = False
        '''
        return not self.touchsw.value()

    def readIRSensors(self):
        '''
        Read the IR sensors using the AD converter and return a list with inverted values:
        white = 1; 4096 = black
        '''
        return [4096 - value.read() for value in self.ground_sensors]
    
    def readSolenoid(self):
        '''
        Read the solenoid state:
        on = True; off = False
        '''
        return self.solenoid_state
    
    def writeSolenoid(self, state):
        if state == True:
            self.solenoid.duty(255)	# Turn on magnet solenoid
            self.solenoid_state = True
            
        elif state == False:
            self.solenoid.duty(0)	# Turn off magnet solenoid
            self.solenoid_state = False
    
    def test_motors(self):
        '''
        Sequence of movements to test the motors at different speeds and directions.
        '''
        print("Left motor")
        print(self.encoders[0].getValue(), self.encoders[1].getValue())
        self.left_motor.forward(60)
        sleep(1)
        self.left_motor.stop()
        print(self.encoders[0].getValue(), self.encoders[1].getValue())

        sleep(1)
        print(self.encoders[0].getValue(), self.encoders[1].getValue())
        self.left_motor.backwards(90)
        sleep(1)
        self.left_motor.stop()
        print(self.encoders[0].getValue(), self.encoders[1].getValue())

        sleep(2)

        print("Right motor")
        print(self.encoders[0].getValue(), self.encoders[1].getValue())
        self.right_motor.forward(60)
        sleep(1)
        self.right_motor.stop()
        print(self.encoders[0].getValue(), self.encoders[1].getValue())

        sleep(1)
        print(self.encoders[0].getValue(), self.encoders[1].getValue())
        self.right_motor.backwards(90)
        sleep(1)
        self.right_motor.stop()
        print(self.encoders[0].getValue(), self.encoders[1].getValue())

        sleep(2)
        print(self.encoders[0].getValue(), self.encoders[1].getValue())
    
    def _update_position(self, encoderValues):
        wl, wr = self._cal_actual_wheels_speed(encoderValues)
        # print(f'Left wheel speed  = {wl} rad/s.')
        # print(f'Right wheel speed = {wr} rad/s.')

        u, w = self._calc_actual_speeds(wl, wr)
        # print(f"Robot linear speed  = {u} m/s")
        # print(f"Robot angular speed = {w} rad/s")

        self._update_actual_odometry(u, w)
        # print(f"Robot pose is: {self.odometry_position[0]} m, {self.odometry_position[1]} m, {self.odometry_position[2]} rad.")
        data = "W1 " + str(wl) + ";" + \
            "W2 " + str(wr) + ";" + \
            "V1 " + str(u) + ";" + \
            "V2 " + str(w) + ";" + \
            "O1 " + str(self.odometry_position[0]) + ";" + \
            "O2 " + str(self.odometry_position[1]) + ";" + \
            "O3 " + str(self.odometry_position[2]) + ";"
        
        return data

    def _cal_actual_wheels_speed(self, encoderValues):
    #    wl = (encoderValues[0] - self.old_encoders_values[0])/self.delta_t
    #    wr = (encoderValues[1] - self.old_encoders_values[1])/self.delta_t
    #    return wl, wr
    
        ## Calculate the change in angular position of the wheels:
        ang_diff_l = 2*math.pi*(encoderValues[0] - self.old_encoders_values[0])/self.pulses_per_turn
        ang_diff_r = 2*math.pi*(encoderValues[1] - self.old_encoders_values[1])/self.pulses_per_turn
        ## Calculate the angular speeds:
        wl = ang_diff_l/self.delta_t
        wr = ang_diff_r/self.delta_t
        return wl, wr
    
    def _calc_actual_speeds(self, wl, wr):
        u = self.whell_size/2.0 * (wr + wl)
        w = self.whell_size/self.whell_distance * (wr - wl)
        return u, w
    
    def _update_actual_odometry(self, u, w):
        delta_phi = w * self.delta_t
        phi = self.odometry_position[2] + delta_phi
        if phi >= math.pi:
            phi = phi - 2*math.pi
        elif phi < -math.pi:
            phi = phi + 2*math.pi

        delta_x = u * math.cos(phi) * self.delta_t
        delta_y = u * math.sin(phi) * self.delta_t

        self.odometry_position[0] = self.odometry_position[0] + delta_x
        self.odometry_position[1] = self.odometry_position[1] + delta_y
        self.odometry_position[2] = phi
    
    def _calc_wheels_speed_to(self, u, w):
        w = -w
        wr = (u/self.whell_size) + (w*self.whell_distance)/(2*self.whell_size)
        wl = (u/self.whell_size) - (w*self.whell_distance)/(2*self.whell_size)
        return wr, wl

    def _convert_speed_to_pwm(self, speed):
        speed_min = -1
        speed_max = 1

        limited_speed = max(speed_min, min(speed_max, speed))

        pwm_value = int(((limited_speed - speed_min) / (speed_max - speed_min)) * 1023)
        return max(0, min(1023, pwm_value))
