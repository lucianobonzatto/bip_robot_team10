class PID:
    def __init__(self, kp, ki, kd, delta_t, max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.delta_t = delta_t
        self.max = max
        self.integral = 0
        self.previous_error = 0

    def update(self, e):
        P_term = self.kp * e                                        # Proportional term; kp is the proportional gain
        I_term = self.previous_error + self.ki * e * self.delta_t   # Intergral term; ki is the integral gain
        D_term = self.kd * (e - self.integral)/self.delta_t         # Derivative term; kd is the derivative gain
        output = P_term + I_term + D_term
        
        self.previous_error = e                  # error value in the previous interation (to calculate the derivative term)
        self.integral = I_term                   # accumulated error value (to calculate the integral term)
        
        if(output > self.max):
            output = self.max
        if(output < -self.max):
            output = -self.max
        
        return output
