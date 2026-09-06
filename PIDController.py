import time

class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output

        self.dt = 0.0
        self.deadzone = 0.0
        self.target = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = time.time()

    def set_active_target(self, target):
        self.target = target
        self.integral = 0.0
        self.deadzone = abs(target) * 0.01

    def compute(self, current_state):
        current_time = time.time()
        self.dt = current_time - self.previous_time
        
        if self.dt <= 0.0:
            self.dt = 1e-16

        error = self.target - current_state

        if abs(error) < self.deadzone:
            error = 0.0

        p_out = self.kp * error

        self.integral += ( error * self.dt )
        i_out = self.ki * self.integral

        derivative = (error - self.previous_error) / self.dt
        d_out = self.kd * derivative

        output = p_out + i_out + d_out

        if output > self.max_output:
            output = self.max_output
        elif output < self.min_output:
            output = self.min_output

        self.previous_error = error
        self.previous_time = current_time

        return output
    
