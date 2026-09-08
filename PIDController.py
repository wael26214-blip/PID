class PIDController:
    def __init__(self,dt, kp, ki, kd, min_output, max_output):
        self.dt = dt
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output

        self.deadzone = 0.0
        self.target = 0.0
        self.integral = 0.0
        self.previous_error = 0.0

    def set_active_target(self, target):
        self.target = target
        self.integral = 0.0
        self.deadzone = abs(target) * 0.001

    def compute(self, current_state):


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

        return output
    
