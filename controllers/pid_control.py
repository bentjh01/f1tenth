class PID():
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.set_point = 0

    def set_set_point(self, set_point):
        self.set_point = set_point

    def set_Kp(self, Kp):
        self.Kp = Kp
    
    def set_Ki(self, Ki):
        self.Ki = Ki

    def set_Kd(self, Kd):
        self.Kd = Kd

    def get_integral(self):
        return self.integral
    
    def reset_integral(self):
        self.integral = 0

    def update(self, feedback):
        error = feedback - self.set_point

        p_term = self.Kp * error

        self.integral += error
        i_term = self.Ki * self.integral

        d_term = self.Kd * (error - self.prev_error)
        self.prev_error = error

        return p_term + i_term + d_term