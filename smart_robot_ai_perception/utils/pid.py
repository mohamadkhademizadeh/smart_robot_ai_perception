class PID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, integral_limit=None):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.integral = 0.0
        self.prev_error = None
        self.integral_limit = integral_limit

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def step(self, error, dt):
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        d = 0.0
        if self.prev_error is not None and dt > 0.0:
            d = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * d
