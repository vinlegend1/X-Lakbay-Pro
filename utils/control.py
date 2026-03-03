import time

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.last_val = None

    def apply(self, value):
        if self.last_val is None:
            self.last_val = value
        self.last_val = self.alpha * value + (1 - self.alpha) * self.last_val
        return self.last_val

class PID:
    def __init__(self, kp, ki, kd, output_limit=1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()
        self.output_limit = output_limit

    def update(self, error):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            dt = 0.001

        self.integral += error * dt
        # Simple anti-windup: clamp integral
        self.integral = max(min(self.integral, self.output_limit), -self.output_limit)
        
        derivative = (error - self.last_error) / dt
        
        self.last_error = error
        self.last_time = now
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, self.output_limit), -self.output_limit)

    def reset_integral(self):
        self.integral = 0
        self.last_error = 0
        self.last_time = time.time()
