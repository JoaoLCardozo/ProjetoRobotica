import math


def angle_diff(target, current):
    """Return the smallest signed difference between two angles."""
    a = (target - current + math.pi) % (2 * math.pi) - math.pi
    return a


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None), windup_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.output_limits = output_limits
        self.windup_limit = windup_limit

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, target, current, dt, angular=True):
        if dt <= 0:
            return 0.0, 0.0
        if angular:
            error = angle_diff(target, current)
        else:
            error = target - current

        # Integral with anti-windup
        self.integral += error * dt
        if self.windup_limit is not None:
            self.integral = max(min(self.integral, self.windup_limit), -self.windup_limit)

        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Clamp output
        lo, hi = self.output_limits
        if lo is not None:
            output = max(lo, output)
        if hi is not None:
            output = min(hi, output)

        self.prev_error = error
        return output, error
