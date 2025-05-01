import math

class PID_controller:
    def __init__(self, Kp, Ki, Kd, Ts, output_min, output_max):
        """ Initialize PID controller with gains and time constant"""
        if(Ts <= 0):
            raise ValueError("Sampling time must be positive")

        self.Kp = Kp 
        self.Ki = Ki
        self.Kd = Kd
        self.Ts = Ts
        self.output_min = output_min
        self.output_max = output_max

        self.integral_sum = 0
        self.previous_error = 0

    def compute(self, error):
        """Compute PID output for given error."""

        #Proportional 
        P = error*self.Kp

        #Integral 
        self.integral_sum += error*self.Ts
        I = self.integral_sum*Ki

        #Derivative
        slope = (error - self.previous_error)/self.Ts
        D = (slope*self.Kd)

        
        output = P + I + D
        #clamping to prevent integral windup
        output = max(self.output_min, min(self.output_max, output))
        self.previous_error = error

        return output
    
    
    def reset(self):
        """Reset the PID controller state."""
        self.integral_sum = 0
        self.previous_error = 0


def compute_motor_values(self, theta_desired, theta_current, distance_desired, distance_current, angle_pid, distance_pid):
    """Compute motor speeds and goal status for differential drive robot."""
        error_angle = math.atan2(math.sin(theta_desired - theta_current), math.cos(theta_desired- theta_current)) #angle warp
        error_distance = distance_desired- distance_current

        delta_theta = angle_pid.compute(error_angle)
        base_speed = distance_pid.compute(error_distance)

        left_motor_speed = base_speed + delta_theta
        right_motor_speed =  base_speed - delta_theta

        angle_done = abs(error_angle) < 0.0872 #5 degrees
        distance_done = abs(error_distance) < 0.2 #20 cm

        return left_motor_speed, right_motor_speed, angle_done, distance_done
