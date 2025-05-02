import math

class PID_controller:
    def __init__(self, Kp=2, Ki=0.1, Kd=0.05, Ts=0.05, output_min=-1, output_max=1):
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
        I = self.integral_sum*self.Ki

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


def compute_motor_values(theta_desired, theta_current, distance_desired, distance_current, angle_pid, distance_pid):
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

# Example Usage (for testing)
if __name__ == "__main__":
    # Initialize PID controllers with defaults
    angle_pid = PID_controller(Kp=2, Ki=0.1, Kd=0.05, Ts=0.05, output_min=-1, output_max=1)
    distance_pid = PID_controller(Kp=1.0, Ki=0.05, Kd=0.02, Ts=0.05, output_min=0.0, output_max=0.8)

    # Test inputs
    theta_desired = math.radians(0)
    theta_current = math.radians(45)
    distance_desired = 2.0
    distance_current = 0.0

    # Compute motor values
    left_speed, right_speed, angle_done, distance_done = compute_motor_values(
        theta_desired, theta_current, distance_desired, distance_current, angle_pid, distance_pid
    )

    def scale_to_pwm(speed):
        if speed < 0:
            return 1000.0
        pwm = 1000 + (speed / 1.8) * 1000
        return min(max(pwm, 1000), 2000)

    # Map speed to appropriate PWM values (min:1000, max:2000)
    # left_speed = 1500 + 500*left_speed
    # right_speed = 1500 + 500*right_speed
    left_speed = scale_to_pwm(left_speed)
    right_speed = scale_to_pwm(right_speed)

    print(f"Left Motor Speed: {left_speed:.3f}, Right Motor Speed: {right_speed:.3f}")
    print(f"Angle Done: {angle_done}, Distance Done: {distance_done}")
