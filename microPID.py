class PID:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(None, None)):
        """
        Initialize a PID controller.

        Parameters:
        kp (float): Proportional gain.
        ki (float): Integral gain.
        kd (float): Derivative gain.
        setpoint (float): Desired target value.
        output_limits (tuple): Min and max output values (default: no limits).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits

        self._integral = 0
        self._previous_error = 0
        self._last_output = 0

    def compute(self, feedback, dt):
        """
        Compute the PID output based on the feedback and elapsed time.

        Parameters:
        feedback (float): Current feedback value.
        dt (float): Time elapsed since the last compute (in seconds).

        Returns:
        float: PID control output.
        """
        error = self.setpoint - feedback
        self._integral += error * dt
        derivative = (error - self._previous_error) / dt

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)

        # Apply output limits
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)

        self._previous_error = error
        self._last_output = output
        return output

    def reset(self):
        """Reset the PID controller."""
        self._integral = 0
        self._previous_error = 0
        self._last_output = 0
