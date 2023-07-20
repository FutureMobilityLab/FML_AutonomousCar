"""Python implementation of open loop lateral controllers. Intended to be used by ROS2 
Node controller in controller.py"""

from typing import Dict, Tuple
import numpy as np


class OpenLoopChirp:
    """Open-loop controller that provides a linear chirp signal to steering."""

    def __init__(self, ctrl_params: Dict[str, float]):
        self.startFreq = ctrl_params.get("start_frequency_hz")
        endFreq = ctrl_params.get("end_frequency_hz")
        self.startAmp = ctrl_params.get("start_amplitude_rad")
        endAmp = ctrl_params.get("end_amplitude_rad")
        duration = ctrl_params.get("duration_s")
        self.ctrl_clock = ctrl_params.get("controller_clock")
        self.velocity_setpoint = ctrl_params.get("speed_setpoint")
        self.max_accel = ctrl_params.get("max_accel")

        # Compute start and end times for acceleration, chirp, and deceleration.
        now = self.ctrl_clock.now().nanoseconds * 10**-9
        accel_duration = self.velocity_setpoint / self.max_accel
        settling_time = 0.5
        self.accel_start_time = now
        self.chirp_start_time = now + accel_duration + settling_time
        self.chirp_end_time = self.chirp_start_time + duration

        # Compute chirp function parameters.
        self.chirpRate = (endFreq - self.startFreq) / duration
        self.ampRate = (endAmp - self.startAmp) / duration

    def get_commands(
        self, x: float, y: float, yaw: float, v: float
    ) -> Tuple[float, float, float, float]:
        now = self.ctrl_clock.now().nanoseconds * 10**-9

        # Steer straight while accelerating to velocity setpoint.
        if now >= self.accel_start_time and now < self.chirp_start_time:
            t = now - self.accel_start_time
            steering_angle = 0
            speed_cmd = np.clip(self.max_accel * t, 0, self.velocity_setpoint)
        # Perform chirp steer maneuver.
        elif now >= self.chirp_start_time and now < self.chirp_end_time:
            t = now - self.start_time
            amp = self.startAmp + self.ampRate * t
            phase = self.chirpRate / 2 * t**2 + self.startFreq * t
            steering_angle = amp * np.sin(2 * np.pi * phase)
            speed_cmd = self.velocity_setpoint
        # Steer straight while decelerating to 0 mps.
        elif now >= self.chirp_end_time:
            t = now - self.chirp_end_time
            steering_angle = 0
            speed_cmd = self.velocity_setpoint - t * self.max_accel
            speed_cmd = np.clip(speed_cmd, 0, self.velocity_setpoint)

        return (steering_angle, speed_cmd, 0.0, 0.0)
