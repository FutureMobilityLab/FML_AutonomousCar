import os
import signal

import rclpy
from pynput.keyboard import Key, Listener

from ros2_traxxas_controls.traxxas_teleop import Teleop


class HoldKeyTeleop(Teleop):
    def __init__(self):
        super().__init__()
        self.lr_binding = None
        self.fb_binding = None
        self.key_listener = Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.key_listener.start()
        self.keys_lr_bindings = {
            "a": (self.ANGULAR_MAX),
            "d": (-self.ANGULAR_MAX),
        }
        self.keys_fb_bindings = {
            "w": (self.LINEAR_MAX),
            "s": (-self.LINEAR_MAX),
        }
        self.special_keys_lr_bindings = {
            Key.left: (self.ANGULAR_MAX),
            Key.right: (-self.ANGULAR_MAX),
        }
        self.special_keys_fb_bindings = {
            Key.up: (self.LINEAR_MAX),
            Key.down: (-self.LINEAR_MAX),
        }
        self.get_logger().info(
            f"""
This node takes inputs from the keyboard and publishes them 
as Ackermann Stamped messages. Pressed keys will
set the maximum configured output, at release these values are set to 0

WARNING: This node will take commands even if your terminal is out of focus

Controls:

WASD or Arrows to move
Any other key to stop
CTRL-C or q to quit

Configuration:

Max Linear Speed: +/-{self.LINEAR_MAX} m/s
Max Angular Speed: +/-{self.ANGULAR_MAX} rad/s
"""
        )

    def on_release(self, key):
        if self._is_special_key(key):

            if key in self.special_keys_fb_bindings:
                self.write_twist(linear=0.0)
            elif key in self.special_keys_lr_bindings:
                self.write_twist(angular=0.0)
        else:
            key = key.char
            if key in self.keys_fb_bindings:
                    self.write_twist(linear=0.0)
            elif key in self.keys_lr_bindings:
                self.write_twist(angular=0.0)

    def on_press(self, key):
        if self._is_special_key(key):
            if key in self.special_keys_lr_bindings:
                self.lr_binding = self.special_keys_lr_bindings[key]
            if key in self.special_keys_fb_bindings:
                self.fb_binding = self.special_keys_fb_bindings[key]
            else:
                self.write_twist(0.0, 0.0)
        else:
            if key.char == "q":
                os.kill(os.getpid(), signal.SIGINT)
            if key.char in self.keys_fb_bindings:
                self.fb_binding = self.keys_fb_bindings[key.char]
            if key.char in self.keys_lr_bindings:
                self.lr_binding = self.keys_lr_bindings[key.char]
            else:
                self.write_twist(0.0, 0.0)
    
        new_linear = self.fb_binding
        new_angular = self.lr_binding
        self.write_twist(new_linear, new_angular)

    def _is_special_key(self, key):
        try:
            key.char
            return False
        except AttributeError:
            return True


def main():
    try:
        rclpy.init()
        node = HoldKeyTeleop()
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
