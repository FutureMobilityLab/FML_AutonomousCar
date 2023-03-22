# Useful Scripts

Scripts in this folder are useful for configuration and tuning of the FML Autonomous Car

## BicycleModelCalculator.py

This script is useful to generate parameters for use with Youla, MPC or other model-based controllers. Simply run the script to generate a text output of useful parameters. Functions can be modified by changing the constants within the script.

## I2C_ThrottleCalibration.py

This script is used whenever the throttle calibration of the Castle ESC is lost or reset. Simply run the script as follows
```
sudo python3 I2C_ThrottleCalibration {CALIBRATION STAGE}
```
where CALIBRATION STAGE should be set as 1, 2, or 3. Once the calibration setpoint is accepted by the ESC, the script can be re-run with the next stage.

1. Start script at stage 1 before powering on the Castle ESC.
2. Power on ESC and wait until 4 beep sequence sounds, confirming full throttle setpoint.
3. Run script at stage 2 and wait for additional 4 beeps, confirming full reverse setpoint.
4. Run script at stage 3 and wait for final 4 beeps, confirming idle setpoint.  

Additionally, please confirm motor_driver node has correctly set register values to match this script.


## TraxxasManualCMD.py

This script is useful for low level confirmation of I2C Communication to the motor driver and servo. Additionally, this script can be run before testing to guarantee that throttle registers are set to idle, such that the motor controller can immediately begin sending commands at startup. Throttle registers reset each time power is lost.