import TraxxasServoCMD
import TraxxasDriverCMD
import time

TraxxasServoCMD.SteeringCommand(5)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(0.5)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(0)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(-0.5)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(-5)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(0)
time.sleep(1)

TraxxasDriverCMD.ThrottleCommand(0x7FFF)
time.sleep(6)
TraxxasDriverCMD.ThrottleCommand(0x0000)
time.sleep(6)