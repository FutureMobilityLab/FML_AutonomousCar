import TraxxasServoCMD
import time

TraxxasServoCMD.SteeringCommand(.5)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(-.5)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(-5)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(5)
time.sleep(1)
TraxxasServoCMD.SteeringCommand(0)
