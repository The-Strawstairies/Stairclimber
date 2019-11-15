# Stairclimber
Operating and firmware code for a wheeled stair climbing robot

## Building

### Operation software (Pi // Laptop)

In the ``src-analysis`` folder, there is a python script for running a
teleoperated serial console with the wsad keyboard keys. Operations include:
* Forward
* Backward
* cw turn
* ccw turn
* quick stopping
* (wip) live speed adjustment

### Driver firmware (arduino)

In the central ``src`` folder, there is a cpp program which is the firmware for
the arduino located directly on the robot. Receives serial commands, operates
the gimbal system automatically, and maintains the motor controller protocols.


Copyright 2019 CC-BY-SA-NC
