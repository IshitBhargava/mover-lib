# carParser

Movement system for <a href="https://www.github.com/ishitbhargava/Autonomous-Car-main">the robot</a>.

## Install

```bash
pip install -e .
```

> [!WARNING]
> run this command in the repository folder.

## Usage

```python
import carParser
import mover
import time

carParser.init('/dev/ttyAMA0', 921600)
time.sleep(2)

mover.calibrate()       # Calibrates IMU
mover.setBasespeed(100) # Sets base speed for motion
mover.setThreshold(20)  # Sets correct arrival threshold in mm
mover.Debug(True)       # Creates a plot for debugging

mover.move(500, 500)              # Blocks while it moves
print("arrived at:", mover.pos())

mover.move(200, 800)      # can call again with a new target immediately
print("arrived at:", mover.pos())
```
