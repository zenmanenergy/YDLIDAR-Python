#!/usr/bin/env python3
"""
Robot entry point. RobotPy runs this file on the roboRIO.

All the setup lives in robotcontainer.py; TimedCommandRobot runs the command
scheduler every 20 ms, which calls each subsystem's periodic() and runs commands.

    robotpy sync        (once, with internet: installs packages from pyproject.toml)
    robotpy deploy      (connected to the robot)
    robotpy sim         (run on your computer in the simulator)
"""

import commands2

from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    def __init__(self):
        super().__init__()
        self.container = RobotContainer()
