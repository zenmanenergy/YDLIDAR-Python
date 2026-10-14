"""
RobotPy's standard robot tests. "robotpy test" runs these (and "robotpy deploy"
runs them before deploying): they start the robot in the simulator and run it
through disabled, autonomous, and teleop to catch crashes.

The lidar logic tests live outside this folder, in ../roborio_tests, because they
use stand-in modules that would conflict with the real RobotPy here.
"""

from pyfrc.tests import *  # noqa: F401,F403
