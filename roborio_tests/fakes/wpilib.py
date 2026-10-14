"""Stand-in for wpilib: TimedRobot, XboxController, a controllable clock, and a dashboard."""


class Timer:
    now = 0.0

    @staticmethod
    def getFPGATimestamp():
        return Timer.now


class SmartDashboard:
    values = {}

    @staticmethod
    def putBoolean(key, value):
        SmartDashboard.values[key] = value
        return True

    @staticmethod
    def putNumber(key, value):
        SmartDashboard.values[key] = value
        return True


class TimedRobot:
    def __init__(self):
        pass


class XboxController:
    """Tests set .a to hold the A button. getAButtonPressed() is true once per press."""

    def __init__(self, port):
        self.port = port
        self.a = False
        self._a_reported = False

    def getAButton(self):
        return self.a

    def getAButtonPressed(self):
        if self.a and not self._a_reported:
            self._a_reported = True
            return True
        if not self.a:
            self._a_reported = False
        return False
