"""Stand-in for wpilib: a controllable clock and a no-op dashboard."""


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
