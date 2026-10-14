"""Stand-in for wpimath.geometry with RobotPy's Python names and real math."""
import math


class Rotation2d:
    def __init__(self, radians=0.0):
        self._r = float(radians)

    @staticmethod
    def fromDegrees(deg):
        return Rotation2d(math.radians(deg))

    def radians(self):
        # WPILib keeps the angle wrapped via cos/sin; mimic with atan2
        return math.atan2(math.sin(self._r), math.cos(self._r))

    def degrees(self):
        return math.degrees(self.radians())

    def cos(self):
        return math.cos(self._r)

    def sin(self):
        return math.sin(self._r)

    def __add__(self, other):
        return Rotation2d(self._r + other._r)

    def __sub__(self, other):
        return Rotation2d(self._r - other._r)


class Translation2d:
    def __init__(self, a=0.0, b=0.0):
        if isinstance(b, Rotation2d):
            self._x, self._y = a * b.cos(), a * b.sin()
        else:
            self._x, self._y = float(a), float(b)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    def norm(self):
        return math.hypot(self._x, self._y)

    def angle(self):
        return Rotation2d(math.atan2(self._y, self._x))

    def rotateBy(self, r):
        c, s = r.cos(), r.sin()
        return Translation2d(self._x * c - self._y * s, self._x * s + self._y * c)

    def __add__(self, o):
        return Translation2d(self._x + o._x, self._y + o._y)

    def __sub__(self, o):
        return Translation2d(self._x - o._x, self._y - o._y)


class Pose2d:
    def __init__(self, translation, rotation):
        self._t, self._r = translation, rotation

    def translation(self):
        return self._t

    def rotation(self):
        return self._r


class Transform2d:
    def __init__(self, translation, rotation):
        self._t, self._r = translation, rotation

    def translation(self):
        return self._t

    def rotation(self):
        return self._r
