"""Stand-in for wpimath.kinematics.ChassisSpeeds (same constructor keywords)."""


class ChassisSpeeds:
    def __init__(self, vx=0.0, vy=0.0, omega=0.0):
        self.vx, self.vy, self.omega = vx, vy, omega
