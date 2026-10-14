#!/usr/bin/env python3
"""
EXAMPLE: read_lidar

Start here. This robot doesn't move. It reads the lidar data that the Orange Pi
publishes to NetworkTables and shows it on the SmartDashboard, so you can check
that everything is working.

Put something in front of the robot and watch "Lidar/FrontDistance" change.

How the data gets here:
    GS2 lidars --USB--> Orange Pi (lidar.py) --NetworkTables--> this robot code

Units: meters and degrees. Directions follow the WPILib convention:
+x is forward, +y is left, and counterclockwise angles are positive.
"""

import wpilib
import ntcore

# If the Orange Pi hasn't sent anything new for this long, treat the data as missing
STALE_SECONDS = 0.25


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Everything the Orange Pi publishes is in the "lidar" table
        table = ntcore.NetworkTableInstance.getDefault().getTable("lidar")

        # A "subscriber" lets us read one value. The number in subscribe() is the
        # default we get back if the Orange Pi hasn't published anything yet.
        self.heartbeat = table.getIntegerTopic("heartbeat").subscribe(0)
        self.connected = table.getBooleanTopic("connected").subscribe(False)
        self.front_distance = table.getDoubleTopic("frontDistance").subscribe(-1.0)
        self.wall_valid = table.getBooleanTopic("wall/valid").subscribe(False)
        self.wall_distance = table.getDoubleTopic("wall/distance").subscribe(0.0)
        self.wall_angle = table.getDoubleTopic("wall/angleDeg").subscribe(0.0)
        # All the points the lidars see, as one flat list: [x0, y0, x1, y1, ...]
        self.points = table.getDoubleArrayTopic("merged/points").subscribe([])

        self.last_heartbeat = None
        self.last_heartbeat_time = -1000.0

    def lidarIsFresh(self):
        """
        True if the Orange Pi is still sending data.

        The Pi adds 1 to "heartbeat" every update (50 times a second). If the
        number stops changing, the script on the Pi has stopped or crashed.
        """
        now = wpilib.Timer.getFPGATimestamp()
        beat = self.heartbeat.get()
        if beat != self.last_heartbeat:
            self.last_heartbeat = beat
            self.last_heartbeat_time = now
        recently_changed = (now - self.last_heartbeat_time) < STALE_SECONDS
        return recently_changed and self.connected.get()

    def robotPeriodic(self):
        # robotPeriodic runs every 20 ms in every mode, even when disabled
        fresh = self.lidarIsFresh()
        wpilib.SmartDashboard.putBoolean("Lidar/Connected", fresh)

        if not fresh:
            # Show -1 so it's obvious the numbers are not real
            wpilib.SmartDashboard.putNumber("Lidar/FrontDistance", -1)
            wpilib.SmartDashboard.putBoolean("Lidar/WallFound", False)
            wpilib.SmartDashboard.putNumber("Lidar/PointCount", 0)
            return

        # Distance from the front bumper to the closest thing straight ahead.
        # The Pi sends -1 when nothing is in range (about 300 mm).
        wpilib.SmartDashboard.putNumber("Lidar/FrontDistance", self.front_distance.get())

        # A "wall" is any flat surface ahead. Its angle is how far the robot is
        # turned away from facing it squarely (0 = perfectly square).
        wall_found = self.wall_valid.get()
        wpilib.SmartDashboard.putBoolean("Lidar/WallFound", wall_found)
        if wall_found:
            wpilib.SmartDashboard.putNumber("Lidar/WallDistance", self.wall_distance.get())
            wpilib.SmartDashboard.putNumber("Lidar/WallAngleDeg", self.wall_angle.get())

        # Two numbers per point (x and y), so divide by 2 to count points
        flat = self.points.get()
        wpilib.SmartDashboard.putNumber("Lidar/PointCount", len(flat) // 2)
