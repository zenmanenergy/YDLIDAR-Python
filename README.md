# YDLIDAR GS2 for FRC

Short-range lidar for an FRC robot: one or more YDLIDAR GS2 sensors plugged into the PhotonVision coprocessor (an Orange Pi), merged into a single point cloud in robot coordinates, and published to NetworkTables for the roboRIO.

The GS2 is a solid-state line lidar: 100° field of view, out to about 300 mm, roughly 25 scans per second with 160 points each. It is great for final alignment, wall distance, and the last few centimeters of intake approach. It is not a mapping lidar.

```
  GS2 ──USB──┐                                   ┌──> /lidar/frontDistance
  GS2 ──USB──┤  Orange Pi                        │    /lidar/wall/...
  camera ────┤   ├─ PhotonVision ──┐             │    /lidar/merged/points
  camera ────┘   └─ lidar.py ──────┴─ Ethernet ──┴──> roboRIO (robot code)
```

PhotonVision and `lidar.py` are separate programs. They never talk to each other; both publish to NetworkTables, and the robot code combines them.

## Repository layout

```
orangepi/
  lidar.py          the service: reads all lidars, merges, publishes /lidar
  gs2.py            GS2 serial protocol and calibration math
  geometry.py       config, robot-frame transforms, front distance, wall fit
  calibrate.py      measures mounting yaw errors against a flat wall
  config.json       team number, robot size, lidar ports and mounting positions
  lidar.service     systemd unit (starts on boot, restarts on crash)
  deploy.sh         installs everything on the Orange Pi
  tools/fake_gs2.py simulated GS2 for testing without hardware
  tests/            automated tests (run against simulated lidars)
roborio/examples/                     RobotPy (Python) robot examples, one robot.py each
  read_lidar/                         start here: shows lidar values, doesn't move
  drive_to_wall_distance/             hold A: drive to a set gap from the wall
  square_to_wall/                     hold A: turn until square to the wall
  lidar_intake_assist/                hold A: camera far away, lidar up close
roborio_tests/                        logic tests for the examples (run without RobotPy)
```

## Hardware

Each GS2 comes with a small EAI adapter board (marked `Radar_Con_V1.4`, with a CP2102 chip). Plug the lidar's 4-wire cable into the adapter, and the adapter's USB-C port (labeled `USB_DATA`) into the Orange Pi. Linux recognizes the CP2102 with no driver install. Each lidar appears as a serial port at 921600 baud.

The second USB port on the adapter (`USB_PWR`) is only for extra power. You shouldn't need it on the Orange Pi, but if a lidar drops out when the robot's battery sags, try it.

Things to get right on the robot:

- **Label the USB ports.** The config refers to lidars by physical USB port, so the left lidar must always go in the same port. Put tape labels on the Orange Pi.
- **Mounting height.** The GS2 sees a thin horizontal slice at the height it is mounted. To detect game pieces, mount it low enough to hit them. To detect walls, mount it where the walls are solid (not at a gap or railing).
- **Mount it flat and rigid.** A lidar tilted down sees the carpet; a loose mount shifts the whole point cloud.
- **Front corners.** Each GS2 covers 100°. Angled about 15° outward on the front corners, the two fields of view overlap ahead of the robot. Angling them much further leaves a blind wedge straight ahead.

## Install on the Orange Pi

Needs internet the first time (for apt and pip). Connect the Pi to a network with internet access, then:

```bash
git clone https://github.com/zenmanenergy/YDLIDAR-Python.git
cd YDLIDAR-Python/orangepi
```

**1. Find the port names.** Plug in the lidars and run:

```bash
ls -l /dev/serial/by-path/
```

You'll see one entry per lidar, named after the physical USB port. Unplug one to see which is which.

**2. Edit `config.json`:** set `team`, the robot size, and each lidar's `port` and mounting position (see [Config](#config)).

**3. Install:**

```bash
sudo bash deploy.sh
```

This copies the code to `/opt/lidar`, installs Python packages into `/opt/lidar/venv`, installs and starts the `lidar` service, and checks the config (it warns if the team number is unset or a port is missing).

**4. Check it's running:**

```bash
journalctl -u lidar -f
```

You should see `scanning, 1 unit(s) on this port` for each lidar.

Running `sudo bash deploy.sh` again updates the code but keeps the installed `config.json`. Use `sudo bash deploy.sh --replace-config` to overwrite it with the repo's copy.

### After reflashing PhotonVision

Updating PhotonVision usually means flashing a new SD card image, which wipes the lidar install. Keep `config.json` in this repo up to date, and reinstalling is just steps 3 and 4.

## Config

All positions use the WPILib robot coordinate system, the same one PhotonVision and your robot code use:

- origin at the **center of the robot**
- **+x forward**, **+y left**
- angles **counterclockwise positive** (a lidar turned toward the left has a positive yaw)
- distances in **meters**

| Field | Meaning |
|---|---|
| `team` | Team number, used to find the roboRIO |
| `server` | Leave `""`. Set an IP (like `"127.0.0.1"`) only to test against a different NetworkTables server |
| `robot.length_m`, `robot.width_m` | Outer size **including bumpers**. Used to measure distances from the bumper instead of robot center |
| `publish_hz` | How often to publish (50 matches the robot loop) |
| `stale_s` | A lidar frame older than this is ignored |
| `lidars[].name` | Used in NetworkTables topic names. Must be unique |
| `lidars[].port` | The `/dev/serial/by-path/...` path |
| `lidars[].module` | `0` unless lidars are daisy-chained on one cable (then 0, 1, 2 in chain order) |
| `lidars[].x_m`, `y_m` | Where the lidar's sensor window is, relative to robot center |
| `lidars[].yaw_deg` | Which way the lidar faces. `0` = straight ahead |
| `lidars[].flip` | Set `true` if objects on the left show up on the right (see [Troubleshooting](#troubleshooting)) |
| `lidars[].enabled` | Set `false` to ignore a lidar without deleting its entry |

After editing `/opt/lidar/config.json` directly on the Pi, run `sudo systemctl restart lidar`.

## Calibrate the mounting angles

A 2° error in `yaw_deg` shifts a point 300 mm away by about 10 mm, and makes the two lidars disagree about where a wall is. To measure it:

1. `sudo systemctl stop lidar` (the calibration tool needs the serial ports)
2. Face the robot squarely toward a flat wall, about 100-200 mm away. Use a carpenter's square, or make two tape measurements at the bumper corners equal.
3. Measure the gap from the front bumper to the wall.
4. Run:
   ```bash
   cd /opt/lidar
   sudo venv/bin/python calibrate.py --distance-mm 150
   ```
   It prints each lidar's measured error and corrected `yaw_deg` (and `x_m`). Add `--write` to save them.
5. `sudo systemctl start lidar`
6. Copy the new values into `config.json` in the repo and commit, so the next install keeps them.

## Testing without the robot

**Console mode.** Plug a lidar into any Linux laptop (or the Pi with the service stopped) and run:

```bash
python3 lidar.py --config config.json --print
```

It prints the front distance, wall fit, and each lidar's status twice a second, without NetworkTables. Needs `pip install pyserial`.

**Simulated lidar.** No hardware at all:

```bash
python3 tools/fake_gs2.py --wall-mm 180 --wall-angle-deg -15
```

It prints a device path like `/dev/pts/3`. Put that in a test config's `port` and run `lidar.py --print` or `calibrate.py` against it.

**Automated tests** (these use the simulator and need pyserial):

```bash
cd orangepi
python3 -m unittest discover tests -v
```

They cover packet parsing and checksums, the calibration math, robot-frame transforms, the wall fit, two simulated lidars end to end (square and rotated to a wall, corrupted data), console mode, and `calibrate.py` finding a deliberately wrong mounting angle.

## NetworkTables topics

Everything is under `/lidar`. Distances in meters, angles in degrees.

| Topic | Type | Meaning |
|---|---|---|
| `heartbeat` | int | Increases every update. If it stops changing, the script isn't running |
| `connected` | boolean | At least one lidar is producing fresh data |
| `latencyMs` | double | Age of the oldest frame in the current output |
| `frontDistance` | double | Front bumper to the closest point directly ahead (within the robot's width). `-1` if nothing |
| `wall/valid` | boolean | The points ahead form a clean straight line |
| `wall/distance` | double | Perpendicular distance from the front bumper to that line |
| `wall/angleDeg` | double | How far the robot is turned from square to the wall, CCW positive. Turn by the negative to square up |
| `wall/rms` | double | Fit error. Small means a flat surface |
| `merged/points` | double[] | All points, robot frame, flattened `[x0, y0, x1, y1, ...]` |
| `merged/translations` | Translation2d[] | Same points as structs, for AdvantageScope (if robotpy-wpimath installed) |
| `robot/length`, `robot/width` | double | Robot size from config |
| `<name>/connected` | boolean | This lidar is producing fresh data |
| `<name>/fps` | double | Scans per second (about 25 is normal) |
| `<name>/points` | double[] | This lidar's points only, robot frame |
| `<name>/pointCount` | int | Valid points in its latest scan |
| `<name>/status` | string | `ok`, or the last error |
| `<name>/checksumErrors` | int | Corrupted packets so far. Climbing steadily means a bad cable or USB problem |

## Robot code examples

Each folder in `roborio/examples` is a complete robot program for RobotPy 2026, with all its code in one `robot.py` that reads top to bottom. They use a plain `TimedRobot` and an Xbox controller, nothing else. Start with `read_lidar`.

| Example | What it does |
|---|---|
| `read_lidar` | Shows the lidar values on the SmartDashboard. Doesn't move. Use it to check everything is connected |
| `drive_to_wall_distance` | Hold A: drives forward or backward until the front bumper is 50 mm from the wall |
| `square_to_wall` | Hold A: turns in place until the robot faces the wall squarely |
| `lidar_intake_assist` | Hold A: steers to a game piece with the PhotonVision camera, then finishes the approach with the lidar |

In every example, letting go of A stops the robot.

**They don't move motors yet.** Each driving example has a `drive(forward, left, turn)` method at the bottom that only shows the speeds it wants on the SmartDashboard (`Drive/Forward`, `Drive/Left`, `Drive/Turn`). That makes them safe to try on a robot on blocks. To drive for real, replace the inside of `drive()` with a call to your drivetrain. The comment in each file shows how.

Each example folder contains:

- `robot.py`: all the code
- `pyproject.toml`: which RobotPy version and packages to install
- `tests/pyfrc_test.py`: one line that runs RobotPy's standard startup test. `robotpy deploy` runs it before deploying and won't deploy without it. You don't need to read it.

**Run an example** (with RobotPy 2026 installed on your laptop):

```bash
cd roborio/examples/read_lidar
robotpy sync      # once, with internet: downloads RobotPy and packages
robotpy sim       # try it in the simulator on your laptop
robotpy deploy    # or send it to the robot
```

For `lidar_intake_assist`, set the camera's name and mounting position at the top of `robot.py`, and keep `photonlibpy` in its `pyproject.toml` at the same version as PhotonVision on the Orange Pi (shown in its web dashboard).

Every example checks that the lidar data is fresh before using it: the Orange Pi increases `heartbeat` every update, and if it stops changing, the examples treat the lidar as missing and stop instead of acting on an old number.

The tuning numbers (speeds and gains) at the top of each `robot.py` are starting points. Tune them on the robot, slowly.

### Tests

```bash
python3 -m unittest discover roborio_tests -v
```

These run on any computer, without RobotPy. `roborio_tests/fakes` holds small stand-ins for wpilib, ntcore and photonlibpy that use the same names as RobotPy 2026. Each test loads an example's `robot.py`, connects its `drive()` to a simulated robot, and runs it like the real 20 ms robot loop: driving to a wall gap (forward and backward), squaring up from either side, stopping when A is released or the lidar goes quiet, and the full intake approach where the camera loses the piece near the bumper and the lidar finishes the job. They live outside `roborio` so their stand-ins never conflict with the real RobotPy.

They check the logic, not your drivetrain. Still test on the real robot, on blocks or at low speed first.

## Combining with PhotonVision

The two sensors are good at opposite ranges. PhotonVision works from across the field but its distance estimates get noisy up close. The lidar is blind past about 300 mm but precise to a few millimeters inside that.

- **Game piece intake** (`lidar_intake_assist`): the camera gives the direction to the piece. The robot looks for lidar points along that same line from the camera's position. Once the lidar sees the piece, its distance drives the final approach, and it keeps going on the lidar alone if the camera loses the piece right at the bumper.
- **Scoring alignment:** drive to the scoring location with your AprilTag pose estimate, then finish with the logic from `square_to_wall` and `drive_to_wall_distance`. The pose estimate gets you close; the lidar gets you exact.

## Viewing in AdvantageScope

Connect AdvantageScope to the robot and look under `/lidar` for every value in the table above. `/lidar/merged/translations` holds the points as robot-relative positions, which AdvantageScope can draw in a 2D field view (attached to the robot's pose).

## Troubleshooting

**`journalctl -u lidar -f` says "no calibration reply"**: the lidar isn't answering. Check the cable is fully seated in the adapter, try the other USB port, and confirm with the YDLIDAR Point Cloud Viewer on a PC.

**A lidar shows `connected` false**: check `/lidar/<name>/status` for the reason. If the port path is wrong, run `ls -l /dev/serial/by-path/` and fix the config.

**`checksumErrors` keeps climbing**: a flaky cable, loose connector, or a USB port sharing bandwidth with a camera. Try a different port.

**Left and right are mirrored** (an object on the left of the robot shows up with negative y): set `"flip": true` for that lidar. This depends on how the sensor is mounted (upside down flips it).

**Points don't line up between the two lidars**: run the calibration. If they're offset (not rotated), check the `x_m` and `y_m` measurements.

**Lidar drops out when the robot drives hard**: the battery voltage sag is browning out USB power. Plug a second cable into the adapter's `USB_PWR` port from a powered source.

**Robot code says not connected but the Pi log looks fine**: check that `team` in the config is right and that the Pi can reach the roboRIO (`ping 10.TE.AM.2`).

## Protocol notes

The GS2 protocol and the angle-correction math in `gs2.py` follow YDLIDAR's official SDK ([YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK), `src/GSLidarDriver.cpp`). Each GS2 has two sensor halves with their own factory calibration, which the driver reads from the device at startup and applies to every point. Up to three GS2 units can be daisy-chained on one serial line; `gs2.py` supports this through the `module` field, though the USB-per-lidar setup is simpler.
