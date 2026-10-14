"""
YDLIDAR GS2 serial protocol driver.

Protocol details and calibration math follow YDLIDAR's official SDK
(github.com/YDLIDAR/YDLidar-SDK, src/GSLidarDriver.cpp).

Every packet, in both directions, has the same layout:

    A5 A5 A5 A5 | addr | type | size (2 bytes, little endian) | data (size bytes) | checksum

checksum = (addr + type + size_lo + size_hi + sum(data)) & 0xFF

A scan packet (type 0x63) carries 2 bytes of environment data followed by
160 points of 2 bytes each. Each point is a little-endian uint16:
    low 9 bits  = raw distance in mm (0 means no return)
    high 7 bits = intensity
The angle is not sent. It comes from the point's index and per-device
calibration values (k0, b0, k1, b1, bias) read with the 0x61 command.
Points 0-79 come from one half of the sensor and 80-159 from the other.

Up to 3 GS2 units can be chained on one serial line. Each answers with its
own address byte (0x01, 0x02, 0x04), which this driver maps to module
index 0, 1, 2.
"""

import math
import struct
import time
from dataclasses import dataclass, field

import serial

BAUD_RATE = 921600

SYNC = b"\xA5\xA5\xA5\xA5"
HEADER_SIZE = 8

CMD_GET_ADDRESS = 0x60
CMD_GET_PARAMETER = 0x61
CMD_GET_VERSION = 0x62
CMD_SCAN = 0x63
CMD_STOP = 0x64
CMD_RESET = 0x67

POINTS_PER_FRAME = 160
SCAN_DATA_SIZE = 2 + POINTS_PER_FRAME * 2  # environment + points = 322

# Fixed geometry constants from the SDK (mm and degrees)
ANGLE_PX = 1.22
ANGLE_PY = 5.315
PITCH_ANGLE_DEG = 22.5

MAX_PACKET_SIZE = 1024


def checksum(addr, ptype, payload):
    size = len(payload)
    return (addr + ptype + (size & 0xFF) + ((size >> 8) & 0xFF) + sum(payload)) & 0xFF


def build_packet(ptype, payload=b"", addr=0x00):
    header = SYNC + struct.pack("<BBH", addr, ptype, len(payload))
    return header + bytes(payload) + bytes([checksum(addr, ptype, payload)])


def address_to_module(addr):
    """Device address 0x01, 0x02, 0x04 -> module index 0, 1, 2."""
    return addr >> 1


@dataclass
class Calibration:
    k0: float
    b0: float
    k1: float
    b1: float
    bias: float  # degrees

    @classmethod
    def from_payload(cls, payload):
        k0, b0, k1, b1, bias = struct.unpack("<HHHHb", payload[:9])
        return cls(k0 / 10000.0, b0 / 10000.0, k1 / 10000.0, b1 / 10000.0, bias * 0.1)


@dataclass
class Point:
    angle: float      # radians, in the lidar's own frame (+ = counterclockwise)
    distance: float   # mm from the lidar
    intensity: int


@dataclass
class ScanFrame:
    module: int                 # 0, 1 or 2 (position in a cascade chain)
    timestamp: float            # time.monotonic() when the frame finished arriving
    points: list = field(default_factory=list)  # valid points only
    raw_count: int = 0          # points in the frame, including no-returns


def correct_point(raw_dist, index, cal, count=POINTS_PER_FRAME):
    """
    Convert one raw reading into (angle_deg, distance_mm) using the SDK's
    angTransform math. Returns None if the point should be discarded.
    """
    if raw_dist <= 0:
        return None

    half = count // 2
    pitch = PITCH_ANGLE_DEG + cal.bias
    rad = math.pi / 180.0

    if index < half:
        pixel_u = half - index
        if cal.b0 > 1:
            temp_theta = cal.k0 * pixel_u - cal.b0
        else:
            temp_theta = math.atan(cal.k0 * pixel_u - cal.b0) / rad
        temp_dist = (raw_dist - ANGLE_PX) / math.cos((pitch - temp_theta) * rad)
        t = temp_theta * rad
        x = math.cos(pitch * rad) * temp_dist * math.cos(t) + math.sin(pitch * rad) * temp_dist * math.sin(t)
        y = -math.sin(pitch * rad) * temp_dist * math.cos(t) + math.cos(pitch * rad) * temp_dist * math.sin(t)
        x += ANGLE_PX
        y -= ANGLE_PY
    else:
        pixel_u = count - index
        if cal.b1 > 1:
            temp_theta = cal.k1 * pixel_u - cal.b1
        else:
            temp_theta = math.atan(cal.k1 * pixel_u - cal.b1) / rad
        temp_dist = (raw_dist - ANGLE_PX) / math.cos((pitch + temp_theta) * rad)
        t = temp_theta * rad
        x = math.cos(-pitch * rad) * temp_dist * math.cos(t) + math.sin(-pitch * rad) * temp_dist * math.sin(t)
        y = -math.sin(-pitch * rad) * temp_dist * math.cos(t) + math.cos(-pitch * rad) * temp_dist * math.sin(t)
        x += ANGLE_PX
        y += ANGLE_PY

    if abs(x) < 1e-9:
        return None

    theta = math.atan(y / x) / rad      # -90 .. +90 degrees
    distance = math.sqrt(x * x + y * y)

    # The SDK discards points that land on the wrong side of center:
    # the first half must have negative angles, the second half positive.
    if index < half and theta >= 0:
        return None
    if index >= half and theta < 0:
        return None

    return theta, distance


def parse_scan_payload(payload, cal):
    """Turn a 0x63 payload into (valid Points, total point count)."""
    count = (len(payload) - 2) // 2
    raw = struct.unpack_from("<%dH" % count, payload, 2)
    points = []
    for i, word in enumerate(raw):
        dist = word & 0x01FF
        corrected = correct_point(dist, i, cal, count)
        if corrected is None:
            continue
        theta_deg, distance = corrected
        points.append(Point(math.radians(theta_deg), distance, word >> 9))
    return points, count


class PacketReader:
    """Pulls checksum-validated packets out of a byte stream."""

    def __init__(self):
        self.buf = bytearray()
        self.checksum_errors = 0

    def feed(self, data):
        self.buf.extend(data)

    def packets(self):
        """Yield (addr, type, payload) for each complete, valid packet."""
        while True:
            start = self.buf.find(SYNC)
            if start < 0:
                # Keep the last 3 bytes in case a sync sequence is split
                del self.buf[:-3]
                return
            if start > 0:
                del self.buf[:start]
            # Skip extra A5 bytes (the SDK does the same)
            while len(self.buf) > 4 and self.buf[4] == 0xA5:
                del self.buf[0]
            if len(self.buf) < HEADER_SIZE:
                return
            addr, ptype, size = struct.unpack_from("<BBH", self.buf, 4)
            if size > MAX_PACKET_SIZE:
                del self.buf[0]
                continue
            total = HEADER_SIZE + size + 1
            if len(self.buf) < total:
                return
            payload = bytes(self.buf[HEADER_SIZE:HEADER_SIZE + size])
            cs = self.buf[HEADER_SIZE + size]
            if checksum(addr, ptype, payload) != cs:
                self.checksum_errors += 1
                del self.buf[0]  # resync from the next byte
                continue
            del self.buf[:total]
            yield addr, ptype, payload


class GS2Error(Exception):
    pass


class GS2Port:
    """
    One serial port with one GS2, or up to three cascaded GS2 units.

    Typical use:
        port = GS2Port("/dev/ttyUSB0")
        port.open()
        port.start()
        for frame in port.frames():
            ...
    """

    def __init__(self, device, baud=BAUD_RATE):
        self.device = device
        self.baud = baud
        self.ser = None
        self.reader = PacketReader()
        self.calibrations = {}  # module index -> Calibration
        self.module_count = 1

    def open(self):
        self.ser = serial.Serial(self.device, self.baud, timeout=0.05)
        self.ser.reset_input_buffer()

    def close(self):
        if self.ser is not None:
            try:
                self.send(CMD_STOP)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def send(self, ptype, payload=b""):
        self.ser.write(build_packet(ptype, payload))
        self.ser.flush()

    def _read_packets(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if data:
            self.reader.feed(data)
        return list(self.reader.packets())

    def _wait_for(self, ptype, count=1, timeout=1.0):
        """Wait for `count` replies of a given type. Returns a list of (addr, payload)."""
        replies = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for addr, t, payload in self._read_packets():
                if t == ptype:
                    replies.append((addr, payload))
                    if len(replies) >= count:
                        return replies
        return replies

    def _flush(self):
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.reader = PacketReader()

    def start(self, timeout=1.0):
        """Stop any running scan, read calibration, and start scanning."""
        self.send(CMD_STOP)
        time.sleep(0.1)
        self._flush()

        # How many units are chained on this port? The reply comes from the
        # last unit in the chain, and its address tells us the count.
        self.send(CMD_GET_ADDRESS)
        replies = self._wait_for(CMD_GET_ADDRESS, timeout=timeout)
        if replies:
            self.module_count = address_to_module(replies[0][0]) + 1
        else:
            self.module_count = 1
        self._flush()

        # Each unit answers with its own calibration values
        self.send(CMD_GET_PARAMETER)
        replies = self._wait_for(CMD_GET_PARAMETER, count=self.module_count, timeout=timeout)
        if not replies:
            raise GS2Error("no calibration reply from %s (check wiring and power)" % self.device)
        self.calibrations = {}
        for addr, payload in replies:
            if len(payload) < 9:
                raise GS2Error("short calibration reply from %s" % self.device)
            self.calibrations[address_to_module(addr)] = Calibration.from_payload(payload)
        self._flush()

        self.send(CMD_SCAN)

    def frames(self, stall_timeout=1.0):
        """
        Yield ScanFrames forever.
        Raises serial.SerialException on disconnect, and GS2Error if no scan
        frame arrives for stall_timeout seconds (so the caller can restart).
        """
        last_frame = time.monotonic()
        while True:
            if time.monotonic() - last_frame > stall_timeout:
                raise GS2Error("no scan data from %s for %.1f s" % (self.device, stall_timeout))
            for addr, ptype, payload in self._read_packets():
                if ptype != CMD_SCAN or len(payload) < 4:
                    continue  # command acknowledgements, version replies, etc.
                module = address_to_module(addr)
                cal = self.calibrations.get(module)
                if cal is None:
                    continue
                points, count = parse_scan_payload(payload, cal)
                last_frame = time.monotonic()
                yield ScanFrame(module, time.monotonic(), points, count)

    @property
    def checksum_errors(self):
        return self.reader.checksum_errors
