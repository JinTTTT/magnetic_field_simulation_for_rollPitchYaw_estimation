"""Coherent TLV493D-A1B6 magnetic reads with frame-status validation.

The sensor exposes each 12-bit axis across an MSB register and a shadowed LSB
register. Infineon's reference implementation treats a read as coherent only
when the CHANNEL status in register 0x03 is zero. The Adafruit driver decodes
every register read without checking that status, so this wrapper retries until
all axes belong to one complete measurement frame.
"""

import time


LSB_UT = 98.0
COMPLETE_FRAME_CHANNEL = 0
READER_TYPE = "tlv493d_a1b6_complete_frame_v1"
COHERENCE_RULE = "accept only frames with CHANNEL status equal to zero"
DEFAULT_CONVERSION_WAIT = 0.02


class CoherentReadError(RuntimeError):
    """Raised when no complete TLV493D frame is obtained within the limit."""


def decode_signed_12(msb, lsb_nibble):
    """Decode one signed 12-bit axis from its MSB and four-bit LSB."""
    value = (msb << 4) | (lsb_nibble & 0x0F)
    if value & 0x800:
        value -= 0x1000
    return value


def decode_registers(registers):
    """Return raw signed counts (Bx, By, Bz) from a ten-byte read frame."""
    if len(registers) < 6:
        raise ValueError("a TLV493D frame must contain at least six bytes")
    return (
        decode_signed_12(registers[0], registers[4] >> 4),
        decode_signed_12(registers[1], registers[4]),
        decode_signed_12(registers[2], registers[5]),
    )


class CoherentTLV493D:
    """Drop-in magnetic interface that rejects incomplete conversion frames."""

    def __init__(self, chip, max_attempts=50, retry_delay=0.0002):
        if max_attempts < 1:
            raise ValueError("max_attempts must be at least 1")
        if retry_delay < 0:
            raise ValueError("retry_delay cannot be negative")
        self.chip = chip
        self.max_attempts = max_attempts
        self.retry_delay = retry_delay
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.last_attempts = 0
        self.last_channel = None
        self.last_frame_counter = None
        self.last_registers = None

    def read_counts(self):
        """Read one complete frame and return signed 12-bit axis counts."""
        for attempt in range(1, self.max_attempts + 1):
            # Adafruit 2.0.12 exposes the complete ten-byte register snapshot
            # through these attributes but does not validate CHANNEL itself.
            self.chip._read_i2c()
            registers = bytes(self.chip.read_buffer)
            channel = registers[3] & 0x03
            frame_counter = (registers[3] >> 2) & 0x03
            if channel == COMPLETE_FRAME_CHANNEL:
                self.accepted_frames += 1
                self.rejected_frames += attempt - 1
                self.last_attempts = attempt
                self.last_channel = channel
                self.last_frame_counter = frame_counter
                self.last_registers = registers
                return decode_registers(registers)
            if self.retry_delay:
                time.sleep(self.retry_delay)

        self.rejected_frames += self.max_attempts
        self.last_attempts = self.max_attempts
        self.last_channel = channel
        self.last_frame_counter = frame_counter
        self.last_registers = registers
        raise CoherentReadError(
            f"no complete TLV493D frame after {self.max_attempts} reads "
            f"(last channel={channel}, frame_counter={frame_counter})"
        )

    @property
    def magnetic(self):
        """Coherent (Bx, By, Bz) in microteslas, matching Adafruit's API."""
        return tuple(value * LSB_UT for value in self.read_counts())

    @property
    def magnetic_mT(self):
        """Coherent (Bx, By, Bz) in milliteslas."""
        return tuple(value * LSB_UT / 1000.0 for value in self.read_counts())

    def diagnostics(self):
        return {
            "accepted_frames": self.accepted_frames,
            "rejected_frames": self.rejected_frames,
            "last_attempts": self.last_attempts,
            "last_channel": self.last_channel,
            "last_frame_counter": self.last_frame_counter,
        }


def open_sensor_pair(bus1=3, bus2=4):
    """Open two independent-bus TLV493D sensors with coherent readers."""
    from adafruit_extended_bus import ExtendedI2C
    import adafruit_tlv493d

    i2c_buses = [ExtendedI2C(bus1), ExtendedI2C(bus2)]
    sensors = [
        CoherentTLV493D(adafruit_tlv493d.TLV493D(i2c_bus))
        for i2c_bus in i2c_buses
    ]
    return i2c_buses, sensors


def read_pair_mT(sensors):
    """Return [S1 Bx,By,Bz, S2 Bx,By,Bz] from coherent frames."""
    fields = []
    for sensor in sensors:
        fields.extend(sensor.magnetic_mT)
    return fields


def prime_sensor_pair(sensors, conversion_wait=DEFAULT_CONVERSION_WAIT):
    """Discard the pre-trigger frame and wait for a fresh pose measurement.

    In the configured master-controlled behavior, the first coherent read after
    a pose change can still contain the previous pose while triggering the next
    conversion. Discard it before starting an averaged acquisition window.
    """
    if conversion_wait < 0:
        raise ValueError("conversion_wait cannot be negative")
    read_pair_mT(sensors)
    if conversion_wait:
        time.sleep(conversion_wait)
