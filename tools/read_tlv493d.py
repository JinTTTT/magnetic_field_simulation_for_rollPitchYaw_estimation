"""Read both TLV493D sensors (bus 3 and bus 4, addr 0x5E) and print Bx,By,Bz in mT."""
import time

from adafruit_extended_bus import ExtendedI2C
import adafruit_tlv493d

SENSORS = [("S1", 3), ("S2", 4)]


def open_sensors():
    out = []
    for name, bus in SENSORS:
        i2c = ExtendedI2C(bus)
        out.append((name, bus, adafruit_tlv493d.TLV493D(i2c)))
    return out


def main():
    sensors = open_sensors()
    print(f"{'sensor':>6} {'bus':>3} {'Bx (mT)':>9} {'By (mT)':>9} {'Bz (mT)':>9}")
    for _ in range(5):
        for name, bus, s in sensors:
            bx, by, bz = s.magnetic  # microtesla
            print(f"{name:>6} {bus:>3} {bx/1000:9.3f} {by/1000:9.3f} {bz/1000:9.3f}")
        print()
        time.sleep(0.2)


if __name__ == "__main__":
    main()
