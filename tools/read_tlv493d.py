"""Read both TLV493D sensors (bus 3 and bus 4, addr 0x5E) and print Bx,By,Bz in mT."""
import time

from tlv493d_coherent import open_sensor_pair, prime_sensor_pair

SENSORS = [("S1", 3), ("S2", 4)]


def open_sensors():
    _i2c_buses, sensors = open_sensor_pair(SENSORS[0][1], SENSORS[1][1])
    return [
        (name, bus, sensor)
        for (name, bus), sensor in zip(SENSORS, sensors)
    ]


def main():
    sensors = open_sensors()
    prime_sensor_pair([sensor for _name, _bus, sensor in sensors])
    print(f"{'sensor':>6} {'bus':>3} {'Bx (mT)':>9} {'By (mT)':>9} {'Bz (mT)':>9}")
    for _ in range(5):
        for name, bus, s in sensors:
            bx, by, bz = s.magnetic  # coherent microtesla frame
            print(f"{name:>6} {bus:>3} {bx/1000:9.3f} {by/1000:9.3f} {bz/1000:9.3f}")
        print()
        time.sleep(0.2)


if __name__ == "__main__":
    main()
