#!/usr/bin/env python3
"""Read-only probe of the AP002 tactile registers, to check the sensor layout on the real hand.

  ros2 run finger_node tactile_probe                      # /dev/ttyUSB0
  ros2 run finger_node tactile_probe --port /dev/ttyUSB1 --csv probe.csv

Never writes to the hand. Do not run together with hand_serial (one program per serial port).
Shows, about 5 times a second, the values that changed against the first reading (baseline):
  fingers (FORCE_EX0..4): each point decoded as [normal, tangential, direction] (byte-swapped
                          16-bit, 0xFFFF = invalid), as the OYMotion demo force_chart.py does
  palm    (FORCE_EX5)   : 28 registers shown two ways, to decide which one is right:
                          A = high byte only (28 values, what force_chart.py uses)
                          B = high and low byte (56 values -> first 55 as an 11 x 5 grid)
  FORCE_EX6..8          : read too, in case the hand reports more groups
Press one spot at a time (fingertip, proximal phalanx, palm areas) and watch what changes.
Press Enter (hand untouched) to take a new baseline.
"""
import argparse
import csv
import select
import sys
import time

SLAVE_ID = 2
PROTO_ADDR, HW_ADDR, MANU_DATA_ADDR = 1000, 1003, 1265
FORCE_EX_ADDR, GROUP_SIZE = 2000, 100
EXTRA_GROUPS = (6, 7, 8)
EXTRA_COUNT = 10
LAYOUTS = {  # sub_model -> registers per group (thumb, index, middle, ring, little, palm)
    0: ('dot_matrix', (18, 30, 30, 30, 16, 28)),
    1: ('force3d', (3, 6, 6, 6, 3, 28)),
}
NAMES = ('thumb', 'index', 'middle', 'ring', 'little', 'palm')


def swap(reg):
    return ((reg & 0xFF) << 8) | ((reg >> 8) & 0xFF)


class Probe:
    def __init__(self, port):
        from pymodbus.client import ModbusSerialClient
        self.client = ModbusSerialClient(port=port, baudrate=115200, timeout=1)
        if not self.client.connect():
            sys.exit(f'serial port open failed: {port}')

    def read(self, address, count):
        result = self.client.read_holding_registers(address=address, count=count, slave=SLAVE_ID)
        if result is None or result.isError() or len(result.registers) != count:
            return None
        return list(result.registers)


def finger_points(kind, regs):
    """force3d: list of (normal, tangential, direction); dot_matrix: taxel bytes."""
    if kind == 'force3d':
        values = [swap(r) for r in regs]
        values = [None if v == 0xFFFF else v for v in values]
        return [tuple(values[i:i + 3]) for i in range(0, len(values) - 2, 3)]
    taxels = []
    for r in regs:
        taxels += [(r >> 8) & 0xFF, r & 0xFF]
    return taxels


def enter_pressed():
    """True if Enter was pressed since the last call (terminal only)."""
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        sys.stdin.readline()
        return True
    return False


def grid(values, rows=11, cols=5):
    lines = []
    for r in range(rows):
        row = values[r * cols:(r + 1) * cols]
        lines.append(' '.join(f'{v:3d}' for v in row))
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--rate', type=float, default=5.0)
    parser.add_argument('--csv', default='', help='also log every reading (raw registers) to this file')
    args, _ = parser.parse_known_args()

    probe = Probe(args.port)
    proto, hw, manu = probe.read(PROTO_ADDR, 1), probe.read(HW_ADDR, 1), probe.read(MANU_DATA_ADDR, 1)
    if proto is None or hw is None:
        sys.exit('no Modbus answer (power, wiring, baud 115200, slave id 2)')
    sub_model = (manu[0] >> 8) & 0xFF if manu else None
    print(f'protocol=0x{proto[0]:04X} hardware=0x{hw[0]:04X} manu_data0=0x{manu[0]:04X} '
          f'-> sub_model={sub_model}' if manu else 'manu_data0 unreadable')
    kind, counts = LAYOUTS.get(sub_model, LAYOUTS[1])
    print(f'tactile layout: {kind} {counts}\n')

    writer = None
    if args.csv:
        csv_file = open(args.csv, 'w', newline='')
        writer = csv.writer(csv_file)
        writer.writerow(['time'] + [f'{NAMES[g]}_reg{i}' for g, n in enumerate(counts) for i in range(n)]
                        + [f'ex{g}_reg{i}' for g in EXTRA_GROUPS for i in range(EXTRA_COUNT)])

    baseline = None
    try:
        while True:
            started = time.monotonic()
            groups = [probe.read(FORCE_EX_ADDR + g * GROUP_SIZE, n) for g, n in enumerate(counts)]
            extra = [probe.read(FORCE_EX_ADDR + g * GROUP_SIZE, EXTRA_COUNT) for g in EXTRA_GROUPS]
            if baseline is None or enter_pressed():
                baseline = groups
            if writer:
                row = [round(time.time(), 3)]
                for regs, n in zip(groups + extra, list(counts) + [EXTRA_COUNT] * len(EXTRA_GROUPS)):
                    row += regs if regs is not None else [''] * n
                writer.writerow(row)

            out = ['\033[2J\033[H', f'{kind}  (Enter: new baseline, Ctrl+C: quit)  changed vs. baseline marked *', '']
            for g in range(5):
                regs, base = groups[g], baseline[g]
                if regs is None:
                    out.append(f'{NAMES[g]:7} read error')
                    continue
                mark = '*' if regs != base else ' '
                if kind == 'force3d':
                    points = finger_points(kind, regs)
                    text = '  '.join(f'p{i}[n={p[0]} t={p[1]} dir={p[2]}]' for i, p in enumerate(points))
                else:
                    taxels = finger_points(kind, regs)
                    text = f'sum={sum(taxels)} max={max(taxels)}'
                out.append(f'{mark} {NAMES[g]:7} {text}')
                out.append(f'          raw {" ".join(f"{r:04X}" for r in regs)}')
            palm, base = groups[5], baseline[5]
            out.append('')
            if palm is None:
                out.append('palm read error')
            else:
                changed = [i for i, (a, b) in enumerate(zip(palm, base)) if a != b]
                high = [(r >> 8) & 0xFF for r in palm]
                both = []
                for r in palm:
                    both += [(r >> 8) & 0xFF, r & 0xFF]
                low_changed = any((a & 0xFF) != (b & 0xFF) for a, b in zip(palm, base))
                out.append(f'palm: changed registers {changed}  | low bytes changed: {low_changed}')
                out.append(f'  A (high byte only, 28): {" ".join(f"{v:3d}" for v in high)}')
                out.append('  B (both bytes, first 55 as 11 x 5):')
                out += ['    ' + line for line in grid(both[:55])]
                out.append(f'    (56th value: {both[55]})')
            out.append('')
            for g, regs in zip(EXTRA_GROUPS, extra):
                text = 'read error' if regs is None else ' '.join(f'{r:04X}' for r in regs)
                out.append(f'EX{g}: {text}')
            print('\n'.join(out), flush=True)
            time.sleep(max(0.0, 1.0 / args.rate - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        probe.client.close()
        if writer:
            csv_file.close()
            print(f'saved {args.csv}')


if __name__ == '__main__':
    main()
