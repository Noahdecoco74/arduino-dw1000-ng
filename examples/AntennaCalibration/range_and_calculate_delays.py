
"""Calibrate DW1000 antenna delays from three anchors.

This script expects three Arduino/serial endpoints running the calibration
examples from this repository. Each board must respond to:

* `100` -> return its anchor id on a single line.
* `<peer_id>` -> run TWR against that peer and eventually print
  `average:<from>:<to>:<distance_meters>`.

The calibration uses the three-device EDM approach described in the prompt:
it builds the physical distance matrix, collects the six ordered RF ranges,
and searches for the aggregate antenna delays that minimize the residual
between the measured and actual time-of-flight matrices.

The final aggregate delays are split into Tx/Rx using the 44% / 56% ratio.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import random
import re
import sys
import time
from typing import Dict, List, Mapping, Sequence, Tuple

try:
    import serial
except ImportError as exc:  # pragma: no cover - runtime dependency check
    raise SystemExit(
        "pyserial is required. Install it with: python3 -m pip install pyserial"
    ) from exc


SPEED_OF_LIGHT_M_S = 299_792_458.0
DW1000_TIME_UNIT_S = 1.0 / (499.2e6 * 128.0)
TX_RATIO = 0.44
RX_RATIO = 0.56


@dataclasses.dataclass(frozen=True)
class AnchorDevice:
    port: str
    anchor_id: int


@dataclasses.dataclass
class CalibrationResult:
    aggregate_delay_ns: Dict[int, float]
    tx_delay_ns: Dict[int, float]
    rx_delay_ns: Dict[int, float]
    aggregate_delay_raw: Dict[int, int]
    error_ns: float
    measured_tof_s: Dict[Tuple[int, int], float]


def distance_to_tof(distance_m: float) -> float:
    """Convertis des mètres en secondes de tof"""
    return distance_m / SPEED_OF_LIGHT_M_S


def seconds_to_dw1000_units(seconds: float) -> float:
    return seconds / DW1000_TIME_UNIT_S


def ns_to_seconds(value_ns: float) -> float:
    return value_ns * 1e-9


def seconds_to_ns(value_s: float) -> float:
    return value_s * 1e9


def build_distance_matrix(distance_between_anchors: float, distances: Mapping[Tuple[int, int], float] | None = None) -> Dict[Tuple[int, int], float]:
    matrix: Dict[Tuple[int, int], float] = {}
    anchor_ids = (1, 2, 3)

    if distances is None:
        for source_id in anchor_ids:
            for target_id in anchor_ids:
                if source_id == target_id:
                    continue
                matrix[(source_id, target_id)] = distance_between_anchors
        return matrix

    for source_id in anchor_ids:
        for target_id in anchor_ids:
            if source_id == target_id:
                continue
            if (source_id, target_id) in distances:
                matrix[(source_id, target_id)] = float(distances[(source_id, target_id)])
            elif (target_id, source_id) in distances:
                matrix[(source_id, target_id)] = float(distances[(target_id, source_id)])
            else:
                raise ValueError(f"Missing distance for pair {source_id}->{target_id}")
    return matrix


def parse_anchor_id(line: str) -> int | None:
    stripped = line.strip()
    if not stripped:
        return None
    if re.fullmatch(r"\d+", stripped):
        return int(stripped)
    return None


def parse_average_line(line: str) -> Tuple[int, int, float] | None:
    stripped = line.strip()
    match = re.fullmatch(r"average:(\d+):(\d+):([-+0-9.eE]+)", stripped)
    if not match:
        return None
    return int(match.group(1)), int(match.group(2)), float(match.group(3))/1000


class AnchorSerial:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.15) -> None:
        self.port = port
        self._serial = serial.Serial(port, baudrate=baudrate, timeout=timeout, write_timeout=timeout)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def close(self) -> None:
        self._serial.close()

    def read_line(self, deadline_s: float) -> str | None:
        while time.monotonic() < deadline_s:
            raw = self._serial.readline()
            if raw:
                return raw.decode("utf-8", errors="replace").strip()
        return None

    def query_anchor_id(self, timeout_s: float = 5.0) -> int:
        self._serial.reset_input_buffer()
        self._serial.write(b"100\n")
        self._serial.flush()

        deadline_s = time.monotonic() + timeout_s
        while True:
            line = self.read_line(deadline_s)
            if line is None:
                raise TimeoutError(f"Timed out waiting for anchor id on {self.port}")
            anchor_id = parse_anchor_id(line)
            if anchor_id is not None:
                return anchor_id

    def measure_distance(self, peer_id: int, timeout_s: float = 15.0) -> float:
        self._serial.reset_input_buffer()
        self._serial.write(f"{peer_id}".encode("ascii"))
        self._serial.flush()

        deadline_s = time.monotonic() + timeout_s
        while True:
            line = self.read_line(deadline_s)
            if line is None:
                raise TimeoutError(f"Timed out waiting for range result on {self.port} -> {peer_id}")
            if line == "timeout":
                raise TimeoutError(f"Anchor on {self.port} reported timeout while ranging to {peer_id}")
            parsed = parse_average_line(line)
            if parsed is not None:
                source_id, target_id, distance_m = parsed
                print(f"got:{source_id}:{target_id}:{distance_m}")
                if target_id != peer_id:
                    continue
                return distance_m


def discover_anchors(port_names: Sequence[str], baudrate: int = 115200) -> List[AnchorDevice]:
    anchors: List[AnchorDevice] = []
    opened_ports: List[AnchorSerial] = []
    try:
        for port_name in port_names:
            connection = AnchorSerial(port_name, baudrate=baudrate)
            opened_ports.append(connection)
            anchor_id = connection.query_anchor_id()
            anchors.append(AnchorDevice(port=port_name, anchor_id=anchor_id))
        return anchors
    finally:
        for connection in opened_ports:
            connection.close()


def collect_measurements(port_names: Sequence[str], baudrate: int = 115200) -> Dict[Tuple[int, int], float]:
    anchors: List[Tuple[AnchorDevice, AnchorSerial]] = []
    try:
        for port_name in port_names:
            connection = AnchorSerial(port_name, baudrate=baudrate)
            anchor_id = connection.query_anchor_id()
            anchors.append((AnchorDevice(port=port_name, anchor_id=anchor_id), connection))

        id_to_connection = {anchor.anchor_id: connection for anchor, connection in anchors}
        measurements: Dict[Tuple[int, int], float] = {}

        for source_anchor, _ in anchors:
            for target_anchor, _ in anchors:
                if source_anchor.anchor_id == target_anchor.anchor_id:
                    continue
                distance_m = id_to_connection[source_anchor.anchor_id].measure_distance(target_anchor.anchor_id)
                measurements[(source_anchor.anchor_id, target_anchor.anchor_id)] = distance_m

        return measurements
    finally:
        for _, connection in anchors:
            connection.close()



def find_delays(distance: float,
                measured_distances_m: Mapping[Tuple[int, int], float]):
    
    Tab = (measured_distances_m[(1, 2)] + measured_distances_m[(2, 1)])/2 - distance
    Tbc = (measured_distances_m[(2, 3)] + measured_distances_m[(3, 2)])/2 - distance
    Tca = (measured_distances_m[(3, 1)] + measured_distances_m[(1, 3)])/2 - distance
    
    Tab = seconds_to_dw1000_units(distance_to_tof(Tab))*2
    Tbc = seconds_to_dw1000_units(distance_to_tof(Tbc))*2
    Tca = seconds_to_dw1000_units(distance_to_tof(Tca))*2
    
    Ta = (Tab + Tca - Tbc)/2
    Tb = round(Tab - Ta)
    Tc = round(Tca - Ta)
    Ta = round(Ta)

    print(f"You need to add:\n{Ta=}\n{Tb=}\n{Tc=}")


def format_matrix(values: Mapping[Tuple[int, int], float], unit_label: str) -> str:
    rows = []
    for source_id in (1, 2, 3):
        row = []
        for target_id in (1, 2, 3):
            if source_id == target_id:
                row.append("0.000000")
            else:
                row.append(f"{values[(source_id, target_id)]:.6f}")
        rows.append(f"{source_id}: " + ", ".join(row) + f" {unit_label}")
    return "\n".join(rows)


def parse_distance_override(args: argparse.Namespace) -> Dict[Tuple[int, int], float] | None:
    pairwise = {
        (1, 2): args.distance_12,
        (1, 3): args.distance_13,
        (2, 3): args.distance_23,
    }
    if all(value is None for value in pairwise.values()):
        return None
    if any(value is None for value in pairwise.values()):
        raise SystemExit(
            "Provide either --distance for an equidistant setup, or all of --distance-12, --distance-13 and --distance-23."
        )
    return pairwise  # type: ignore[return-value]


def print_result(result: CalibrationResult) -> None:
    print("Measured ToF matrix (ns):")
    measured_tof_ns = {pair: seconds_to_ns(tof_s) for pair, tof_s in result.measured_tof_s.items()}
    print(format_matrix(measured_tof_ns, "ns"))
    print()
    print(f"Optimization error: {result.error_ns:.6f} ns")
    print("Aggregate delays per anchor:")
    for anchor_id in (1, 2, 3):
        aggregate_ns = result.aggregate_delay_ns[anchor_id]
        tx_ns = result.tx_delay_ns[anchor_id]
        rx_ns = result.rx_delay_ns[anchor_id]
        raw_units = result.aggregate_delay_raw[anchor_id]
        print(
            f"  anchor {anchor_id}: aggregate={aggregate_ns:.6f} ns, tx={tx_ns:.6f} ns, "
            f"rx={rx_ns:.6f} ns, raw={raw_units}"
        )
    print()
    print("Arduino helper values:")
    for anchor_id in (1, 2, 3):
        print(f"  anchor {anchor_id}: setAntennaDelay({result.aggregate_delay_raw[anchor_id]})")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Calibrate DW1000 antenna delays from three anchors.")
    parser.add_argument(
        "--port",
        dest="ports",
        action="append",
        required=True,
        help="Serial port for one anchor. Pass this option three times.",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Serial baudrate used by the Arduino examples.",
    )
    parser.add_argument(
        "--distance",
        type=float,
        default=0.31,
        help="Uniform physical distance between every pair of anchors in meters.",
    )
    parser.add_argument("--distance-12", type=float, default=0.32, help="Physical distance between anchors 1 and 2.")
    parser.add_argument("--distance-13", type=float, default=0.32, help="Physical distance between anchors 1 and 3.")
    parser.add_argument("--distance-23", type=float, default=0.32, help="Physical distance between anchors 2 and 3.")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    if len(args.ports) != 3:
        parser.error("Exactly three --port values are required.")

    distance_overrides = parse_distance_override(args)
    actual_distances_m = build_distance_matrix(args.distance, distance_overrides)

    print("Discovering anchors...")
    anchors = discover_anchors(args.ports, baudrate=args.baudrate)
    for anchor in anchors:
        print(f"  {anchor.port} -> anchor {anchor.anchor_id}")

    if sorted(anchor.anchor_id for anchor in anchors) != [1, 2, 3]:
        raise SystemExit("Expected anchor ids 1, 2 and 3 across the three serial ports.")

    print("\nCollecting measurements...")
    measured_distances_m = collect_measurements(args.ports, baudrate=args.baudrate)
    print(format_matrix(measured_distances_m, "m"))
    find_delays(distance=args.distance, measured_distances_m=measured_distances_m)

    print()
    #print_result(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())



"""
distance_between_anchors = 0.31


# Connect to the 3 anchors with serial 
# /dev/ttyUSB0, /dev/ttyUSB1 and /dev/ttyUSB2

# Ask the anchor's id by puting "100" in the console, the id will be returned on one line
# ex : "1\n"

# Range the 6 combinations
# for example send "2" to anchor 1 in order to start ranging 1->2
# it will return "timeout", or "average:1:2:0.24" -> initiator:slave:distance_in_meters

# Solve the optimisation problem to calculate the delays
"""