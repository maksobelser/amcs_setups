#!/usr/bin/env python3
"""
Pull measurements from the AMCS -> DeepBlue Modbus link (via the Moxa MB3270I
gateway) and dump them to a pipe-delimited text file in the format
`CHANNEL | VALUE`.

The register layout and tag naming follow the latest AMCS FID (23000666-I601-FID-R1.02):
    - AI.0000 .. AI.0659   @ offsets 0..1319 (float, 2 registers per channel)
    - AO.0000 .. AO.0043   @ offsets 2000..2087 (float)
    - HC.<list from App. A> @ offsets 3000..3101 (double word / uint32)
    - TK.0000 .. TK.0051   @ offsets 4000..4103 (float)

Usage example:
    python read_moxa_gateway.py --host 192.168.10.11 --unit-id 1 --output logbook_snapshot.txt
"""

from __future__ import annotations

import argparse
import datetime as dt
import logging
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence

try:
    # pymodbus >= 3.0
    from pymodbus.client import ModbusTcpClient
except ImportError:  # pragma: no cover - fallback for pymodbus 2.x
    from pymodbus.client.sync import ModbusTcpClient  # type: ignore


@dataclass(frozen=True)
class RegisterBlock:
    """Description of a contiguous register block that holds channel values."""

    label: str
    channel_ids: Sequence[str]
    start_register: int  # 0-based Modbus offset (FC3/FC16 holding register address)
    value_type: str  # "float" or "uint32"

    @property
    def register_count(self) -> int:
        return len(self.channel_ids) * 2  # 2 registers per 32-bit value


# Hour-counter identifiers copied from Appendix A in 23000666-I601-FID-R1.02
HOUR_COUNTER_IDS: Sequence[str] = (
    "HC.0171",
    "HC.0174",
    "HC.0190",
    "HC.0425",
    "HC.0428",
    "HC.0429",
    "HC.0584",
    "HC.0587",
    "HC.0588",
    "HC.0614",
    "HC.0615",
    "HC.0616",
    "HC.0622",
    "HC.0653",
    "HC.0654",
    "HC.0912",
    "HC.0913",
    "HC.0949",
    "HC.0952",
    "HC.1320",
    "HC.1323",
    "HC.1326",
    "HC.1329",
    "HC.1336",
    "HC.1343",
    "HC.1346",
    "HC.1349",
    "HC.1352",
    "HC.1353",
    "HC.1354",
    "HC.1360",
    "HC.1363",
    "HC.1367",
    "HC.1370",
    "HC.1373",
    "HC.1380",
    "HC.1387",
    "HC.1390",
    "HC.1393",
    "HC.1396",
    "HC.1399",
    "HC.1400",
    "HC.1401",
    "HC.1696",
    "HC.1704",
    "HC.1712",
    "HC.1721",
    "HC.1725",
    "HC.1836",
    "HC.1908",
    "HC.1909",
)

# Register layout derived from Section 1.3 of the FID.
REGISTER_BLOCKS: Sequence[RegisterBlock] = (
    RegisterBlock(
        label="Analog inputs",
        channel_ids=tuple(f"AI.{index:04d}" for index in range(660)),
        start_register=0,
        value_type="float",
    ),
    RegisterBlock(
        label="Analog outputs",
        channel_ids=tuple(f"AO.{index:04d}" for index in range(44)),
        start_register=2000,
        value_type="float",
    ),
    RegisterBlock(
        label="Hour counters",
        channel_ids=HOUR_COUNTER_IDS,
        start_register=3000,
        value_type="uint32",
    ),
    RegisterBlock(
        label="Tank volumes",
        channel_ids=tuple(f"TK.{index:04d}" for index in range(52)),
        start_register=4000,
        value_type="float",
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read AMCS tags from the Moxa gateway and export channel/value pairs.",
    )
    parser.add_argument("--host", required=True, help="IP or hostname of the Moxa MB3270I (e.g. 192.168.10.11)")
    parser.add_argument("--port", type=int, default=502, help="Modbus TCP port (default: 502)")
    parser.add_argument("--unit-id", type=int, default=1, help="Unit ID / Slave ID exposed by the Moxa (default: 1)")
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=120,
        help="Max number of registers per Modbus read (default: 120, must be <= 125).",
    )
    parser.add_argument(
        "--word-order",
        choices=("big", "little"),
        default="big",
        help="Register word order for 32-bit values (default: big).",
    )
    parser.add_argument(
        "--byte-order",
        choices=("big", "little"),
        default="big",
        help="Byte order inside a 16-bit register (default: big).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("moxa_logbook_snapshot.txt"),
        help="Destination text file for `CHANNEL | VALUE` lines.",
    )
    parser.add_argument("--timeout", type=float, default=3.0, help="TCP timeout in seconds (default: 3.0)")
    parser.add_argument(
        "--log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
        help="Console log level.",
    )
    return parser.parse_args()


def read_registers(
    client: ModbusTcpClient,
    start_address: int,
    register_count: int,
    unit_id: int,
    chunk_size: int,
) -> List[int]:
    """Read `register_count` registers starting at `start_address`, honoring Modbus limits."""
    if register_count <= 0:
        return []

    registers: List[int] = []
    already_read = 0
    while already_read < register_count:
        this_read = min(chunk_size, register_count - already_read)
        response = _call_read_holding_registers(client, start_address + already_read, this_read, unit_id)
        if response.isError():
            raise RuntimeError(f"Modbus error while reading addr {start_address + already_read}: {response}")
        registers.extend(response.registers)
        already_read += this_read
    return registers


_READ_CALL_STYLE: Optional[str] = None


def _call_read_holding_registers(
    client: ModbusTcpClient,
    address: int,
    count: int,
    unit_id: int,
):
    """Call `read_holding_registers` while handling different pymodbus signatures."""
    global _READ_CALL_STYLE
    _set_client_unit(client, unit_id)

    base_styles = ["unit_kw", "slave_kw", "kw_only", "positional_two", "positional_one"]
    if _READ_CALL_STYLE and _READ_CALL_STYLE in base_styles:
        styles = [_READ_CALL_STYLE] + [style for style in base_styles if style != _READ_CALL_STYLE]
    else:
        styles = base_styles

    last_error: Optional[TypeError] = None
    for style in styles:
        try:
            if style == "unit_kw":
                response = client.read_holding_registers(address=address, count=count, unit=unit_id)
            elif style == "slave_kw":
                response = client.read_holding_registers(address=address, count=count, slave=unit_id)
            elif style == "kw_only":
                response = client.read_holding_registers(address=address, count=count)
            elif style == "positional_two":
                response = client.read_holding_registers(address, count)
            elif style == "positional_one":
                response = client.read_holding_registers(address)
            else:
                continue
        except TypeError as err:
            last_error = err
            continue

        _READ_CALL_STYLE = style
        return response

    raise last_error or TypeError("Unsupported pymodbus read_holding_registers signature")


def _set_client_unit(client: ModbusTcpClient, unit_id: int) -> None:
    """Best-effort attempt to store the slave/unit id on the client instance."""
    for attr in ("unit_id", "unit", "slave_id", "slave"):
        if hasattr(client, attr):
            try:
                setattr(client, attr, unit_id)
            except Exception:  # noqa: BLE001 - some attrs might be properties
                continue


def decode_values(
    registers: Sequence[int],
    block: RegisterBlock,
    word_order: str,
    byte_order: str,
) -> List[float]:
    """Decode the raw register list into Python numbers (float or uint32)."""
    if len(registers) != block.register_count:
        raise ValueError(f"Expected {block.register_count} registers for {block.label}, got {len(registers)}")

    values: List[float] = []
    fmt_prefix = ">" if byte_order == "big" else "<"
    for idx in range(0, len(registers), 2):
        pair = [registers[idx], registers[idx + 1]]
        if word_order == "little":
            pair.reverse()
        raw = b"".join(word.to_bytes(2, byteorder=byte_order, signed=False) for word in pair)
        if block.value_type == "float":
            values.append(struct.unpack(fmt_prefix + "f", raw)[0])
        elif block.value_type == "uint32":
            values.append(float(struct.unpack(fmt_prefix + "I", raw)[0]))
        else:
            raise ValueError(f"Unsupported value type {block.value_type}")
    return values


def format_value(value: float, value_type: str) -> str:
    if value_type == "uint32":
        return f"{int(value)}"
    # Preserve sign and three decimals for analog/tank readings.
    return f"{value:.3f}"


def collect_snapshot(
    client: ModbusTcpClient,
    unit_id: int,
    chunk_size: int,
    word_order: str,
    byte_order: str,
) -> List[tuple[str, str]]:
    """Read all configured blocks and return (channel_id, formatted_value) pairs."""
    snapshot: List[tuple[str, str]] = []
    for block in REGISTER_BLOCKS:
        logging.info("Reading %s (%d channels starting at %d)", block.label, len(block.channel_ids), block.start_register)
        raw = read_registers(client, block.start_register, block.register_count, unit_id, chunk_size)
        values = decode_values(raw, block, word_order, byte_order)
        for channel_id, value in zip(block.channel_ids, values):
            snapshot.append((channel_id, format_value(value, block.value_type)))
    return snapshot


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(message)s",
    )

    word_order = args.word_order
    byte_order = args.byte_order

    client = ModbusTcpClient(host=args.host, port=args.port, timeout=args.timeout)
    if not client.connect():
        logging.error("Unable to connect to %s:%s", args.host, args.port)
        return 1

    try:
        snapshot = collect_snapshot(
            client,
            unit_id=args.unit_id,
            chunk_size=args.chunk_size,
            word_order=word_order,
            byte_order=byte_order,
        )
    finally:
        client.close()

    timestamp = dt.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S UTC")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="ascii") as fh:
        fh.write(f"# AMCS snapshot generated {timestamp}\n")
        for channel_id, value in snapshot:
            fh.write(f"{channel_id} | {value}\n")
    logging.info("Wrote %d channel values to %s", len(snapshot), args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
