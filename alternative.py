#!/usr/bin/env python3
"""
Drop-in replacement for read_moxa_gateway.py

Reads AI, AO, HC, TK from a Moxa MB3270I via Modbus/TCP and writes:
<channel> | <value>
to a text file.

pymodbus compatibility:
- Endian enum names differ across versions (BIG/LITTLE vs Big/Little)
- read_holding_registers() uses unit= or slave= depending on version
"""

import sys
import argparse
from typing import List, Tuple

from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder

# Endian import + enum-name compatibility
try:
    from pymodbus.constants import Endian  # pymodbus >=3.x
    _ENDIAN_BIG = getattr(Endian, "BIG", getattr(Endian, "Big"))
    _ENDIAN_LITTLE = getattr(Endian, "LITTLE", getattr(Endian, "Little"))
except Exception:
    # very old pymodbus
    from pymodbus.payload import Endian  # type: ignore
    _ENDIAN_BIG = getattr(Endian, "BIG", getattr(Endian, "Big"))
    _ENDIAN_LITTLE = getattr(Endian, "LITTLE", getattr(Endian, "Little"))

MB_MAX_REGS_PER_REQ = 120  # conservative chunking under Modbus 125-register max


def endian_from_str(s: str):
    return _ENDIAN_BIG if s.lower() == "big" else _ENDIAN_LITTLE


def _read_holding_registers(client: ModbusTcpClient, address: int, count: int, unit: int):
    """
    Support both APIs:
      - read_holding_registers(..., unit=)
      - read_holding_registers(..., slave=)
    """
    try:
        return client.read_holding_registers(address=address, count=count, unit=unit)
    except TypeError:
        return client.read_holding_registers(address=address, count=count, slave=unit)


def read_block(client: ModbusTcpClient, start_addr: int, reg_count: int, unit: int) -> List[int]:
    """Read a contiguous block of holding registers in safe-size chunks."""
    out: List[int] = []
    addr = start_addr
    remaining = reg_count
    while remaining > 0:
        n = min(MB_MAX_REGS_PER_REQ, remaining)
        rr = _read_holding_registers(client, addr, n, unit)
        if rr.isError():
            raise RuntimeError(f"Modbus error at address {addr} count {n}: {rr}")
        out.extend(rr.registers)
        addr += n
        remaining -= n
    return out


def decode_float(reg_pair: List[int], wordorder, byteorder) -> float:
    dec = BinaryPayloadDecoder.fromRegisters(
        reg_pair, wordorder=wordorder, byteorder=byteorder
    )
    return dec.decode_32bit_float()


def decode_u32(reg_pair: List[int], wordorder, byteorder) -> int:
    dec = BinaryPayloadDecoder.fromRegisters(
        reg_pair, wordorder=wordorder, byteorder=byteorder
    )
    return dec.decode_32bit_uint()


def main():
    p = argparse.ArgumentParser(description="Read Moxa MB3270I data to text file.")
    p.add_argument("--ip", default="192.168.10.11")
    p.add_argument("--port", type=int, default=502)
    p.add_argument("--unit", type=int, default=1, help="Modbus unit/slave ID behind the gateway")
    p.add_argument(
        "--base",
        type=int,
        choices=[0, 1],
        default=0,
        help="Address base. 0 for 0-based (0==40000). 1 for 1-based (0==40001).",
    )
    p.add_argument("--wordorder", choices=["big", "little"], default="big",
                   help="16-bit register word order for 32-bit values")
    p.add_argument("--byteorder", choices=["big", "little"], default="big",
                   help="Byte order inside each 16-bit register")
    p.add_argument("--outfile", default="moxa_values.txt")
    args = p.parse_args()

    wordorder = endian_from_str(args.wordorder)
    byteorder = endian_from_str(args.byteorder)

    client = ModbusTcpClient(host=args.ip, port=args.port, timeout=3.0)
    if not client.connect():
        print(f"Connection failed to {args.ip}:{args.port}", file=sys.stderr)
        sys.exit(2)

    try:
        lines: List[str] = []

        # ---- AI (660 floats) at register offsets 0..1319 (pairs)
        # FID 1.3.1 shows AI.0000 at 0,1 and AI.0659 at 1318,1319.
        ai_start = 0 + args.base
        ai_regs = read_block(client, ai_start, 660 * 2, args.unit)
        for i in range(660):
            pair = ai_regs[2 * i: 2 * i + 2]
            val = decode_float(pair, wordorder, byteorder)
            lines.append(f"AI.{i:04d} | {val:.3f}")

        # ---- AO (44 floats) at offsets 2000..2087 (pairs)
        # FID 1.3.2 shows AO.0000 at 2000,2001 and AO.0043 at 2086,2087.
        ao_start = 2000 + args.base
        ao_regs = read_block(client, ao_start, 44 * 2, args.unit)
        for i in range(44):
            pair = ao_regs[2 * i: 2 * i + 2]
            val = decode_float(pair, wordorder, byteorder)
            lines.append(f"AO.{i:04d} | {val:.3f}")

        # ---- HC (51 counters) listed in Appendix A from 3000..3101 (pairs)
        # FID 1.3.3 + Appendix A provide the explicit tag->offset table.
        hc_map: List[Tuple[str, int]] = [
            ("HC.0171", 3000), ("HC.0174", 3002), ("HC.0190", 3004),
            ("HC.0425", 3006), ("HC.0428", 3008), ("HC.0429", 3010),
            ("HC.0584", 3012), ("HC.0587", 3014), ("HC.0588", 3016),
            ("HC.0614", 3018), ("HC.0615", 3020), ("HC.0616", 3022),
            ("HC.0622", 3024), ("HC.0653", 3026), ("HC.0654", 3028),
            ("HC.0912", 3030), ("HC.0913", 3032), ("HC.0949", 3034),
            ("HC.0952", 3036), ("HC.1320", 3038), ("HC.1323", 3040),
            ("HC.1326", 3042), ("HC.1329", 3044), ("HC.1336", 3046),
            ("HC.1343", 3048), ("HC.1346", 3050), ("HC.1349", 3052),
            ("HC.1352", 3054), ("HC.1353", 3056), ("HC.1354", 3058),
            ("HC.1360", 3060), ("HC.1363", 3062), ("HC.1367", 3064),
            ("HC.1370", 3066), ("HC.1373", 3068), ("HC.1380", 3070),
            ("HC.1387", 3072), ("HC.1390", 3074), ("HC.1393", 3076),
            ("HC.1396", 3078), ("HC.1399", 3080), ("HC.1400", 3082),
            ("HC.1401", 3084), ("HC.1696", 3086), ("HC.1704", 3088),
            ("HC.1712", 3090), ("HC.1721", 3092), ("HC.1725", 3094),
            ("HC.1836", 3096), ("HC.1908", 3098), ("HC.1909", 3100),
        ]
        hc_block_start = 3000 + args.base
        hc_regs = read_block(client, hc_block_start, 51 * 2, args.unit)
        for tag, addr in hc_map:
            idx = (addr - 3000) // 2
            pair = hc_regs[2 * idx: 2 * idx + 2]
            val = decode_u32(pair, wordorder, byteorder)
            lines.append(f"{tag} | {val}")

        # ---- TK (52 floats) at offsets 4000..4103 (pairs)
        # FID 1.3.4 shows TK.0000 at 4000,4001 and TK.0051 at 4102,4103.
        tk_start = 4000 + args.base
        tk_regs = read_block(client, tk_start, 52 * 2, args.unit)
        for i in range(52):
            pair = tk_regs[2 * i: 2 * i + 2]
            val = decode_float(pair, wordorder, byteorder)
            lines.append(f"TK.{i:04d} | {val:.3f}")

        with open(args.outfile, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

        print(f"Wrote {len(lines)} tags to {args.outfile}")

    finally:
        try:
            client.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()