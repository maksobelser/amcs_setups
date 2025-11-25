#!/usr/bin/env python3
# Minimal Modbus/TCP server with mixed-encoding decode (float32 or int16+scale).
# Leave running to accept writes, and emit "<channel> | <value>" to the outfile.

import argparse, socketserver, struct, threading, time, math
from typing import List, Tuple

AI_N, AO_N, HC_N, TK_N = 660, 44, 51, 52
REGS = [0] * 4104
REG_LOCK = threading.Lock()
LAST_WRITE_TS = 0.0

HC_ADDRS: List[Tuple[str, int]] = [
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

def _word_to_bytes(word: int, byteorder: str) -> bytes:
    hi, lo = (word >> 8) & 0xFF, word & 0xFF
    return bytes([hi, lo]) if byteorder == "big" else bytes([lo, hi])

def _regs_to_bytes2(r0: int, r1: int, wordorder: str, byteorder: str) -> bytes:
    w0, w1 = _word_to_bytes(r0, byteorder), _word_to_bytes(r1, byteorder)
    return w0 + w1 if wordorder == "big" else w1 + w0

def _f32_from_regs(r0: int, r1: int, wordorder: str, byteorder: str) -> float:
    b = _regs_to_bytes2(r0, r1, wordorder, byteorder)
    return struct.unpack("!f", b)[0]

def _u32_from_regs(r0: int, r1: int, wordorder: str, byteorder: str) -> int:
    b = _regs_to_bytes2(r0, r1, wordorder, byteorder)
    return int.from_bytes(b, "big", signed=False)

def _i16_from_lowword(r0: int) -> int:
    # signed 16-bit
    val = r0 & 0xFFFF
    return val - 0x10000 if val & 0x8000 else val

def _looks_bad_float(x: float) -> bool:
    if math.isnan(x) or math.isinf(x):
        return True
    # Treat utterly tiny/huge magnitudes as suspicious for engineering values.
    return abs(x) < 1e-6 or abs(x) > 1e6

def _decode_mixed(r0: int, r1: int, wordorder: str, byteorder: str, scale: float) -> float:
    """Prefer float32; if high word is 0x0000/0xFFFF and float looks wrong, use int16*scale from the first word."""
    f = _f32_from_regs(r0, r1, wordorder, byteorder)
    hiword = r1 if wordorder == "little" else r0
    if (hiword == 0x0000 or hiword == 0xFFFF) and _looks_bad_float(f):
        return _i16_from_lowword(r0) * scale  # r0 is the first word (low word with --wordorder little)
    return f

def _write_values_to_file(path: str, wordorder: str, byteorder: str,
                          ai_scale: float, ao_scale: float, tk_scale: float):
    lines: List[str] = []
    # AI
    for i in range(AI_N):
        r0, r1 = REGS[2*i], REGS[2*i+1]
        v = _decode_mixed(r0, r1, wordorder, byteorder, ai_scale)
        lines.append(f"AI.{i:04d} | {v:.3f}")
    # AO
    base = 2000
    for i in range(AO_N):
        r0, r1 = REGS[base+2*i], REGS[base+2*i+1]
        v = _decode_mixed(r0, r1, wordorder, byteorder, ao_scale)
        lines.append(f"AO.{i:04d} | {v:.3f}")
    # HC
    for tag, addr in HC_ADDRS:
        r0, r1 = REGS[addr], REGS[addr+1]
        v = _u32_from_regs(r0, r1, wordorder, byteorder)
        lines.append(f"{tag} | {v}")
    # TK
    base = 4000
    for i in range(TK_N):
        r0, r1 = REGS[base+2*i], REGS[base+2*i+1]
        v = _decode_mixed(r0, r1, wordorder, byteorder, tk_scale)
        lines.append(f"TK.{i:04d} | {v:.3f}")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

class ModbusHandler(socketserver.StreamRequestHandler):
    outfile = "/tmp/moxa_values.txt"
    wordorder = "big"
    byteorder = "big"
    addr_base = 40001
    ai_scale = 1.0
    ao_scale = 1.0
    tk_scale = 1.0
    allowed = set()

    def handle(self):
        ip = self.client_address[0]
        if ModbusHandler.allowed and ip not in ModbusHandler.allowed:
            return
        while True:
            hdr = self.rfile.read(7)
            if not hdr or len(hdr) != 7:
                return
            tid, pid, length, uid = struct.unpack("!HHHB", hdr)
            pdu = self.rfile.read(length - 1)
            if len(pdu) < 1:
                return
            fc = pdu[0]

            if fc == 0x10:  # Write Multiple Registers
                if len(pdu) < 6:
                    self._exception(tid, uid, fc, 0x03); continue
                start, qty, bc = struct.unpack("!HHB", pdu[1:6])
                if len(pdu) != 6 + bc or bc != qty * 2:
                    self._exception(tid, uid, fc, 0x03); continue
                idx0 = start - self.addr_base if start >= self.addr_base else start
                vals = [ (pdu[6+2*i] << 8) | pdu[6+2*i+1] for i in range(qty) ]
                with REG_LOCK:
                    for i, w in enumerate(vals):
                        j = idx0 + i
                        if 0 <= j < len(REGS):
                            REGS[j] = w
                    global LAST_WRITE_TS
                    now = time.time()
                    if now - LAST_WRITE_TS >= 0.2:
                        _write_values_to_file(self.outfile, self.wordorder, self.byteorder,
                                              self.ai_scale, self.ao_scale, self.tk_scale)
                        LAST_WRITE_TS = now
                rsp_pdu = struct.pack("!BHH", 0x10, start, qty)
                self._send(tid, uid, rsp_pdu)

            elif fc == 0x06:  # Write Single Register
                if len(pdu) != 5:
                    self._exception(tid, uid, fc, 0x03); continue
                start, value = struct.unpack("!HH", pdu[1:5])
                idx = start - self.addr_base if start >= self.addr_base else start
                with REG_LOCK:
                    if 0 <= idx < len(REGS):
                        REGS[idx] = value
                        _write_values_to_file(self.outfile, self.wordorder, self.byteorder,
                                              self.ai_scale, self.ao_scale, self.tk_scale)
                self._send(tid, uid, pdu[:5])

            else:
                self._exception(tid, uid, fc, 0x01)

    def _send(self, tid: int, uid: int, pdu: bytes):
        mbap = struct.pack("!HHHB", tid, 0, len(pdu)+1, uid)
        self.wfile.write(mbap + pdu)

    def _exception(self, tid: int, uid: int, fc: int, code: int):
        self._send(tid, uid, bytes([fc | 0x80, code]))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--outfile", default="/tmp/moxa_values.txt")
    ap.add_argument("--wordorder", choices=["big","little"], default="little")
    ap.add_argument("--byteorder", choices=["big","little"], default="big")
    ap.add_argument("--addr-base", type=int, default=40001)
    ap.add_argument("--allow", action="append", default=[], help="Allowed client IPs; repeat flag")
    # scales applied only when falling back to int16
    ap.add_argument("--ai-scale", type=float, default=1.0)
    ap.add_argument("--ao-scale", type=float, default=1.0)
    ap.add_argument("--tk-scale", type=float, default=1.0)
    args = ap.parse_args()

    ModbusHandler.outfile = args.outfile
    ModbusHandler.wordorder = args.wordorder
    ModbusHandler.byteorder = args.byteorder
    ModbusHandler.addr_base = args.addr_base
    ModbusHandler.allowed = set(args.allow)
    ModbusHandler.ai_scale = args.ai_scale
    ModbusHandler.ao_scale = args.ao_scale
    ModbusHandler.tk_scale = args.tk_scale

    with socketserver.ThreadingTCPServer((args.listen, args.port), ModbusHandler) as srv:
        srv.allow_reuse_address = True
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()