#!/usr/bin/env python3
# Minimal Modbus/TCP server for FC16/FC6 writes. No external deps.
# Maps Siemens 1-based 4xxxx addresses to FID zero-based offsets:
#   40001..41320 -> AI (0..1319 words, 660 float32)
#   42001..42088 -> AO (2000..2087 words, 44 float32)
#   43001..43102 -> HC (3000..3101 words, 51 uint32)
#   44001..44104 -> TK (4000..4103 words, 52 float32)
# Writes "channel | value" to the outfile on each update.

import argparse, socketserver, struct, threading, time
from typing import List, Tuple

AI_N = 660     # float32, pairs at 0..1319
AO_N = 44      # float32, pairs at 2000..2087
HC_N = 51      # uint32,  pairs at 3000..3101 (explicit map below)
TK_N = 52      # float32, pairs at 4000..4103

# Pre-size register file to 0..4103 (words). 4104 words total.
REGS = [0] * 4104
REG_LOCK = threading.Lock()
LAST_WRITE_TS = 0.0

# HC address map (FID Appendix A)
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

def _write_values_to_file(path: str, wordorder: str, byteorder: str):
    lines: List[str] = []
    # AI
    for i in range(AI_N):
        r0, r1 = REGS[2*i], REGS[2*i+1]
        v = _f32_from_regs(r0, r1, wordorder, byteorder)
        lines.append(f"AI.{i:04d} | {v:.3f}")
    # AO
    base = 2000
    for i in range(AO_N):
        r0, r1 = REGS[base+2*i], REGS[base+2*i+1]
        v = _f32_from_regs(r0, r1, wordorder, byteorder)
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
        v = _f32_from_regs(r0, r1, wordorder, byteorder)
        lines.append(f"TK.{i:04d} | {v:.3f}")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

class ModbusHandler(socketserver.StreamRequestHandler):
    # globals configured in main()
    outfile = "/tmp/moxa_values.txt"
    wordorder = "big"
    byteorder = "big"
    addr_base = 40001  # subtract this from incoming addresses

    def handle(self):
        while True:
            hdr = self.rfile.read(7)
            if not hdr:
                return
            if len(hdr) != 7:
                return
            tid, pid, length, uid = struct.unpack("!HHHB", hdr)
            # length includes 1 byte of uid + PDU
            pdu = self.rfile.read(length - 1)
            if len(pdu) < 1:
                return

            fc = pdu[0]
            if fc == 0x10:  # Write Multiple Registers
                if len(pdu) < 6:
                    self._exception(tid, uid, fc, 0x03)
                    continue
                start, qty, bc = struct.unpack("!HHB", pdu[1:6])
                if len(pdu) != 6 + bc or bc != qty * 2:
                    self._exception(tid, uid, fc, 0x03)
                    continue
                # Map to FID 0-based space
                idx0 = start - self.addr_base if start >= self.addr_base else start
                vals = [ (pdu[6+2*i] << 8) | pdu[6+2*i+1] for i in range(qty) ]
                with REG_LOCK:
                    for i, w in enumerate(vals):
                        j = idx0 + i
                        if 0 <= j < len(REGS):
                            REGS[j] = w
                    # throttle file writes to 0.2s
                    global LAST_WRITE_TS
                    now = time.time()
                    if now - LAST_WRITE_TS >= 0.2:
                        _write_values_to_file(self.outfile, self.wordorder, self.byteorder)
                        LAST_WRITE_TS = now
                # normal response: echo start + quantity
                rsp_pdu = struct.pack("!BHH", 0x10, start, qty)
                self._send(tid, uid, rsp_pdu)

            elif fc == 0x06:  # Write Single Register
                if len(pdu) != 5:
                    self._exception(tid, uid, fc, 0x03)
                    continue
                start, value = struct.unpack("!HH", pdu[1:5])
                idx = start - self.addr_base if start >= self.addr_base else start
                with REG_LOCK:
                    if 0 <= idx < len(REGS):
                        REGS[idx] = value
                        _write_values_to_file(self.outfile, self.wordorder, self.byteorder)
                self._send(tid, uid, pdu[:5])

            else:
                self._exception(tid, uid, fc, 0x01)  # illegal function

    def _send(self, tid: int, uid: int, pdu: bytes):
        mbap = struct.pack("!HHHB", tid, 0, len(pdu) + 1, uid)
        self.wfile.write(mbap + pdu)

    def _exception(self, tid: int, uid: int, fc: int, code: int):
        pdu = struct.pack("!BB", fc | 0x80, code)
        self._send(tid, uid, pdu)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--outfile", default="/tmp/moxa_values.txt")
    ap.add_argument("--wordorder", choices=["big","little"], default="big")
    ap.add_argument("--byteorder", choices=["big","little"], default="big")
    ap.add_argument("--addr-base", type=int, default=40001,
                    help="If incoming uses 4xxxx, set 40001. If 0-based, set 0. If 40000, set 40000.")
    args = ap.parse_args()

    ModbusHandler.outfile = args.outfile
    ModbusHandler.wordorder = args.wordorder
    ModbusHandler.byteorder = args.byteorder
    ModbusHandler.addr_base = args.addr_base

    with socketserver.ThreadingTCPServer((args.listen, args.port), ModbusHandler) as srv:
        srv.allow_reuse_address = True
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    main()