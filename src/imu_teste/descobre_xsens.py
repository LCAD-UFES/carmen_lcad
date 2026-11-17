#!/usr/bin/env python3
"""Sonda um Xsens MT/MTi na porta serial: descobre baudrate, DeviceID e ProductCode."""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUDS = [115200, 460800, 230400, 921600, 57600, 38400, 19200, 9600]

def msg(mid, data=b""):
    body = bytes([0xFF, mid, len(data)]) + data
    cs = (-sum(body)) & 0xFF
    return b"\xFA" + body + bytes([cs])

GOTO_CONFIG = msg(0x30)
REQ_DID     = msg(0x00)
REQ_PROD    = msg(0x1C)

def parse(buf):
    """Devolve lista de (mid, data) achados no buffer."""
    out, i = [], 0
    while i < len(buf) - 4:
        if buf[i] == 0xFA and buf[i+1] == 0xFF:
            mid, ln = buf[i+2], buf[i+3]
            if ln != 0xFF and i + 5 + ln <= len(buf):
                out.append((mid, buf[i+4:i+4+ln]))
                i += 5 + ln
                continue
        i += 1
    return out

for baud in BAUDS:
    try:
        s = serial.Serial(PORT, baud, timeout=0.3)
    except Exception as e:
        print(f"  !! nao abriu {PORT}: {e}"); sys.exit(1)
    s.reset_input_buffer()

    # 1) so escuta: se ja estiver em measurement, vem MTData (0x32)
    time.sleep(0.4)
    passive = s.read(4096)

    # 2) forca config e pergunta identidade
    for _ in range(3):
        s.write(GOTO_CONFIG); s.flush(); time.sleep(0.1)
    s.read(4096)
    s.write(REQ_DID); s.flush(); time.sleep(0.2)
    did = s.read(256)
    s.write(REQ_PROD); s.flush(); time.sleep(0.2)
    prod = s.read(256)
    s.close()

    pkts = parse(passive) + parse(did) + parse(prod)
    if not pkts:
        print(f"[{baud:>6}] nada (passivo: {len(passive)} bytes brutos)")
        continue

    print(f"[{baud:>6}] *** RESPONDEU *** {len(pkts)} pacote(s) Xsens")
    for mid, d in pkts:
        if mid == 0x01:
            print(f"           DeviceID   : {d.hex().upper()}")
        elif mid == 0x1D:
            print(f"           ProductCode: {d.decode('ascii', 'replace').strip()}")
        elif mid == 0x31:
            print(f"           GoToConfigAck (estava em measurement/config)")
        elif mid == 0x32:
            print(f"           MTData streaming ({len(d)} bytes de payload)")
        else:
            print(f"           MID 0x{mid:02X} len={len(d)} {d.hex().upper()[:40]}")
    print(f"           >>> BAUDRATE = {baud}")
    break
else:
    print("\nNenhuma resposta em nenhum baudrate. Ou nao e um Xsens, ou esta sem alimentacao/cabo.")
