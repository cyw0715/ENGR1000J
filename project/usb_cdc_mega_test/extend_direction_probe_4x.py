import re, time, sys
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
from p4_mega_bridge_client import Bridge, CMD_MEGA_TX

b = Bridge('COM5')
all_results = {}
try:
    b.hello()
    for actuator in ('A1', 'A2', 'A3', 'A4'):
        deltas = []
        for pulse in range(4):
            b.send(CMD_MEGA_TX, b'ARM_JOG IDENTIFY\n')
            armed = False; end = time.monotonic() + 2
            while time.monotonic() < end:
                _, p = b.receive(end - time.monotonic())
                if 'OK JOG_ARMED' in p.decode('utf-8', 'replace'):
                    armed = True; break
            if not armed: raise RuntimeError(f'{actuator} pulse {pulse}: arm failed')
            b.send(CMD_MEGA_TX, f'JOG {actuator} EXTEND\n'.encode())
            result = None; end = time.monotonic() + 3
            while time.monotonic() < end:
                _, p = b.receive(end - time.monotonic())
                text = p.decode('utf-8', 'replace').strip()
                m = re.search(rf'JOG_RESULT actuator={actuator} polarity=EXTEND before_mm=(\d+) after_mm=(\d+) delta_mm=([-\d]+) outputs=ALL_LOW', text)
                if m:
                    result = tuple(map(int, m.groups())); break
            if result is None: raise RuntimeError(f'{actuator} pulse {pulse}: no result')
            deltas.append(result)
            print(actuator, pulse + 1, result)
            time.sleep(.3)
        all_results[actuator] = deltas
    print('EXTEND_4X_RESULTS', all_results)
finally:
    try: b.send(CMD_MEGA_TX, b'STOP\n')
    except Exception: pass
    b.close()
