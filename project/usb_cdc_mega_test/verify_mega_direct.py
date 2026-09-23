import serial
import time

with serial.Serial("COM6", 115200, timeout=0.25, write_timeout=1, rtscts=False, dsrdtr=False) as port:
    port.dtr = False
    port.rts = False
    time.sleep(1.5)  # allow normal Arduino reset-on-open behavior to finish
    port.reset_input_buffer()
    port.write(b"PING\n")
    port.flush()
    deadline = time.monotonic() + 3
    received = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(128)
        if chunk:
            received.extend(chunk)
            if b"PONG" in received:
                break
    print("MEGA_DIRECT_RX=", bytes(received).decode("utf-8", "replace").strip())
    if b"PONG" not in received:
        raise SystemExit("Expected PONG was not received")
    print("MEGA_DIRECT_PING_PONG_OK")
