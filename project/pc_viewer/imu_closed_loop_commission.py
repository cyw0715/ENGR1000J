import json
import threading
import time
import urllib.request

from landing_control import (
    ImuLegController,
    ImuDeltaCommandGenerator,
    transform_imu_to_platform,
)

HOST = "192.168.137.244"


def get(path, timeout=1.0):
    return json.load(urllib.request.urlopen(f"http://{HOST}{path}", timeout=timeout))


def post(payload, timeout=0.6):
    req = urllib.request.Request(
        f"http://{HOST}/cmd",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    return json.load(urllib.request.urlopen(req, timeout=timeout))


def platform_attitude():
    imu = get("/imu")
    roll, pitch = transform_imu_to_platform(
        float(imu["roll"]),
        float(imu["pitch"]),
        "swap_neg",
        roll_zero_deg=-180.0,
        pitch_zero_deg=0.0,
    )
    return roll, pitch, imu


class CommandPump(threading.Thread):
    """Own the Mega sequence and keep heartbeats independent of sensor reads."""

    def __init__(self):
        super().__init__(daemon=True)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_delta = None
        self.sent_delta = None
        self.seq = 0
        self.error = None
        self.consecutive_errors = 0
        self.last_ack = None

    def set_delta(self, delta):
        rounded = {k: int(round(delta[k])) for k in ("fl", "fr", "rl", "rr")}
        with self.lock:
            self.latest_delta = rounded

    def _next_seq(self):
        self.seq += 1
        return self.seq

    def run(self):
        next_heartbeat = 0.0
        while not self.stop_event.is_set():
            now = time.monotonic()
            try:
                if now >= next_heartbeat:
                    seq = self._next_seq()
                    self.last_ack = post({"imu_heartbeat": True, "seq": seq})
                    next_heartbeat = time.monotonic() + 0.15
                with self.lock:
                    pending = None if self.latest_delta == self.sent_delta else dict(self.latest_delta or {})
                if pending:
                    seq = self._next_seq()
                    payload = {"imu_set_delta": True, "seq": seq, **pending}
                    self.last_ack = post(payload)
                    with self.lock:
                        self.sent_delta = pending
                self.consecutive_errors = 0
            except Exception as exc:
                self.consecutive_errors += 1
                if self.consecutive_errors >= 3:
                    self.error = exc
                    self.stop_event.set()
                    return
                next_heartbeat = time.monotonic()
                self.stop_event.wait(0.05)
                continue
            self.stop_event.wait(0.02)

    def stop(self):
        self.stop_event.set()
        self.join(timeout=1.0)


def run():
    pre = get("/health")["mega_imu"]
    if pre["mode"] == "FAULT" and pre.get("fault_reason") in ("heartbeat_timeout", "session_timeout"):
        print("RECOVER_STOP", post({"imu_stop": True}))
        time.sleep(0.4)
        pre = get("/health")["mega_imu"]
    if pre["mode"] != "DISARMED" or pre.get("fault_reason") not in (None, "", "none"):
        raise RuntimeError(f"unsafe pre-state {pre}")

    ctrl = ImuLegController()
    gen = ImuDeltaCommandGenerator()
    initial_roll, initial_pitch, _ = platform_attitude()
    print("INITIAL_PLATFORM", initial_roll, initial_pitch)
    print("ARM", post({"imu_arm": True}))

    pump = CommandPump()
    pump.start()
    last_report = 0.0
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            if pump.error is not None:
                raise RuntimeError(f"command pump failed: {pump.error!r}")
            now = time.monotonic()
            roll, pitch, _ = platform_attitude()
            gen.update_imu_time(now)
            result = ctrl.compute(roll, pitch, now=now)
            delta = gen.compute_deltas(
                result["adjustments_mm"], now, hold=result["within_target"]
            )
            if delta is not None:
                pump.set_delta(delta)

            state = get("/health")["mega_imu"]
            if state["mode"] in ("FAULT", "ESTOP"):
                raise RuntimeError(f"Mega fault {state}")
            if now - last_report >= 1.0:
                print("LOOP", round(roll, 3), round(pitch, 3), state)
                last_report = now
            time.sleep(0.05)

        final_roll, final_pitch, _ = platform_attitude()
        print("FINAL_PLATFORM", final_roll, final_pitch)
        print(
            "IMPROVEMENT",
            abs(initial_roll) - abs(final_roll),
            abs(initial_pitch) - abs(final_pitch),
        )
        print("FINAL_STATE", get("/health")["mega_imu"])
    finally:
        pump.stop()
        try:
            print("STOP", post({"imu_stop": True}))
        except Exception as exc:
            print("STOP_ERROR", repr(exc))
        time.sleep(0.4)
        print("POST_STOP", get("/health")["mega_imu"])


if __name__ == "__main__":
    run()
