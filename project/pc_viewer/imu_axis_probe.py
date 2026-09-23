import json, math, statistics, time, urllib.request

HOST = "192.168.137.244"


def get(path):
    return json.load(urllib.request.urlopen(f"http://{HOST}{path}", timeout=3))


def post(payload):
    req = urllib.request.Request(
        f"http://{HOST}/cmd",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    return json.load(urllib.request.urlopen(req, timeout=3))


def mean_imu(duration=0.8):
    rows = []
    end = time.monotonic() + duration
    while time.monotonic() < end:
        rows.append(get("/imu"))
        time.sleep(0.04)
    result = {}
    for key in ("yaw", "pitch", "roll"):
        # The short probe cannot cross more than one wrap; unwrap around first sample.
        first = float(rows[0][key])
        values = [first + ((float(r[key]) - first + 180) % 360 - 180) for r in rows]
        result[key] = statistics.mean(values)
    for axis in ("x", "y", "z"):
        result[f"acc_{axis}"] = statistics.mean(float(r["raw_accel_mg"][axis]) for r in rows)
    result["count"] = len(rows)
    return result


def angle_delta(after, before):
    return (after - before + 180) % 360 - 180


def probe(name, targets):
    pre_state = get("/health")["mega_imu"]
    if pre_state["mode"] != "DISARMED" or pre_state["fault_count"]:
        raise RuntimeError(f"unsafe pre-state: {pre_state}")
    before = mean_imu()
    print("BEFORE", json.dumps(before))
    print("ARM", post({"imu_arm": True}))
    seq = 1
    print("HB", post({"imu_heartbeat": True, "seq": seq}))
    seq += 1
    payload = {"imu_set_delta": True, "seq": seq, **targets}
    print("TARGET", post(payload))
    deadline = time.monotonic() + 6.0
    last_hb = 0.0
    final_state = None
    settled_since = None
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_hb >= 0.30:
                seq += 1
                post({"imu_heartbeat": True, "seq": seq})
                last_hb = now
            state = get("/health")["mega_imu"]
            print("STATE", state)
            if state["mode"] == "FAULT" or state["estop"]:
                raise RuntimeError(f"Mega fault: {state}")
            target_ok = all(state[f"target_{leg}"] == value for leg, value in targets.items())
            if target_ok and state["mode"] == "ARMED":
                if settled_since is None:
                    settled_since = now
                elif now - settled_since >= 0.6:
                    final_state = state
                    break
            else:
                settled_since = None
            time.sleep(0.12)
        if final_state is None:
            raise RuntimeError(f"probe did not settle, last={state}")
        after = mean_imu()
        delta = {
            "yaw": angle_delta(after["yaw"], before["yaw"]),
            "pitch": angle_delta(after["pitch"], before["pitch"]),
            "roll": angle_delta(after["roll"], before["roll"]),
            "acc_x": after["acc_x"] - before["acc_x"],
            "acc_y": after["acc_y"] - before["acc_y"],
            "acc_z": after["acc_z"] - before["acc_z"],
        }
        print("AFTER", json.dumps(after))
        print("DELTA", json.dumps(delta))
        print("PROBE_RESULT", json.dumps({"name": name, "before": before, "after": after, "delta": delta, "mega": final_state}))
    finally:
        try:
            print("STOP", post({"imu_stop": True}))
        except Exception as exc:
            print("STOP_ERROR", repr(exc))
        time.sleep(0.4)
        print("POST_STOP", get("/health")["mega_imu"])


if __name__ == "__main__":
    probe("physical_roll_negative", {"fl": 2, "fr": -2, "rl": 2, "rr": -2})
