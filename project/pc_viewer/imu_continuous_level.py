import time
import urllib.error

from imu_closed_loop_commission import (
    CommandPump,
    ImuDeltaCommandGenerator,
    ImuLegController,
    get,
    platform_attitude,
    post,
)

MAX_SEGMENTS = 1
SEGMENT_TIMEOUT_S = 90.0
TARGET_DEG = 1.0


def retry_get(path, attempts=3):
    last = None
    for _ in range(attempts):
        try:
            return get(path)
        except Exception as exc:
            last = exc
            time.sleep(0.15)
    raise last


def arm_confirmed():
    try:
        print('ARM', post({'imu_arm': True}))
    except Exception as exc:
        print('ARM_RESPONSE_LOST', repr(exc))
    for _ in range(5):
        state = retry_get('/health')['mega_imu']
        if state['mode'] in ('ARMED', 'ACTIVE'):
            return state
        if state['mode'] in ('FAULT', 'ESTOP'):
            raise RuntimeError(f"ARM fault: {state}")
        time.sleep(.15)
    raise RuntimeError(f"ARM not confirmed: {state}")


def stop_confirmed():
    for _ in range(4):
        try: print('STOP', post({'imu_stop': True}))
        except Exception as exc: print('STOP_RESPONSE_LOST', repr(exc))
        time.sleep(.2)
        state = retry_get('/health')['mega_imu']
        if state['mode'] == 'DISARMED': return state
    raise RuntimeError(f"STOP not confirmed: {state}")


def run_segment(index: int):
    ctrl = ImuLegController()
    gen = ImuDeltaCommandGenerator()
    start_roll, start_pitch, _ = platform_attitude()
    print(f"SEGMENT {index} START roll={start_roll:+.3f} pitch={start_pitch:+.3f}")
    if abs(start_roll) <= TARGET_DEG and abs(start_pitch) <= TARGET_DEG:
        return True, (start_roll, start_pitch)

    state = retry_get('/health')['mega_imu']
    if state['mode'] == 'FAULT' and state.get('fault_reason') == 'heartbeat_timeout':
        state = stop_confirmed()
    if state['mode'] != 'DISARMED' or state.get('fault_reason') not in ('none', '', None):
        raise RuntimeError(f"unsafe segment pre-state: {state}")
    arm_confirmed()
    pump = CommandPump(); pump.start()
    stable_since = None; saturated_since = None; last_report = 0.0
    try:
        deadline = time.monotonic() + SEGMENT_TIMEOUT_S
        while time.monotonic() < deadline:
            if pump.error is not None:
                raise RuntimeError(f"pump failed: {pump.error!r}")
            now = time.monotonic()
            roll, pitch, _ = platform_attitude()
            gen.update_imu_time(now)
            result = ctrl.compute(roll, pitch, now=now)
            delta = gen.compute_deltas(result['adjustments_mm'], now, hold=result['within_target'])
            if delta is not None: pump.set_delta(delta)
            state = retry_get('/health')['mega_imu']
            if state['mode'] in ('FAULT', 'ESTOP'):
                raise RuntimeError(f"Mega fault: {state}")

            within = abs(roll) <= TARGET_DEG and abs(pitch) <= TARGET_DEG
            if within:
                stable_since = stable_since or now
                if now - stable_since >= 1.0:
                    print(f"SEGMENT {index} TARGET roll={roll:+.3f} pitch={pitch:+.3f}")
                    return True, (roll, pitch)
            else:
                stable_since = None

            # With the full per-leg 50..400 mm range enabled, remain in one ARM
            # session. Mega is the authority for absolute target boundaries.
            targets = [state[f'target_{x}'] for x in ('fl', 'fr', 'rl', 'rr')]
            if now - last_report >= 1.0:
                print(f"SEGMENT {index} LOOP roll={roll:+.3f} pitch={pitch:+.3f} mode={state['mode']} target={targets}")
                last_report = now
            time.sleep(.05)
        roll, pitch, _ = platform_attitude()
        print(f"SEGMENT {index} TIMEOUT roll={roll:+.3f} pitch={pitch:+.3f}")
        return False, (roll, pitch)
    finally:
        pump.stop()
        state = stop_confirmed()
        print('POST_STOP', state)


def main():
    initial = platform_attitude()[:2]
    for index in range(1, MAX_SEGMENTS + 1):
        done, attitude = run_segment(index)
        if done:
            print('CONTINUOUS_LEVEL_SUCCESS', {'initial': initial, 'final': attitude, 'segments': index})
            return
    final = platform_attitude()[:2]
    print('CONTINUOUS_LEVEL_LIMIT_REACHED', {'initial': initial, 'final': final, 'segments': MAX_SEGMENTS})


if __name__ == '__main__':
    main()
