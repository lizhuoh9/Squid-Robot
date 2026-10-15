"""RAM capture HTTP client/decoder. Never sends robot movement or OTA commands."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
import threading
import time
import urllib.error
import urllib.request
import zlib

HEADER = struct.Struct("<8sHHIIIII8f")
RECORD = struct.Struct("<III9fHBBB3x")
assert HEADER.size == 64 and RECORD.size == 56
FIELDS = ["record_index", "elapsed_s", "depth_age_ms", "depth_sequence", "depth_valid", "depth_fresh",
          "depth_cm", "raw_depth_cm", "depth_speed_cm_s", "depth_accel_cm_s2", "target_valid", "target_depth_cm",
          "proposal_desired_accel_cm_s2", "proposal_bias_cm_s2", "requested_duty", "battery_window_v",
          "pwm_commanded", "direction_commanded", "valve_mask_estimated", "forward_active", "balance_active", "output_saturated"]
STATUS_TIMEOUT_S = 8
DATA_TIMEOUT_S = 25
START_TIMEOUT_S = 12
GET_RETRIES = 3
DATA_RETRIES = 3


def parse_capture(blob: bytes):
    if len(blob) < HEADER.size:
        raise ValueError("Truncated capture header")
    magic, version, size, count, interval, checksum, flags, missed, *parameters = HEADER.unpack_from(blob)
    if magic != b"SQDCAP1\0" or version != 1 or size != RECORD.size or interval != 50000:
        raise ValueError("Unsupported capture schema")
    if not 1 <= count <= 2400 or len(blob) != HEADER.size + count * size:
        raise ValueError("Invalid record count or truncated payload")
    payload = blob[HEADER.size:]
    if zlib.crc32(payload) != checksum:
        raise ValueError("Capture CRC32 mismatch; refuse corrupted data")
    if flags != 0 or not all(math.isfinite(v) for v in parameters):
        raise ValueError("Invalid header metadata")
    rows, previous_time, previous_sequence = [], -1, -1
    for index in range(count):
        elapsed, age, sequence, depth, raw, speed, accel, target, desired, bias, duty, battery, mask, pwm, direction, bits = RECORD.unpack_from(payload, index * size)
        if elapsed <= previous_time or elapsed >= 120000000 or sequence < previous_sequence:
            raise ValueError("Non-monotonic record time/sequence")
        if direction > 3 or bits & ~63 or not math.isfinite(duty) or abs(duty) > 1.001:
            raise ValueError("Invalid command or flags")
        valid, fresh, target_valid = bool(bits & 1), bool(bits & 2), bool(bits & 4)
        if fresh and not valid:
            raise ValueError("Fresh depth flag without valid measurement")
        if valid and not all(math.isfinite(v) for v in (depth, raw, speed, accel)):
            raise ValueError("Valid depth flag with non-finite measurement")
        if target_valid and (not math.isfinite(target) or not 0 <= target <= 100):
            raise ValueError("Invalid target")
        rows.append(dict(zip(FIELDS, [index, elapsed / 1e6, age, sequence, int(valid), int(fresh),
                    depth if valid else "", raw if valid else "", speed if valid else "", accel if valid else "",
                    int(target_valid), target if target_valid else "", desired, bias, duty, battery,
                    pwm, direction, mask, int(bool(bits & 8)), int(bool(bits & 16)), int(bool(bits & 32))])))
        previous_time, previous_sequence = elapsed, sequence
    metadata = {"schema": version, "record_count": count, "nominal_interval_us": interval,
                "missed_slots": missed, "crc32": checksum, "payload_sha256": hashlib.sha256(payload).hexdigest(),
                "parameters": dict(zip(["KP", "KI", "KD", "G_SINK", "G_RISE", "A_REST_REF", "ALPHA", "DEPTH_REF"], parameters)),
                "actuator_feedback": False, "battery_measurement": "3-second maximum window",
                "note": "20Hz snapshots are not 20Hz independent depth measurements; PWM/mask are commands/estimates"}
    return rows, metadata


def save_capture(blob: bytes, prefix: Path):
    rows, metadata = parse_capture(blob)  # Validate everything before creating output files.
    paths = [prefix.with_suffix(suffix) for suffix in (".sqdcap", ".csv", ".json")]
    if any(p.exists() for p in paths):
        raise FileExistsError("Capture outputs already exist; choose a new prefix")
    prefix.parent.mkdir(parents=True, exist_ok=True)
    with paths[0].open("xb") as handle:
        handle.write(blob)
    with paths[1].open("x", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    with paths[2].open("x", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)
    return paths


def _retryable_error(exc):
    if isinstance(exc, urllib.error.HTTPError):
        return exc.code in (408, 429, 500, 502, 503, 504)
    return isinstance(exc, (urllib.error.URLError, TimeoutError, ConnectionError, OSError))


def request(base, route, payload=None, timeout=None, retries=None):
    if not base.startswith("http://"):
        raise ValueError("Use the robot's local http:// address")
    url = base.rstrip("/") + "/capture/" + route
    body = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"} if body else {})
    # Local robot traffic must not go through a system HTTP proxy.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    if timeout is None:
        timeout = DATA_TIMEOUT_S if route == "data" else STATUS_TIMEOUT_S if route == "status" else START_TIMEOUT_S
    if retries is None:
        retries = DATA_RETRIES if route == "data" else GET_RETRIES if route == "status" else 0
    for attempt in range(retries + 1):
        try:
            with opener.open(req, timeout=timeout) as response:
                content = response.read()
            return content if route == "data" else json.loads(content)
        except (urllib.error.URLError, TimeoutError, ConnectionError, OSError, urllib.error.HTTPError) as exc:
            if attempt >= retries or not _retryable_error(exc):
                raise
            delay = min(.75 * (2 ** attempt), 3)
            print(f"[연결 재시도] /capture/{route}: {attempt + 1}/{retries}회, {delay:.1f}초 후 다시 요청합니다.", flush=True)
            time.sleep(delay)


def _capture_already_exists(status):
    return status.get("recording", False) or status.get("state") == "recording" or int(status.get("count", 0)) > 0


def start_capture(base, duration):
    try:
        status = request(base, "status")
    except (urllib.error.URLError, TimeoutError, ConnectionError, OSError, urllib.error.HTTPError) as exc:
        raise RuntimeError("로봇 상태를 확인할 수 없어 기록 시작 요청을 보내지 않았습니다. PC와 로봇의 Wi-Fi망, 로봇 IP를 확인하세요.") from exc
    if _capture_already_exists(status):
        raise RuntimeError("로봇에 진행 중이거나 저장된 RAM 기록이 있습니다. 새 기록을 시작하지 않았습니다. 먼저 status를 확인하고 기존 데이터를 다운로드하세요.")
    if status.get("state") != "idle":
        raise RuntimeError(f"로봇 기록기 상태가 idle이 아닙니다(state={status.get('state')!r}). 새 기록을 시작하지 않았습니다.")

    payload = {"duration_s": duration}
    for attempt in range(2):
        try:
            return request(base, "start", payload, timeout=START_TIMEOUT_S)
        except (urllib.error.URLError, TimeoutError, ConnectionError, OSError, urllib.error.HTTPError) as exc:
            if isinstance(exc, urllib.error.HTTPError) and exc.code not in (408, 429, 500, 502, 503, 504):
                raise RuntimeError(f"로봇이 기록 시작을 거부했습니다(HTTP {exc.code}). status를 확인하세요.") from exc
            try:
                status = request(base, "status")
            except (urllib.error.URLError, TimeoutError, ConnectionError, OSError, urllib.error.HTTPError) as status_exc:
                raise RuntimeError("시작 응답과 로봇 상태를 확인할 수 없습니다. 재시작/clear를 누르지 말고 연결 복구 후 status로 기록 여부를 확인하세요.") from status_exc
            if _capture_already_exists(status):
                print("[시작 확인] HTTP 응답은 끊겼지만 로봇 상태에서 RAM 기록을 확인했습니다.", flush=True)
                return status
            if status.get("state") != "idle":
                raise RuntimeError(f"시작 요청 결과를 확정할 수 없습니다(state={status.get('state')!r}). 추가 시작 요청은 보내지 않았습니다.") from exc
            if attempt == 0:
                print("[시작 재시도] 로봇 상태가 idle임을 확인했습니다. 1초 후 한 번 더 요청합니다.", flush=True)
                time.sleep(1)
                continue
            raise RuntimeError("로봇이 계속 idle입니다. 기록은 시작되지 않았습니다. IP와 Wi-Fi 연결을 확인한 뒤 다시 시도하세요.") from exc
    raise RuntimeError("기록 시작을 확인하지 못했습니다.")


def wait_for_capture(base, duration, wait_surface):
    def notify_time_elapsed():
        print(f"[시간 종료] 설정한 {duration}초가 지났습니다. 로봇을 수면으로 회수하세요. "
              "실제 기록 완료 여부는 로봇 응답으로 확인합니다. 자동 부상하지 않습니다.", flush=True)

    # A separate timer can notify even while an HTTP request is timing out.
    # This timer measures PC time after the successful start response, not
    # a confirmed robot completion event.
    timer = threading.Timer(duration, notify_time_elapsed)
    timer.daemon = True
    deadline = time.monotonic() + duration + wait_surface
    timer.start()
    try:
        while time.monotonic() < deadline:
            try:
                status = request(base, "status")
                if status["state"] == "ready" and not status["recording"]:
                    print(f"[기록 완료 확인] 로봇 RAM에 {status.get('count', '?')}개가 기록되었습니다. "
                          "수면으로 회수하세요. PC 다운로드를 시도합니다.", flush=True)
                    return
            except (OSError, urllib.error.URLError):
                # WiFi may be unreachable underwater. RAM capture continues.
                pass
            time.sleep(2)
        raise TimeoutError("Surface the robot and use download later. Data remain in RAM; avoid power cycling.")
    finally:
        timer.cancel()
        timer.join()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://192.168.0.183")
    sub = parser.add_subparsers(dest="action", required=True)
    for name in ("status", "stop", "clear"):
        sub.add_parser(name)
    for name in ("start", "record"):
        item = sub.add_parser(name)
        item.add_argument("--duration", type=int, default=60)
        if name == "record":
            item.add_argument("--wait-surface", type=int, default=120)
            item.add_argument("--output-prefix", type=Path)
    item = sub.add_parser("download")
    item.add_argument("--output-prefix", type=Path)
    item = sub.add_parser("decode")
    item.add_argument("input", type=Path)
    item.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()
    if args.action in ("start", "record") and not 1 <= args.duration <= 120:
        parser.error("duration must be 1..120 seconds")
    if args.action == "record" and not 0 <= args.wait_surface <= 600:
        parser.error("wait-surface must be 0..600 seconds")
    if args.action == "decode":
        for path in save_capture(args.input.read_bytes(), args.output_prefix):
            print(path)
        return
    if args.action in ("status", "stop", "clear"):
        # stop/clear affect recorder data ONLY. No l0/s/movement commands here.
        print(json.dumps(request(args.url, args.action, {} if args.action != "status" else None), indent=2))
        return
    if args.action in ("start", "record"):
        print(json.dumps(start_capture(args.url, args.duration), indent=2))
        if args.action == "start":
            return
        print("RAM recording started. Control/surface/recover robot manually; no motion is commanded by this tool.", flush=True)
        wait_for_capture(args.url, args.duration, args.wait_surface)
    prefix = args.output_prefix or (Path(__file__).resolve().parents[1] / "data" /
              ("capture-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")))
    try:
        capture = request(args.url, "data")
    except (urllib.error.URLError, TimeoutError, ConnectionError, OSError, urllib.error.HTTPError) as exc:
        raise RuntimeError("다운로드에 실패했습니다. 로봇 RAM 기록은 지우지 않았습니다. 연결 복구 후 `download`를 다시 실행하세요.") from exc
    for path in save_capture(capture, prefix):
        print(path)
    print("Capture retained on robot. Use clear explicitly only after verifying saved files.")


if __name__ == "__main__":
    main()
