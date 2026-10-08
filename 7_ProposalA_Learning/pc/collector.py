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


def request(base, route, payload=None):
    if not base.startswith("http://"):
        raise ValueError("Use the robot's local http:// address")
    url = base.rstrip("/") + "/capture/" + route
    body = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"} if body else {})
    # Local robot traffic must not go through a system HTTP proxy.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=5) as response:
        content = response.read()
    return content if route == "data" else json.loads(content)


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
        print(json.dumps(request(args.url, "start", {"duration_s": args.duration}), indent=2))
        if args.action == "start":
            return
        print("RAM recording started. Control/surface/recover robot manually; no motion is commanded by this tool.")
        deadline = time.monotonic() + args.duration + args.wait_surface
        while time.monotonic() < deadline:
            try:
                status = request(args.url, "status")
                if status["state"] == "ready" and not status["recording"]:
                    break
            except (OSError, urllib.error.URLError):
                # WiFi may be unreachable underwater. Timed RAM capture continues.
                pass
            time.sleep(2)
        else:
            raise TimeoutError("Surface the robot and use download later. Data remain in RAM; avoid power cycling.")
    prefix = args.output_prefix or (Path(__file__).resolve().parents[1] / "data" /
              ("capture-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")))
    for path in save_capture(request(args.url, "data"), prefix):
        print(path)
    print("Capture retained on robot. Use clear explicitly only after verifying saved files.")


if __name__ == "__main__":
    main()
