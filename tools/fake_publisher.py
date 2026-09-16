#!/usr/bin/env python3
"""Emit synthetic FiducialFrame datagrams to Nexus. No camera, no drone, no Gazebo.

Drives the consumer's precision-landing state machine without hardware. This is how
the wire format is tested before a camera is available, and it serves as a
regression harness for the consumer side.

It can also fly a repeatable, analytically known descent profile, so the consumer's
NED conversion can be checked against arithmetic rather than against a hover.

Usage
-----
  # 10 Hz heartbeats plus a tag descending 10 m -> 0.3 m over 30 s
  ./fake_publisher.py --profile descent --duration 30

  # a static tag 5 m out, for wiring up a consumer
  ./fake_publisher.py --profile static --range 5.0

  # two cameras, to exercise Nexus's demux and candidate selection
  ./fake_publisher.py --profile descent --camera-id down --camera-id forward

  # heartbeats only: proves liveness handling and the abort path when they stop
  ./fake_publisher.py --profile heartbeat --duration 10

  # present a different boot id, to exercise the clock-domain mismatch path
  ./fake_publisher.py --profile descent --clock-domain boot:deadbeef-0000

Requires the generated fiducial_pb2. Build the project first, then:
  protoc --python_out=tools protos/fiducial.proto
or just run this script — it generates the binding into a temp dir on the fly.
"""

import argparse
import math
import os
import random
import socket
import subprocess
import sys
import tempfile
import time

# ---------------------------------------------------------------------------
# Locate or generate fiducial_pb2
# ---------------------------------------------------------------------------

# protoc on many boards (Bookworm, JetPack 6) predates the installed python
# protobuf runtime, and the C++ descriptor implementation refuses the older
# generated code outright. The pure-python implementation accepts it and is more
# than fast enough for a test harness. Must be set BEFORE protobuf is imported.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

def load_proto_module():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)

    # Already generated next to this script?
    sys.path.insert(0, here)
    try:
        import fiducial_pb2  # noqa: F401
        return fiducial_pb2
    except ImportError:
        pass

    proto = os.path.join(repo, "protos", "fiducial.proto")
    if not os.path.exists(proto):
        sys.exit(f"cannot find {proto}")

    out = tempfile.mkdtemp(prefix="fiducial_pb_")
    try:
        subprocess.run(
            ["protoc", f"--python_out={out}", f"--proto_path={os.path.dirname(proto)}", proto],
            check=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        sys.exit(f"protoc failed ({e}). Install protobuf-compiler, or run:\n"
                 f"  protoc --python_out=tools --proto_path=protos protos/fiducial.proto")
    sys.path.insert(0, out)
    import fiducial_pb2  # noqa: E402
    return fiducial_pb2


pb = load_proto_module()

# ---------------------------------------------------------------------------

def monotonic_us():
    return int(time.clock_gettime(time.CLOCK_MONOTONIC) * 1e6)


def realtime_us():
    return int(time.time() * 1e6)


def read_boot_id():
    try:
        with open("/proc/sys/kernel/random/boot_id") as f:
            return "boot:" + f.read().strip()
    except OSError:
        return "rand:fake_publisher"


def rotation_z(yaw_rad):
    """Row-major tag->camera rotation for a tag facing the camera, rotated by yaw."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return [c, -s, 0.0,
            s,  c, 0.0,
            0.0, 0.0, 1.0]


# ---------------------------------------------------------------------------
# Trajectory profiles: t (seconds since start) -> list of detections, or None
# for "nothing in view this frame" (which must produce a heartbeat instead).
# ---------------------------------------------------------------------------

def profile_static(t, args):
    return [dict(tag_id=args.tag_id, range_m=args.range, x=0.0, y=0.0)]


def profile_descent(t, args):
    """Range decays 10 m -> 0.3 m across --duration, with a lateral wobble.

    The lateral movement matters: a sign error in the consumer's
    optical->body->NED transform is invisible against a perfectly centred tag
    and obvious against one that moves.
    """
    frac = min(1.0, t / max(args.duration, 1e-6))
    rng = 10.0 * (1.0 - frac) + 0.3 * frac
    wobble = 0.15 * math.sin(2.0 * math.pi * 0.25 * t)
    return [dict(tag_id=args.tag_id, range_m=rng, x=wobble, y=wobble * 0.5)]


def profile_nested(t, args):
    """Large approach tag plus a small co-located one; the large one leaves
    the field of view below ~1.5 m. Exercises the consumer's selection logic."""
    frac = min(1.0, t / max(args.duration, 1e-6))
    rng = 10.0 * (1.0 - frac) + 0.3 * frac
    dets = []
    if rng > 1.5:
        dets.append(dict(tag_id=0, range_m=rng, x=0.0, y=0.0, size_m=0.50))
    if rng < 4.0:
        dets.append(dict(tag_id=1, range_m=rng, x=0.02, y=0.02, size_m=0.12))
    return dets


def profile_intermittent(t, args):
    """Tag visible in 2 s bursts separated by 1 s gaps. The gaps must appear
    as heartbeats rather than silence."""
    if (t % 3.0) < 2.0:
        return profile_descent(t, args)
    return []


def profile_heartbeat(t, args):
    return []


PROFILES = {
    "static": profile_static,
    "descent": profile_descent,
    "nested": profile_nested,
    "intermittent": profile_intermittent,
    "heartbeat": profile_heartbeat,
}

# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Publish synthetic FiducialFrame datagrams to a Nexus UdpTagSource.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5602)
    ap.add_argument("--profile", choices=sorted(PROFILES), default="descent")
    ap.add_argument("--camera-id", action="append", default=None,
                    help="repeatable; each camera gets its own frame_id sequence")
    ap.add_argument("--fps", type=float, default=30.0, help="frame publish rate")
    ap.add_argument("--heartbeat-hz", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=30.0,
                    help="seconds; also the descent profile's time base. 0 = forever")
    ap.add_argument("--tag-id", type=int, default=0)
    ap.add_argument("--family", default="apriltag_36h11")
    ap.add_argument("--size-m", type=float, default=0.30)
    ap.add_argument("--range", type=float, default=5.0, help="static profile range")
    ap.add_argument("--reproj-px", type=float, default=0.8,
                    help="base reprojection error; grows with range, as the real one does")
    ap.add_argument("--noise-m", type=float, default=0.01, help="gaussian noise on translation")
    ap.add_argument("--clock-domain", default=None,
                    help="override the boot id, to exercise the cross-SoC mismatch path")
    ap.add_argument("--frame-width", type=int, default=1280)
    ap.add_argument("--frame-height", type=int, default=720)
    ap.add_argument("--drop-rate", type=float, default=0.0,
                    help="fraction of datagrams to silently drop, to test gap detection "
                         "via frame_id")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    cams = args.camera_id or ["down"]
    clock_domain = args.clock_domain or read_boot_id()
    profile = PROFILES[args.profile]

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)

    print(f"fake_publisher -> {args.host}:{args.port}")
    print(f"  profile={args.profile} cameras={cams} fps={args.fps} "
          f"heartbeat={args.heartbeat_hz}Hz")
    print(f"  clock_domain={clock_domain}")
    print("  Ctrl-C to stop.\n")

    frame_ids = {c: 0 for c in cams}
    last_publish = {c: 0.0 for c in cams}
    hb_period = 1.0 / args.heartbeat_hz if args.heartbeat_hz > 0 else None
    period = 1.0 / args.fps
    t_start = time.monotonic()
    sent = dropped = 0

    try:
        while True:
            now = time.monotonic()
            t = now - t_start
            if args.duration > 0 and t > args.duration:
                break

            for cam in cams:
                specs = profile(t, args)

                msg = pb.FiducialFrame()
                cap_us = monotonic_us()
                msg.capture_timestamp_us = cap_us
                msg.publish_timestamp_us = cap_us + random.randint(2000, 8000)
                msg.capture_realtime_us = realtime_us()
                msg.clock_domain = clock_domain
                frame_ids[cam] += 1
                msg.frame_id = frame_ids[cam]
                msg.frame_width = args.frame_width
                msg.frame_height = args.frame_height
                msg.camera_id = cam

                if specs:
                    msg.heartbeat = False
                    msg.detect_ms = random.uniform(4.0, 12.0)
                    for s in specs:
                        d = msg.detections.add()
                        d.tag_id = s["tag_id"]
                        d.family = args.family
                        d.size_m = s.get("size_m", args.size_m)
                        rng = s["range_m"]
                        n = lambda: random.gauss(0.0, args.noise_m)
                        # Camera optical frame: +x right, +y down, +z forward.
                        d.t_tag_wrt_cam.extend([s["x"] + n(), s["y"] + n(), rng + n()])
                        d.r_tag_to_cam.extend(rotation_z(0.1 * math.sin(t)))
                        # Real reprojection error grows with range and obliquity;
                        # a flat constant would let a Nexus ranking bug hide.
                        d.reproj_error_px = args.reproj_px * (1.0 + rng / 10.0)
                        d.camera_id = cam
                        d.loc_type = 0
                    last_publish[cam] = now
                else:
                    # Nothing in view. Heartbeat on schedule, not every frame.
                    if hb_period is None or (now - last_publish[cam]) < hb_period:
                        continue
                    msg.heartbeat = True
                    last_publish[cam] = now

                if args.drop_rate > 0 and random.random() < args.drop_rate:
                    dropped += 1
                    continue

                payload = msg.SerializeToString()
                sock.sendto(payload, dest)
                sent += 1

                if not args.quiet and sent % 30 == 0:
                    kind = "HB " if msg.heartbeat else "DET"
                    rng_s = (f" range={msg.detections[0].t_tag_wrt_cam[2]:6.2f}m"
                             if msg.detections else "")
                    print(f"  t={t:6.1f}s [{cam:>8}] {kind} frame={msg.frame_id:5d}"
                          f" bytes={len(payload):3d}{rng_s}")

            slept = time.monotonic() - now
            if period > slept:
                time.sleep(period - slept)

    except KeyboardInterrupt:
        print()

    print(f"\nsent {sent} datagrams" + (f", dropped {dropped} (simulated)" if dropped else ""))


if __name__ == "__main__":
    main()
