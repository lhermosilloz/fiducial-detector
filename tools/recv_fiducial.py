#!/usr/bin/env python3
"""Bind 5602 and decode FiducialFrame datagrams. The other half of the harness.

This stands in for Nexus's UdpTagSource during bringup: it proves the detector is
publishing, shows what it is publishing, and — the part that matters — checks the
properties a silent failure would violate.

It deliberately does NOT convert to NED. That transform lives in Nexus, and duplicating it here would create a second place for a mount-sign error
to hide.

Usage
-----
  ./recv_fiducial.py                    # live table
  ./recv_fiducial.py --raw              # one line per datagram
  ./recv_fiducial.py --check            # exit nonzero if an invariant is violated
  ./recv_fiducial.py --timeout 5        # give up after 5 s of silence
"""

import argparse
import collections
import os
import socket
import subprocess
import sys
import tempfile
import time


# protoc on many boards (Bookworm, JetPack 6) predates the installed python
# protobuf runtime, and the C++ descriptor implementation refuses the older
# generated code outright. The pure-python implementation accepts it and is more
# than fast enough for a test harness. Must be set BEFORE protobuf is imported.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

def load_proto_module():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    sys.path.insert(0, here)
    try:
        import fiducial_pb2
        return fiducial_pb2
    except ImportError:
        pass
    proto = os.path.join(repo, "protos", "fiducial.proto")
    if not os.path.exists(proto):
        sys.exit(f"cannot find {proto}")
    out = tempfile.mkdtemp(prefix="fiducial_pb_")
    subprocess.run(["protoc", f"--python_out={out}",
                    f"--proto_path={os.path.dirname(proto)}", proto], check=True)
    sys.path.insert(0, out)
    import fiducial_pb2
    return fiducial_pb2


pb = load_proto_module()


class CameraState:
    def __init__(self):
        self.frames = 0
        self.heartbeats = 0
        self.detections = 0
        self.last_frame_id = None
        self.gaps = 0
        self.last_rx = 0.0
        self.max_bytes = 0
        self.ranges = collections.deque(maxlen=50)
        self.reproj = collections.deque(maxlen=50)
        self.latency_ms = collections.deque(maxlen=50)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5602)
    ap.add_argument("--raw", action="store_true", help="one line per datagram")
    ap.add_argument("--check", action="store_true",
                    help="verify wire invariants and exit nonzero on violation")
    ap.add_argument("--timeout", type=float, default=0.0,
                    help="seconds of silence before giving up (0 = forever)")
    ap.add_argument("--interval", type=float, default=2.0, help="summary interval")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    sock.settimeout(0.5)

    print(f"listening on {args.host}:{args.port}  (Ctrl-C to stop)\n")

    cams = {}
    problems = []
    clock_domains = set()
    t_last_summary = time.monotonic()
    t_last_rx = time.monotonic()
    total = 0

    def note(msg):
        if msg not in problems:
            problems.append(msg)
            print(f"  !! {msg}")

    try:
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except socket.timeout:
                if args.timeout and (time.monotonic() - t_last_rx) > args.timeout:
                    print(f"\nno datagram for {args.timeout}s — giving up")
                    break
                data = None

            now = time.monotonic()

            if data is not None:
                t_last_rx = now
                total += 1
                msg = pb.FiducialFrame()
                try:
                    msg.ParseFromString(data)
                except Exception as e:
                    note(f"undecodable datagram ({e})")
                    continue

                cam = msg.camera_id or "<empty>"
                st = cams.setdefault(cam, CameraState())
                st.frames += 1
                st.last_rx = now
                st.max_bytes = max(st.max_bytes, len(data))

                if not msg.camera_id:
                    note("a frame has an empty camera_id — Nexus demuxes on it")
                if not msg.clock_domain:
                    note("a frame has an empty clock_domain — required, not optional")
                else:
                    clock_domains.add(msg.clock_domain)

                # frame_id is monotonic per stream precisely so a consumer can
                # see UDP gaps. A gap is not an error — UDP is lossy by design —
                # but a gap that is not counted is a gap nobody knows about.
                if st.last_frame_id is not None and msg.frame_id != st.last_frame_id + 1:
                    st.gaps += 1
                st.last_frame_id = msg.frame_id

                if msg.publish_timestamp_us and msg.capture_timestamp_us:
                    lat = (msg.publish_timestamp_us - msg.capture_timestamp_us) / 1000.0
                    st.latency_ms.append(lat)
                    if lat < 0:
                        note("publish_timestamp_us precedes capture_timestamp_us — "
                             "capture time is probably being stamped at publish")

                if len(data) > 1472:
                    note(f"datagram is {len(data)} bytes — fragments on a real Ethernet "
                         f"path, and one lost fragment drops the whole frame")

                if msg.heartbeat:
                    st.heartbeats += 1
                    if msg.detections:
                        note("a heartbeat frame carries detections — it must be empty")
                else:
                    st.detections += len(msg.detections)
                    for d in msg.detections:
                        if len(d.t_tag_wrt_cam) != 3:
                            note(f"t_tag_wrt_cam has {len(d.t_tag_wrt_cam)} floats, expected 3")
                            continue
                        if len(d.r_tag_to_cam) not in (0, 9):
                            note(f"r_tag_to_cam has {len(d.r_tag_to_cam)} floats, expected 9")
                        rng = d.t_tag_wrt_cam[2]
                        st.ranges.append(rng)
                        st.reproj.append(d.reproj_error_px)
                        if rng <= 0:
                            note(f"tag {d.tag_id} reports range {rng:.3f} m — "
                                 "a tag behind the camera is a bad pose solve")
                        if d.size_m <= 0:
                            note(f"tag {d.tag_id} has size_m={d.size_m} — "
                                 "a wrong size is a proportional range error")
                        if d.camera_id != msg.camera_id:
                            note("detection.camera_id disagrees with frame.camera_id")

                if args.raw:
                    kind = "HB " if msg.heartbeat else "DET"
                    extra = ""
                    if msg.detections:
                        d = msg.detections[0]
                        extra = (f" id={d.tag_id} t=({d.t_tag_wrt_cam[0]:+.3f},"
                                 f"{d.t_tag_wrt_cam[1]:+.3f},{d.t_tag_wrt_cam[2]:.3f})"
                                 f" reproj={d.reproj_error_px:.2f}px")
                    print(f"[{cam:>8}] {kind} frame={msg.frame_id:6d} "
                          f"{msg.frame_width}x{msg.frame_height} "
                          f"bytes={len(data):3d} detect={msg.detect_ms:5.2f}ms{extra}")

            if not args.raw and (now - t_last_summary) >= args.interval:
                dt = now - t_last_summary
                print(f"--- {time.strftime('%H:%M:%S')} "
                      f"({total} datagrams total) ---")
                for cam, st in sorted(cams.items()):
                    age = now - st.last_rx
                    live = "LIVE" if age < 1.0 else f"STALE {age:.1f}s"
                    rng = (f"{sum(st.ranges)/len(st.ranges):.2f}m" if st.ranges else "—")
                    rep = (f"{sum(st.reproj)/len(st.reproj):.2f}px" if st.reproj else "—")
                    lat = (f"{sum(st.latency_ms)/len(st.latency_ms):.1f}ms"
                           if st.latency_ms else "—")
                    print(f"  [{cam:>8}] {live:>10}  {st.frames/dt:5.1f} fps  "
                          f"hb={st.heartbeats:4d}  tags={st.detections:5d}  "
                          f"range~{rng:>8}  reproj~{rep:>8}  cap→pub~{lat:>8}  "
                          f"gaps={st.gaps}  max={st.max_bytes}B")
                    st.frames = st.heartbeats = st.detections = 0
                if len(clock_domains) > 1:
                    note(f"{len(clock_domains)} distinct clock_domains seen — "
                         "these timestamps are NOT comparable")
                print()
                t_last_summary = now

    except KeyboardInterrupt:
        print()

    print(f"\ntotal datagrams: {total}")
    for cam, st in sorted(cams.items()):
        print(f"  [{cam}] frame_id gaps: {st.gaps}, largest datagram: {st.max_bytes} B")
    if clock_domains:
        print(f"  clock_domain(s): {', '.join(sorted(clock_domains))}")

    if problems:
        print(f"\n{len(problems)} wire-contract problem(s):")
        for p in problems:
            print(f"  - {p}")
        if args.check:
            return 1
    elif args.check:
        print("\nwire contract: OK")

    if args.check and total == 0:
        print("no datagrams received")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
