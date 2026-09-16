# fiducial-detector-service

Headless service that reads camera frames, detects fiducial markers (AprilTag / ArUco),
estimates each marker's pose **in the camera optical frame**, and publishes the result as
protobuf datagrams over UDP to **127.0.0.1:5602**.

Its consumer is **Nexus**, which converts those camera-frame poses into world NED and
feeds them to the precision-landing mission.

**Runs unmodified on Jetson and Raspberry Pi.** One source tree, one build, no CUDA, no
TensorRT, no platform `#ifdef`s. The only platform-varying value in the entire service is
the GStreamer pipeline string in a YAML file.

---

## Status

Implemented and verified against synthetic ground truth on x86. **Camera ingest is
confirmed on the ARK Orin NX**: Argus binds the IMX219 (Raspberry Pi Camera v2) and
streams at 1080p30, so `config/jetson_down.yaml` pins `sensor-mode=3`, the 1640x1232
binned mode that keeps the full field of view. The service itself has
not yet been run against that camera — see
[deploy/jetson/JETSON_DEPLOYMENT.md](deploy/jetson/JETSON_DEPLOYMENT.md) for the bench
procedure. Pi hardware is untouched.

| | Jetson | Raspberry Pi |
|---|---|---|
| Reference board | ARK Jetson Orin NX, JetPack 6 | Pi 4 / Pi 5, Bookworm |
| Camera ingest | `nvarguscamerasrc` | `libcamerasrc` |
| OpenCV | 4.8+ (aruco in core `objdetect`) | 4.6 (aruco in contrib) |
| Detection | OpenCV CPU | OpenCV CPU |
| Sensor modes | IMX219: 5 modes, mode 3 (1640x1232 binned) used | not yet probed |
| Status | ingest confirmed; service not yet bench-run | code complete, untested on board |

Both OpenCV API generations are handled in [`include/aruco_compat.h`](include/aruco_compat.h),
selected by `CV_VERSION`. That is a library-version branch, and it is the only conditional
compilation in the tree.

---

## Quick start

```bash
sudo ./deploy/install_build_deps.sh     # same script on both platforms
./deploy/build.sh                       # same script on both platforms
./tools/check_camera.sh --capture       # settle the ingest path BEFORE anything else

sudo ./deploy/jetson/install_service.sh --start     # or deploy/pi/install_service.sh
```

From a development machine:

```bash
./deploy/jetson/deploy_to_jetson.sh --host 192.168.55.1 --deps --start
./deploy/pi/deploy_to_pi.sh --host raspberrypi.local --deps --start
```

Platform guides: **[deploy/jetson/JETSON_DEPLOYMENT.md](deploy/jetson/JETSON_DEPLOYMENT.md)**
· **[deploy/pi/PI_DEPLOYMENT.md](deploy/pi/PI_DEPLOYMENT.md)**

---

## No camera? Everything still works.

The detector takes no telemetry and has no notion of a world frame, so it is a pure
function of its input. That makes it fully testable on a laptop.

**Drive Nexus with no detector at all:**

```bash
./tools/fake_publisher.py --profile descent --duration 30
```

Synthesises `FiducialFrame` datagrams on 5602 — a tag descending 10 m → 0.3 m with a
lateral wobble. This drives the whole Nexus precision-landing state machine with no
Gazebo, no camera and no drone. Other profiles: `static`, `nested` (approach tag plus a
small final-metre tag), `intermittent` (bursts separated by heartbeat gaps),
`heartbeat`.

**Test the real detector against known ground truth:**

```bash
python3 tools/make_test_video.py --out /tmp/tag.mp4
./build/fiducial-detector-service --config config/replay_test.yaml
```

`make_test_video.py` renders a tag at an analytically known pose and writes
`/tmp/tag.mp4.truth.json` beside it, so the published range can be checked against
arithmetic rather than a tape measure. On this clip the service resolves range to within
**0.3% mean** with 0.2 px mean reprojection error and 100% detection.

**Watch what is on the wire:**

```bash
./tools/recv_fiducial.py --raw       # one line per datagram
./tools/recv_fiducial.py --check     # verify wire invariants, exit nonzero on violation
```

`--check` is the one to run in CI. It catches an empty `camera_id` or `clock_domain`, a
heartbeat carrying detections, a publish stamp preceding its capture stamp, a datagram
past the fragmentation limit, and a tag reporting negative range.

---

## Configuration

One YAML file with a `streams:` list. Multi-camera is the shape from day one: N cameras,
one process, one socket, demuxed by `camera_id`.

| File | |
|---|---|
| `config/jetson_down.yaml` | Jetson, one down camera |
| `config/pi_down.yaml` | Pi, one down camera |
| `config/jetson_dual.yaml` | two cameras, one process |
| `config/replay_test.yaml` | file replay, no hardware |

The Jetson and Pi configs differ in **one substantive value**, and that is the whole
portability claim:

```
$ diff config/jetson_down.yaml config/pi_down.yaml
<       nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),... ! nvvidconv ! ...
>       libcamerasrc ! video/x-raw,...,format=NV12 !
<     detect_fps: 0.0        # Orin NX keeps up at 720p
>     detect_fps: 15.0       # Pi 4 does not — see PI_DEPLOYMENT.md section 3
```

Keep that diff this small.

### The config self-heals

Missing keys are filled with documented defaults and written back, so a half-written
config becomes a complete, readable one. Comments are not preserved by that round-trip.

```bash
fiducial-detector-service -c config/pi_down.yaml     # validate and exit, never starts
```

`-c` exits 0 when the config is usable, so provisioning scripts can bootstrap and check a
config before a camera exists. The systemd unit runs it as `ExecStartPre`, which turns a
bad deploy into a loud failure at boot instead of a restart loop.

### The settings that will actually bite you

```yaml
camera:
  fx: 1357.0        # Calibrate these; shipped values are geometry estimates.
  fy: 1357.0
  cx: 640.0
  cy: 360.0
tags:
  default_size_m: 0.30    # OUTER EDGE OF THE BLACK BORDER, not the quiet zone
```

A 10% error in `fx`/`fy` or `default_size_m` is a 10% range error on every detection, and
`range_m` is what Nexus ranks landing candidates on — it is load-bearing, not diagnostic.
Intrinsics are specific to the camera, lens and sensor mode, and must be re-measured
after any change to those.

```bash
python3 tools/make_test_video.py --print-marker /tmp/tag0.png --size-m 0.30
```

prints a marker and tells you exactly which dimension to measure.

---

## Wire contract

[`protos/fiducial.proto`](protos/fiducial.proto). A frame with a handful of detections is
100–300 bytes.

```protobuf
message FiducialDetection {
  int32  tag_id = 1;  string family = 2;  float size_m = 3;
  repeated float t_tag_wrt_cam = 4;   // 3, camera OPTICAL frame; [2] = range
  repeated float r_tag_to_cam  = 5;   // 9, row-major
  float  reproj_error_px = 6;         // load-bearing: Nexus ranks on it
  string camera_id = 7;
  int32  loc_type = 8;                // 0 = camera-frame only
  ...
}
message FiducialFrame {
  uint64 capture_timestamp_us = 1;    // sensor exposure, CLOCK_MONOTONIC — not publish
  uint64 publish_timestamp_us = 2;
  string clock_domain = 3;            // boot id — required, see below
  int32  frame_id = 4;  ...  bool heartbeat = 8;
  repeated FiducialDetection detections = 9;
  float  detect_ms = 10;
}
```

Three properties worth knowing before you touch it:

- **Pose is camera-optical-frame only.** Nexus owns optical → body → NED, because mount
  extrinsics are per-vehicle and a mount-sign error should be debuggable in one place.
  This service never sees telemetry.
- **Heartbeats are mandatory.** Every camera publishes at `heartbeat_hz` even with
  nothing in view. Without them, "detector crashed" and "tag not visible" are the same
  silence, and a landing mission cannot tell abort-to-RTL from keep-descending. Liveness
  is **per camera** — a dead forward camera must not mask a live down one. A stream whose
  pipeline is down deliberately stops heartbeating.
- **`clock_domain` is not optional.** `capture_timestamp_us` is `CLOCK_MONOTONIC`:
  per-machine, per-boot. On the cross-SoC topology that motivated UDP (Pi detector →
  Jetson Nexus) two boxes emit two small plausible numbers that are not comparable, and a
  staleness gate then passes everything or rejects everything, silently, in flight. Every
  frame carries the producing machine's boot id and a `CLOCK_REALTIME` stamp so a
  consumer can detect the mismatch or measure the offset.

**Never add an image payload.** Past ~1472 bytes a datagram fragments on real Ethernet,
and one lost fragment drops the whole frame — breaking only on the cross-SoC path the
transport was chosen for. The publisher warns if a datagram crosses that line.

### The proto has two copies right now

`protos/fiducial.proto` in this repo is a **bootstrap copy** so the service builds today.
It must be upstreamed into `nexus-protos` and consumed from there, because two copies of
a wire contract synchronised by hand is precisely the hazard that ruled out MPA's packed
struct — it fails slowly and quietly, at the field-number level, in flight.

```bash
./deploy/build.sh --nexus-protos /path/to/nexus-protos
```

builds against the canonical file. Once it has landed upstream, take `nexus-protos` as a
submodule at `protos/` and delete the bootstrap copy.

---

## Reading the logs

```
[down] in=30.0fps det=30.0fps skip=0 tags=147 hb=3 avg_detect=6.21ms
[down] in=30.0fps det=15.0fps skip=15 tags=73 hb=0 avg_detect=41.20ms  rejected[reproj=4 range=1]
```

| | |
|---|---|
| `in` | frames GStreamer delivered |
| `det` | frames the detector actually ran on |
| `skip` | frames dropped by the `detect_fps` limiter |
| `avg_detect` | detect + pose solve, ms. `1000/avg_detect` is your ceiling on `detect_fps` |
| `hb` | heartbeats sent |
| `rejected[...]` | detections that decoded but failed a quality gate |
| `pipeline_rebuilds` | watchdog fired — a wedged pipeline was torn down and rebuilt |

`rejected[reproj=...]` climbing means the pose solve disagrees with the detected corners:
suspect `size_m`, intrinsics, or motion blur. This is the gate `voxl-tag-detector` left as
a `// TODO check hamming and validity` and never implemented.

---

## Layout

```
CMakeLists.txt              one build, both platforms, no CUDA
protos/fiducial.proto       wire contract (bootstrap copy — belongs in nexus-protos)
include/ src/
  aruco_compat.h            the ONLY conditional compilation: OpenCV <4.7 vs >=4.7
  app_config.*              YAML streams: list, self-healing defaults, validation
  gst_capture.*             appsink, NV12 plane-0 zero-copy, PTS -> CLOCK_MONOTONIC
  detector.*                ArUco/AprilTag + solvePnP IPPE_SQUARE + reprojection gate
  udp_publisher.*           protobuf -> sendto()
  stream_worker.*           one camera, one thread, per-stream watchdog
  clock_domain.*            boot id and monotonic/realtime stamps
config/                     jetson_down, pi_down, jetson_dual, replay_test
systemd/                    one unit, both platforms
deploy/                     shared scripts + jetson/ and pi/ wrappers and guides
tools/
  fake_publisher.py         synthetic frames -> drives Nexus with no hardware
  recv_fiducial.py          decode + verify wire invariants
  make_test_video.py        synthetic clip with ground truth; also prints markers
  calibrate_camera.py       measure intrinsics; emits the config's camera: block
  check_camera.sh           does the camera enumerate, and by which path?
```

File naming mirrors `jetson-vision-service` (`app_config`, `udp_publisher`) where the role
matches, so one reviewer reads both services without re-learning a layout.

---

## What is deliberately not here

- **World-frame poses.** No MAVLink, no telemetry, no NED. The consumer owns the
  optical to body to NED transform, because mount extrinsics are per-vehicle.
- **Any GPU dependency.** It would remove the Raspberry Pi target.
- **Consumer-side code.** `UdpTagSource` is built in the Nexus repository. The contract
  between the two is `fiducial.proto` and nothing else.
- **Any code derived from `voxl-tag-detector`.** Its sources carry a licence clause
  restricting use to ModalAI hardware. OpenCV's ArUco detector reads AprilTag
  dictionaries directly, so no vendored AprilTag library is required.
