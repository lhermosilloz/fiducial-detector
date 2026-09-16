# Raspberry Pi deployment

**Reference boards:** Pi 4 and Pi 5, Raspberry Pi OS **Bookworm** (64-bit), Raspberry Pi
HQ camera (IMX477).

This is the platform the service exists to reach. `jetson-vision-service` runs TensorRT
engines and is Jetson-locked; this one is OpenCV CPU, protobuf, GStreamer and libc
sockets — all apt-installable on a Pi. Nothing here is a port or a special case: it is
the same source tree and the same binary build, differing from the Jetson only in the
`gst:` line of a YAML file.

> Nexus's own `deploy/rasberrypi/RPI_DEPLOYMENT.md` still contains the single word
> `TODO`. This service is a plausible candidate to be the first thing that actually
> fills it in, because its dependency surface is far smaller than Nexus core's. That is
> a reason to keep it small.

---

## 1. Bookworm uses libcamera. Say it once more.

The GStreamer element is **`libcamerasrc`**. `rpicamsrc` is the legacy Buster/Broadcom
element: it is not present on Bookworm and is not what you want. Any guide telling you
otherwise predates Bookworm.

Confirm the camera enumerates before anything else:

```bash
./tools/check_camera.sh --capture
```

or by hand:

```bash
rpicam-hello --list-cameras
gst-inspect-1.0 libcamerasrc
gst-launch-1.0 libcamerasrc num-buffers=30 \
  ! video/x-raw,width=1280,height=720,format=NV12 ! fakesink -v
```

If `libcamerasrc` is missing:

```bash
sudo apt-get install gstreamer1.0-libcamera
```

If the camera does not enumerate at all, it is the ribbon or the connector, not this
service. On a Pi 5 note there are two camera connectors and they need the narrower
cable — a Pi 4 cable does not fit.

---

## 2. Build and install

On the Pi:

```bash
sudo ./deploy/install_build_deps.sh
./deploy/build.sh
sudo ./deploy/pi/install_service.sh --start
```

From a development machine:

```bash
./deploy/pi/deploy_to_pi.sh --host raspberrypi.local --user pi --deps --start
```

**Build notes specific to this board.** `deploy/build.sh` caps parallelism to `-j2` on a
board with ≤2 GB RAM: compiling protobuf-generated code with `-j4` on a 2 GB Pi 4 will
meet the OOM killer, and the resulting error message blames the compiler rather than the
memory. A Pi 5 or a 4 GB Pi 4 builds at full width.

Raspberry Pi OS Bookworm ships **OpenCV 4.6**, which predates OpenCV moving ArUco from
contrib into core `objdetect`. The service handles both APIs in
`include/aruco_compat.h`, selected by `CV_VERSION` — a library-version branch, not a
platform branch. You will see which path was taken in the CMake output and in
`fiducial-detector-service --version`:

```
aruco API  cv::aruco::detectMarkers (OpenCV contrib < 4.7)
```

Debian's `libopencv-dev` bundles the contrib modules, so this works out of the box. A
hand-built OpenCV without `-DOPENCV_EXTRA_MODULES_PATH` will not have `cv::aruco` at all,
and CMake warns about exactly that.

---

## 3. Performance — the part that is genuinely different here

The Pi 4 CPU is the constraint. ArUco/AprilTag at 640×480 is roughly 30 fps on a Pi 4,
but **40–80 ms end-to-end once GStreamer latency is counted**. At a 2 m/s descent that
is about 10 cm of pose error, which is why `capture_timestamp_us` carries the sensor
exposure time rather than the publish time — it lets Nexus interpolate against telemetry
later without a proto change.

At 720p the detector does **not** hold 30 fps on a Pi 4. `config/pi_down.yaml` therefore
ships with:

```yaml
detect_fps: 15.0
```

This skips frames on a schedule instead of letting the detector fall progressively
behind. That is the right trade for this application: a latency backlog is worse than a
dropped frame, because a stale pose is a wrong pose.

**Tune it with the stats line, not with guesswork:**

```
[down] in=30.0fps det=15.0fps skip=15 tags=73 hb=0 avg_detect=41.20ms
```

- `avg_detect` is what one detect+solve costs. `1000 / avg_detect` is the ceiling on
  `detect_fps`. Set `detect_fps` comfortably below it, not at it.
- If `avg_detect` is too high, drop the stream to 640×480 before dropping `detect_fps`
  further — resolution buys you more than rate does here, and the detector's cost is
  roughly linear in pixels.
- A Pi 5 is roughly twice as fast; raise `detect_fps` and re-measure rather than
  assuming.

Other levers, in the order worth trying:

1. **Lower the resolution.** 640×480 is the sweet spot on a Pi 4. Remember that
   `calib_width`/`calib_height` must then reflect what you calibrated at — the service
   rescales intrinsics and logs it, but calibrating at the runtime resolution is better.
2. **`corner_refine: none`.** Subpixel refinement is a real cost. It also measurably
   improves pose accuracy, so this is a trade, not a free win. Measure `reproj` in the
   stats before and after.
3. **Keep NV12.** The shipped pipeline negotiates NV12 and the service uses plane 0
   directly as the grayscale image — no `videoconvert` in the hot path. A per-frame
   colour conversion at 720p is real money on this CPU. If you see
   `needs a per-frame colour conversion` in the log, your `gst:` line has regressed.
4. **Do not `undistort` unless you must.** It is a full-frame remap per frame. Turn it on
   only if a wide lens is measurably costing you detections.

Thermals matter on a Pi in an airframe. A throttled Pi 4 loses a large fraction of its
clock, and the symptom is `avg_detect` slowly rising over a flight. Check
`vcgencmd get_throttled` before blaming the code.

---

## 4. Calibrate before you believe any number

Same warning as the Jetson, and it applies per physical camera: the intrinsics in
`config/pi_down.yaml` are placeholders. Two IMX477s with nominally identical 6 mm lenses
do not share intrinsics closely enough for a 10 cm landing.

A 10% error in `fx`/`fy` or in `tags.default_size_m` is a 10% range error on every
detection. Measure `size_m` as the **outer edge of the black border**, not the paper:

```bash
python3 tools/make_test_video.py --print-marker /tmp/tag0.png --tag-id 0 --size-m 0.30
```

prints the marker and tells you exactly what dimension to hold the ruler against.

---

## 5. The cross-SoC topology, and the trap in it

A Pi running the detector next to a Jetson running Nexus is an intended deployment, and
it is the reason this service publishes protobuf over UDP instead of using MPA — MPA
cannot cross a SoC boundary.

To use it, point the Pi at the Jetson:

```yaml
udp:
  dest_ip: 192.168.1.50    # the Jetson
  dest_port: 5602
```

**Read this before you do.** `capture_timestamp_us` is `CLOCK_MONOTONIC`, which is
per-machine and per-boot. Two boxes produce two small, plausible-looking numbers that
are not comparable, and Nexus's staleness gate will then either pass everything or
reject everything — silently, in flight.

Every frame therefore carries `clock_domain` (the producing machine's boot id) and
`capture_realtime_us` (the same instant on `CLOCK_REALTIME`). A cross-box consumer must
either refuse a mismatched domain or apply a measured offset. **Detecting the mismatch is
this service's job; acting on it is Nexus's.** Confirm what is actually on the wire from
the Jetson side:

```bash
./tools/recv_fiducial.py --check
```

It flags a datagram over the ~1472-byte fragmentation limit and reports every distinct
`clock_domain` it sees.

Also check the obvious: a firewall on the Jetson, and that `dest_ip` is not still
`127.0.0.1`, which fails by publishing perfectly into nowhere.

---

## 6. Troubleshooting

**Service refuses to start.** It validates config before touching a camera:

```bash
fiducial-detector-service -c /etc/fiducial-detector-service/config.yaml
```

**`libcamerasrc` works in `gst-launch` but not in the service.** Permissions: the unit
runs as the unprivileged `fiducial` user, which needs `video` and `render`.

```bash
id fiducial
sudo usermod -aG video,render fiducial && sudo systemctl restart fiducial-detector-service
```

**Frames arrive but nothing is ever detected.** In order of likelihood: the tag family in
config does not match the tag you printed (`apriltag_36h11` is the default, Nexus's SITL
uses `aruco_4x4_50`); the tag is not in `tags.accept_ids`; the tag is too few pixels
across at that range. Set `logging.verbose: true` and watch what the detector says.

**Detections appear but the range is wrong by a consistent factor.** That is `size_m` or
`fx`/`fy`, essentially always. A consistent *ratio* is the signature — a 25% overestimate
usually means `size_m` was measured to the edge of the white quiet zone rather than the
black border.
