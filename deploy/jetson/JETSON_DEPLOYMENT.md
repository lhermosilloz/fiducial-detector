# Jetson deployment

**Reference board:** ARK Jetson Orin NX, JetPack 6, Raspberry Pi HQ camera (IMX477).

This service runs on the Jetson CPU. It does not use CUDA or TensorRT; that is what
allows the same binary to run on a Raspberry Pi. Adding a GPU dependency would remove
that property.

It coexists with `jetson-vision-service`: that one owns UDP port 5601 and the GPU, this
one owns 5602 and some CPU. Running both on one Orin NX is an intended configuration.

---

## 1. Settle the ingest path first

This is the step people skip, and it is the step that decides everything else. Whether
the camera reaches you through Argus or through raw V4L2 is a device-tree-and-ribbon
question, not a software one, and a camera that does not enumerate produces exactly the
same symptom as a broken pipeline string.

```bash
./tools/check_camera.sh --capture
```

The decisive test inside it is:

```bash
gst-launch-1.0 nvarguscamerasrc num-buffers=30 \
  ! 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' \
  ! nvvidconv ! video/x-raw,format=NV12 ! fakesink -v
```

Three outcomes:

| Result | What it means | What to do |
|---|---|---|
| Frames appear | Argus and the ISP are working | Use `config/jetson_down.yaml` unchanged. Done. |
| Only raw Bayer over V4L2 (`RG10`/`BG10`) | The sensor is bound but there is no ISP in the path | You debayer yourself. See section 5 — it costs CPU and complexity. |
| Nothing enumerates | Device tree overlay and/or ribbon | Hardware. See below. |

**If nothing enumerates.** The Pi HQ camera is a 15-pin FPC and many Jetson carriers are
22-pin, so it needs an adapter — check ARK's docs for which CSI ports are actually wired
on your carrier, because not all of the connectors present are. Then confirm the driver
bound at all:

```bash
sudo dmesg | grep -i imx477
ls /dev/video*
sudo systemctl restart nvargus-daemon
```

A missing device-tree overlay shows up as no `imx477` lines in dmesg at all.

---

## 2. Build and install

On the Jetson:

```bash
sudo ./deploy/install_build_deps.sh
./deploy/build.sh
sudo ./deploy/jetson/install_service.sh --start
```

From a development machine:

```bash
./deploy/jetson/deploy_to_jetson.sh --host 192.168.55.1 --user jetson --deps --start
```

`192.168.55.1` is the Ethernet-over-USB default. The deploy script syncs source and
builds **on the board** — a native build is fast here and it is the only build that
proves the board's own OpenCV works.

---

## 3. Calibrate before you believe any number

The intrinsics shipped in `config/jetson_down.yaml` are **placeholders** computed from a
nominal 53° horizontal FOV. They are a bringup starting point, not a calibration.

The IMX477 is C/CS mount and **ships with no lens**, so intrinsics are entirely
lens-specific and must be re-measured on any lens swap. A 10% error in `fx`/`fy` or in
`tags.default_size_m` is a 10% range error on every detection — and `range_m` is what
Nexus ranks landing candidates on, so it is load-bearing, not diagnostic.

Two lens notes that matter more than resolution:

- **Take the wide option.** The 6 mm CS lens (~53° horizontal) over the 16 mm. At 1 m
  altitude a narrow lens loses the pad out of frame, which is precisely when you need
  it. FOV is the binding constraint for this application.
- **Use a binned sensor mode, not a cropped one**, at 720p or below. Cropping a 12.3 MP
  sensor narrows FOV drastically and silently — you get a perfectly sharp image of far
  too little of the world.

Measure `tags.default_size_m` as the **outer edge of the black border**, not the white
quiet zone. `tools/make_test_video.py --print-marker` prints a marker and tells you
exactly what to measure.

---

## 4. Verify it works

```bash
# Is it publishing?
./tools/recv_fiducial.py --raw

# Is the wire contract intact?
./tools/recv_fiducial.py --check --timeout 10

# No camera yet? Drive Nexus anyway.
./tools/fake_publisher.py --profile descent --duration 30

# No camera and want to test the real detector? Replay a synthetic clip
# with known ground truth:
python3 tools/make_test_video.py --out /tmp/tag.mp4
# then point a config's gst: at that file (see README).
```

The service logs a stats line every 5 s:

```
[down] in=30.0fps det=30.0fps skip=0 tags=147 hb=3 avg_detect=6.21ms
```

`in` is frames delivered by GStreamer, `det` is frames the detector actually ran on. If
`det` is far below `in` with `skip` climbing, the detector is rate-limited by
`detect_fps`. If `in` itself is low, the problem is upstream in the pipeline.

---

## 5. If you are stuck on raw Bayer

No ISP in the path means the CPU debayers. Replace the `gst:` line with something like:

```yaml
gst: >-
  v4l2src device=/dev/video0 !
  video/x-bayer,format=rggb,width=1280,height=720,framerate=30/1 !
  bayer2rgb ! videoconvert ! video/x-raw,format=GRAY8 !
  appsink name=sink drop=true max-buffers=2 sync=false
```

Expect to pay for it in CPU, and check the `avg_detect` and `in` fields in the stats line
before assuming it is free. It is worth pushing on the device tree instead: Argus does
this work in hardware.

Note that a fiducial detector only ever wants luma. If you are debayering to RGB and then
converting back to grey, you are doing work twice — but the Bayer-to-grey shortcut is
lower quality and the detector is sensitive to it, so measure before optimising.

---

## 6. Troubleshooting

**Service refuses to start.** It validates its config first, on purpose:

```bash
fiducial-detector-service -c /etc/fiducial-detector-service/config.yaml
```

That prints every problem and exits without touching a camera.

**`nvarguscamerasrc` works in `gst-launch` but not in the service.** Almost always
permissions: the unit runs as the unprivileged `fiducial` user. Confirm it is in `video`:

```bash
id fiducial
sudo usermod -aG video fiducial && sudo systemctl restart fiducial-detector-service
```

**Pipeline keeps rebuilding.** Argus wedges; the per-stream watchdog is doing its job.
Look for `pipeline_rebuilds=` in the stats line. Frequent rebuilds usually mean the
`nvargus-daemon` is unhealthy — restart it, and check whether another process
(`jetson-vision-service`, a stray `gst-launch`) is holding the same sensor.

**Nexus sees nothing.** Check the direction of the problem — `recv_fiducial.py` proves
the detector is publishing; if it sees frames and Nexus does not, the bug is on the Nexus
side of the contract, not here.
