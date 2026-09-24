# Jetson deployment

**Reference board:** ARK Jetson Orin NX, JetPack 6, Raspberry Pi Camera v2 (IMX219).

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
| Frames appear | Argus and the ISP are working | Check the sensor mode list it prints, then see section 2. |
| Only raw Bayer over V4L2 (`RG10`/`BG10`) | The sensor is bound but there is no ISP in the path | You debayer yourself; see section 6. |
| Nothing enumerates | Device tree overlay and/or ribbon | Hardware. See below. |

**If nothing enumerates.** Pi camera modules use a 15-pin FPC and many Jetson carriers
are 22-pin, so an adapter is needed. Check ARK's documentation for which CSI ports are
actually wired on your carrier, since not all present connectors are. Then confirm the
driver bound:

```bash
sudo dmesg | grep -i imx219
ls /dev/video*
sudo systemctl restart nvargus-daemon
```

A missing device-tree overlay shows up as no `imx219` lines in dmesg at all.

A probe failure on one address alongside a successful bind on another, such as

```
imx219 9-0010: board setup failed
imx219 10-0010: subdev imx219 10-0010 bound
```

is normal with a single camera: it is the unpopulated CSI port failing its I2C probe.

---

## 2. Choose the sensor mode

This is the decision that determines field of view, and field of view is the binding
constraint for precision landing. List the modes:

```bash
nvargus_nvraw --lps
```

The IMX219 array is 3280x2464, which is what separates a binned mode from a cropped one:

| mode | resolution | fps | readout | horizontal FOV | visible at 1 m |
|---|---|---|---|---|---|
| 0 | 3280x2464 | 21 | full array | ~62° | 1.21 m |
| 1 | 3280x1848 | 28 | full width, vertical crop | ~62° | 1.21 m |
| 2 | 1920x1080 | 30 | centre crop | ~39° | 0.71 m |
| 3 | 1640x1232 | 30 | **2x2 binned, full FOV** | ~62° | 1.21 m |
| 4 | 1280x720 | 59 | crop | ~26° | 0.47 m |

`config/jetson_down.yaml` uses **mode 3**. It is exactly half of 3280x2464 in each axis,
so it is a binned readout that keeps the entire field of view at a manageable 2 MP.

Modes 2 and 4 are crops. Mode 4 in particular looks attractive for its 59 fps and is the
worst choice here: at 1 m altitude it sees 0.47 m across, so the pad leaves the frame
during final approach. Trade frame rate away with `detect_fps` instead of buying it with
field of view.

Verify rather than trust the table: `tools/calibrate_camera.py` reports the measured
horizontal FOV, and a cropped mode reports a markedly narrower one than a binned mode at
the same output resolution.

---

## 3. Build and install

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

## 4. Calibrate before relying on any number

The intrinsics shipped in `config/jetson_down.yaml` are derived from the IMX219's sensor
geometry: a 3.04 mm lens over 1.12 um pixels, binned 2x2 to 2.24 um, giving
fx = fy = 3040 / 2.24 = 1357 px at mode 3. That yields 62.3 x 48.8 degrees, which matches
the published module specification, so it is a sound starting point.

It is still not a calibration. It assumes a perfectly centred sensor and zero distortion,
and the IMX219 lens has visible barrel distortion toward the edges. A 10% error in
`fx`/`fy` or in `tags.default_size_m` produces a 10% range error on every detection, and
range is what the consumer ranks landing candidates on.

```bash
./tools/calibrate_camera.py --sensor imx219 --mode 3 --square-mm <measured>
```

`--sensor imx219 --mode 3` builds exactly the pipeline above and seeds the optimiser with
the 1357 px geometric estimate. `--list-modes` prints the mode table; `--gst "<pipeline>"`
takes over for any camera without a preset.

If you hand it a pipeline copied from `config/*.yaml`, note that those name the appsink
`sink` because the service looks it up by that name, whereas OpenCV only recognises a sink
whose name contains `appsink` or `opencvsink`. Mismatched, it reports `cannot find appsink
in manual pipeline`, which reads as though the element were missing. The tool renames it
and says so, so this is a thing to recognise rather than to fix.

The tool does not simply grab twenty frames. It works through a queue of target boxes and
will not accept a view until the board sits wholly inside the current box and covers
enough of it, which is what forces the coverage that constrains distortion and the
principal point. Watch it and aim the board using the MJPEG preview it prints:

```
preview:  http://192.168.55.1:8080
```

Open that in a browser on the development machine. The target box fades red to green as
the board approaches the required fill. On stdin, ENTER skips a target you cannot reach
(an obstructed corner) and `stop` ends sampling early.

It fails loudly rather than quietly: above 0.75 px RMS reprojection error it refuses to
call the result a calibration, and it reports per-view error so a single bad capture is
distinguishable from a systematically wrong `--square-mm` or a board that is not flat.

Recalibrate after any sensor-mode change: a different mode is a different crop or a
different binning factor, so the focal length and principal point both move.

Measure `tags.default_size_m` as the **outer edge of the black border**, not the white
quiet zone. `tools/make_test_video.py --print-marker` prints a marker and tells you
exactly what to measure.

---

## 5. Verify it works

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

## 6. If you are stuck on raw Bayer

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

## 7. Troubleshooting

**Service refuses to start.** It validates its config first, on purpose:

```bash
fiducial-detector-service -c /etc/fiducial-detector-service/config.yaml
```

That prints every problem and exits without touching a camera.

**`Failed to create CameraProvider` in the journal.** The process cannot reach
`nvargus-daemon`. Argus does not use `/dev/video*`; it talks to the daemon over a Unix
socket in `/tmp`, so this is usually a sandbox or daemon problem rather than a device
permission one.

In order of likelihood:

1. **systemd sandboxing.** `PrivateTmp=yes` gives the service a private `/tmp` and hides
   `/tmp/argus_socket`; `ProtectSystem=strict` makes `/run` and `/var` read-only, and
   `connect()` on a Unix socket needs write permission on the socket inode. The shipped
   unit sets neither for this reason. If you have hardened it further, that is the first
   thing to undo. Confirm by running as the same user without the sandbox:

   ```bash
   sudo systemctl stop fiducial-detector-service
   sudo -u fiducial /usr/local/bin/fiducial-detector-service \
        --config /etc/fiducial-detector-service/config.yaml --verbose
   ```

   Frames here but not under systemd means the sandbox, not permissions.

2. **The daemon is not running.** `sudo systemctl restart nvargus-daemon`

3. **Another Argus consumer holds the sensor.** Only one is permitted:
   `ps aux | grep gst-launch`

**`Failed to create CaptureSession`.** Distinct from the error above, and the distinction
is the whole diagnosis: `CameraProvider` failing means the process could not reach
`nvargus-daemon`, whereas `CaptureSession` failing means it reached the daemon fine and
then could not acquire the *sensor*. So the daemon, the socket, the device tree and the
driver are all working, and the sensor is busy or stale.

Argus permits exactly one client per sensor. The usual cause is simply that the service is
running:

```bash
sudo systemctl stop fiducial-detector-service
./tools/calibrate_camera.py --sensor imx219 --mode 3 --square-mm <measured>
sudo systemctl start fiducial-detector-service
```

If nothing obvious holds it, a previous client that died can leave the session behind;
`sudo systemctl restart nvargus-daemon` clears that. `ps aux | grep -E 'gst-launch|argus'`
finds the rest.

This is easy to mistake for a pipeline problem, because GStreamer reports the pipeline as
opened and only then produces no frames. `tools/calibrate_camera.py` checks for both
causes before it starts capturing and names them if it still gets no frame.

**`nvarguscamerasrc` works in `gst-launch` but not in the service.** If the error is not
the one above, it is usually group membership: the unit runs as the unprivileged
`fiducial` user.

```bash
id fiducial      # expect video and render
sudo usermod -aG video,render fiducial && sudo systemctl restart fiducial-detector-service
```

**Pipeline keeps rebuilding.** Argus wedges; the per-stream watchdog is doing its job.
Look for `pipeline_rebuilds=` in the stats line. Frequent rebuilds usually mean the
`nvargus-daemon` is unhealthy — restart it, and check whether another process
(`jetson-vision-service`, a stray `gst-launch`) is holding the same sensor.

**Nexus sees nothing.** Check the direction of the problem — `recv_fiducial.py` proves
the detector is publishing; if it sees frames and Nexus does not, the bug is on the Nexus
side of the contract, not here.
