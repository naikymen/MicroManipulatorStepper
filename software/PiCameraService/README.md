# PiCameraService

Serves a Raspberry Pi HQ Camera (Sony IMX477) over HTTP so it can be used as a
camera in the [OpenMicroManipulatorGUI](../OpenMicroManipulatorGUI/), and so that
photometric measurements can be taken from a workstation.

The service is deliberately split into two paths with different guarantees:

| Path | Endpoint | Transport | Use it for |
| --- | --- | --- | --- |
| **Preview** | `GET /stream.mjpg` | MJPEG over HTTP | Framing, focusing, live feedback |
| **Measurement** | `POST /capture` | JSON API | Anything you will quantify or publish |

**The MJPEG preview is lossy and must not be used for photometry.** It runs
through the hardware JPEG encoder at a reduced resolution and a compression
quality chosen for smoothness, not accuracy. A still capture stops the preview,
reconfigures the sensor for a full-resolution frame, and writes both a JPEG and a
DNG on the Pi, so the RAW data survives regardless of what happens on the
network. See [Preview vs measurement](#preview-vs-measurement).

Background, optics and the rationale for this design are in
[the camera setup and design notes](../../documentation/camera/pi_zero_hq_microscope_setup.md).

A separate, optional workstation tool — `bridge/pi_camera_v4l2_bridge.py` — can
relay the preview into a local `v4l2loopback` device so generic Linux
applications see the camera as a webcam. It is independent of the service and of
the measurement path. See
[Expose the preview as a local video device](#expose-the-preview-as-a-local-video-device).

---

## Contents

- [Hardware and software assumptions](#hardware-and-software-assumptions)
- [Install](#install)
- [Run it](#run-it)
- [Run it as a service](#run-it-as-a-service)
- [Finding the Pi from the workstation](#finding-the-pi-from-the-workstation)
- [HTTP API](#http-api)
- [Controls](#controls)
- [Still captures and the measurement index](#still-captures-and-the-measurement-index)
- [Preview vs measurement](#preview-vs-measurement)
- [Expose the preview as a local video device](#expose-the-preview-as-a-local-video-device)
- [Performance](#performance)
- [Troubleshooting](#troubleshooting)
- [Tests](#tests)

---

## Hardware and software assumptions

Verified against:

- Raspberry Pi Zero 2 W (Rev 1.0), quad core, **415 MB RAM (~229 MB available)**
- Raspberry Pi HQ Camera with the IMX477 sensor, plus a 40x finite-conjugate
  RMS microscope objective
- Raspberry Pi OS based on Debian 13 (trixie), aarch64, kernel 6.18.50
- Python 3.13, `picamera2` 0.3.37, `libcamera` v0.7.2

On Raspberry Pi OS the camera stack ships with the image:

```bash
sudo apt install -y python3-picamera2 python3-libcamera
```

**Do not install `picamera2` from PyPI.** The pip wheels cannot talk to the
VC4/PiSP hardware pipelines, which is what makes the hardware MJPEG encoder and
the full sensor resolution available.

`zeroconf` is **optional** and only used for mDNS advertisement and `--scan`:

```bash
sudo apt install -y python3-zeroconf   # optional
```

Without it the service still runs; it just does not announce itself, so you reach
it by address (see [Finding the Pi](#finding-the-pi-from-the-workstation)).

## Install

The reference deployment lives in `/home/pi/PiCameraService`:

```bash
scp -r PiCameraService pi@<pi-address>:/home/pi/
ssh pi@<pi-address>
sudo apt install -y python3-picamera2 python3-libcamera
cd /home/pi/PiCameraService
python3 picamera_daemon.py --list-controls   # sanity check: does the sensor answer?
```

If you prefer a virtual environment, create it with `--system-site-packages` so
the apt-provided `picamera2` stays visible:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

## Run it

```bash
# Foreground, defaults (0.0.0.0:8000, 1024x768 @ 25 fps)
python3 picamera_daemon.py

# Detached, logging to a file
setsid nohup python3 picamera_daemon.py --log-level info > /tmp/daemon.log 2>&1 &
```

Full option list:

```
--host HOST            interface to bind (default: 0.0.0.0)
--port PORT            port to bind (default: 8000)
--width WIDTH          preview width (default: 1024)
--height HEIGHT        preview height (default: 768)
--fps FPS              preview frame rate (default: 25)
--quality {very_low,low,medium,high,very_high}
                       MJPEG preview quality (default: medium)
--bitrate BITRATE      explicit MJPEG bitrate in bits/s, overriding the derived default
--buffer-count N       frames requested from libcamera, fewer means lower latency (default: 3)
--capture-dir DIR      where still captures and shots.csv are written (default: captures)
--device-name NAME     name advertised over mDNS (default: Pi HQ Camera)
--max-viewers N        maximum simultaneous MJPEG viewers (default: 4)
--no-advertise         do not announce over mDNS
--log-level L          debug, info, warning or error (default: info)
--list-controls        print control ranges and exit
--scan                 scan the network for other services and exit
```

`--quality` is advisory: on the hardware MJPEG encoder it does not change the
bitrate. Use `--bitrate` when you need to bound the network load.

**`--log-level` values are lowercase.** Passing `INFO` is rejected by argparse.

## Run it as a service

```bash
sudo cp systemd/picamera.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now picamera
systemctl status picamera
journalctl -u picamera -f
```

The unit assumes the service is at `/home/pi/PiCameraService`. Edit
`WorkingDirectory` and `ExecStart` if you installed it elsewhere. It waits for
`network-online.target` and restarts on failure.

Stopping the service is clean: the daemon exits within about a second after
`SIGTERM` rather than lingering on libcamera's internal threads.

## Finding the Pi from the workstation

The GUI probes, in order of cost:

1. **The address you used last**, remembered in `QSettings` under
   `Connections/last_pi_camera_address`. This is the primary path in practice.
2. **The built-in defaults** in
   [`camera_pi.py`](../OpenMicroManipulatorGUI/source/hardware/camera_pi.py):
   `raspberrypi.local`, `picam.local` and a literal reference address. Replace
   that literal address with your own, or leave it — an unreachable address just
   costs one short probe timeout.
3. **Anything you type into the camera box**, which accepts `host`, `host:port`,
   `http://host:port`, `http://host:port/info` and mDNS names alike.

There is **no network scan** and no manual "add camera" dialog by design: a
missing camera simply does not appear in the list, and you type its address
instead. A typed address that does not answer produces a dialog naming the exact
address that failed.

The GUI's default port is **8000**, matching this service. If you change one,
change the other — a test in [Tests](#tests) fails if they drift apart.

If you use a **USB Ethernet gadget** to connect the Pi directly to the
workstation, the Pi is typically reachable at `192.168.7.2` (or the address you
assigned to the `usb0`/`enx...` interface on the workstation side). Type that
address into the camera box.

## HTTP API

All JSON responses are objects. Errors are JSON with an `error` field, except the
stream and file routes, which use plain HTTP status codes.

| Method | Route | Purpose |
| --- | --- | --- |
| `GET` | `/` | Human-readable index of the API |
| `GET` | `/info` | Service identity, camera identity, stream configuration, control specs, current control values, sensor modes |
| `GET` | `/status` | Liveness plus viewer/frame counters |
| `GET` | `/stream.mjpg`, `/stream` | Multipart MJPEG preview |
| `GET` | `/snapshot.jpg`, `/snapshot` | A single JPEG frame, for one-off grabs |
| `GET` | `/captures` | Listing of previously written captures |
| `GET` | `/captures/<name>` | Download a capture (rejects path traversal) |
| `GET` | `/control` | Control specs plus current control values |
| `POST` | `/control` | Apply libcamera controls |
| `POST` | `/capture` | Take a full-resolution still |
| `POST` | `/reconfigure` | Change preview geometry and frame rate |

`/info` is the probe endpoint. A client identifies a PiCameraService by
`payload["service"] == "PiCameraService"`, which is how the GUI avoids mistaking
a random web server for a camera.

### `POST /control`

```bash
curl -X POST http://<pi>:8000/control \
     -H 'Content-Type: application/json' \
     -d '{"controls": {"ExposureTime": 25000, "AeEnable": false, "AnalogueGain": 2.0}}'
```

```json
{"applied": {"ExposureTime": 25000, "AeEnable": false, "AnalogueGain": 2.0},
 "controls": {}, "warnings": []}
```

A bare control dict also works, so `-d '{"ExposureTime": 25000}'` is accepted.
Unknown control names and malformed values produce **warnings, not errors** — the
request still returns 200 and the sensor applies whatever it understood. This is
intentional: the set of controls an IMX477 exposes varies by sensor mode, and a
client should not have to know the exact list.

A `ValueError` returns **400**; an unusable camera returns **503**.

`GET /control` returns both the schema and the live values, and `/info` carries
the same values under `current_controls`:

```json
{"controls": {"ExposureTime": {"type": "int", "min": 37, "max": 667234896,
                               "default": 20000, "unit": "us", "description": "..."},
              "...": {}},
 "current":  {"ExposureTime": 20000, "AnalogueGain": 1.0, "AeEnable": true,
              "ColourGains": null, "AwbEnable": null, "ScalerCrop": [708, 528, 2640, 1980]}}
```

Note that the white balance controls read back as `null` on an IMX477 while the
ISP's own AWB is in charge, so a `null` there does not mean the value failed to
apply — use the resulting image, not the read-back, to judge white balance.

### `POST /capture`

| Query | Default | Meaning |
| --- | --- | --- |
| `raw` | `1` | Also write the DNG |
| `quality` | `95` | JPEG quality for the still |
| `label` | – | Suffix appended to the filename, e.g. `dark`, `sample-3` |

```bash
curl -X POST 'http://<pi>:8000/capture?raw=1&label=sample-3'
```

```json
{"ok": true, "stem": "20261009-170226-0004-sample-3", "label": "sample-3",
 "files": {"jpeg": {"name": "...jpg", "url": "/captures/...jpg", "bytes": 1877663},
           "raw":  {"name": "...dng", "url": "/captures/...dng", "bytes": 18496096}},
 "controls": {"ExposureTime": 20000, "AnalogueGain": 1.0, "ColourGains": [3.66, 1.53]},
 "metadata": {"Lux": ..., "SensorTimestamp": ..., "ColourTemperature": ...},
 "stream_configuration": {...}}
```

**`/capture` is POST-only and has no `GET` counterpart.** It stops the preview,
reconfigures the sensor, integrates, writes the files and restores the preview.
A `GET` returns 404. When testing by hand, always use `curl -X POST`.

A still takes a few seconds and blocks the preview for its duration; the next
`/capture` or reconnect restores streaming automatically.

## Controls

Discover what the attached sensor actually accepts:

```bash
python3 picamera_daemon.py --list-controls
```

On an IMX477 with `AeEnable` and `AwbEnable` both true:

| Control | Range | Notes |
| --- | --- | --- |
| `ExposureTime` | 37 – 667 234 896 us | **Mode-dependent.** The 1332x990 mode advertises a max of 667 234 896 us and a min of 37; the full-resolution modes advertise 694 422 939 us and a min of 92. `0` hands control back to the AEC. |
| `AnalogueGain` | 1.0 – 22.26 (linear) | The GUI sends decibels; `gain = 10 ** (dB / 20)`. |
| `AwbEnable` / `ColourTemperature` | 100 – 100 000 K | Setting a temperature while `AwbEnable` is true only biases the algorithm. |
| `ColourGains` | 0.0 – 32.0 | Advertised as a scalar but accepts a `[red, blue]` pair. |
| `Contrast`, `Saturation` | 0.0 – 32.0 | 1.0 is neutral. |
| `Sharpness` | 0.0 – 16.0 | 1.0 is neutral. |
| `ScalerCrop` | max `[696, 528, 2664, 1980]` | Digital zoom and region of interest. |

Two control names that look plausible **do not exist** and are quietly ignored
with a warning: `AeLocked` and every `Af*` autofocus control (the HQ Camera lens
is manual). There is no autofocus to drive.

Because unknown controls only warn, a client can post an optimistic control set
and inspect `warnings` to see what the sensor rejected.

## Still captures and the measurement index

Every capture also appends a row to `shots.csv` in the capture directory, so a
measurement can be traced back to the settings that produced it:

| stem | timestamp | label | jpeg | dng | ExposureTime | AnalogueGain | ColourGains | ColourTemperature | SensorTimestamp | Lux | FrameDuration |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |

Filenames are `YYYYMMDD-HHMMSS-NNNN[-label]`, monotonic within a session.
Downloads via `GET /captures/<name>` reject path traversal.

## Preview vs measurement

**Use the preview to see, the still capture to measure.** Concretely:

- The preview is MJPEG at 1024x768 by default, encoded by the hardware JPEG
  encoder, and is a lossy, downscaled view of the sensor.
- The same preview costs bandwidth continuously; a still costs it once.
- The preview runs through the ISP's automatic exposure, gain and white balance,
  which are re-evaluated per frame. A measurement should pin them: set
  `AeEnable: false` with an explicit `ExposureTime` and `AnalogueGain`, and
  `AwbEnable: false` with an explicit `ColourTemperature`. Read `current_controls`
  back to confirm the exposure and gain stuck; the white balance controls report
  `null` while the ISP's AWB is active, so judge those from the image.
- The DNG written by `/capture` is the unprocessed sensor data and is the only
  artefact that survives any post-hoc analysis.

The 40x objective changes what "good" looks like. The objective is a
**finite-conjugate** design: it expects a specific tube length and covers only
part of the sensor. Expect to set `ScalerCrop` to the illuminated region rather
than using the full frame, and expect the preview's default sensor mode to be a
downscale that looks soft at that magnification.

The sensor offers 15 modes, but only a few matter for a 1024x768 preview:

| Mode | Size | Bit depth | Max fps | Exposure limits (us) |
| --- | --- | --- | --- | --- |
| 0 | 1332x990 | 10 | 120.5 | 31 – 667 234 896 |
| 5 | 1332x990 | 12 | 101.7 | 37 – 667 234 896 |
| 2 | 2028x1520 | 10 | 53.8 | 47 – 674 181 621 |
| 7 | 2028x1520 | 12 | 45.2 | 56 – 674 181 621 |
| 12 | 2028x1520 | 8 | 66.4 | 38 – 674 181 621 |
| 4 | 4056x3040 | 10 | 14.0 | 92 – 694 422 939 |
| 14 | 4056x3040 | 8 | 17.4 | 74 – 694 422 939 |

The daemon lets libcamera choose, and for a 1024x768 preview it settles on
**mode 5, a 1332x990 12-bit readout** — libcamera's log confirms the final
`Selected sensor format: 1332x990-SBGGR12_1X12/RAW`, and the advertised exposure
range of 37 us matches that mode's. That is a **1.4x downscale**, the softest
option on the list. If the preview looks under-resolved under the 40x objective,
that is why. A 2028x1520 mode would give the ISP real detail to work with, at the
cost of frame rate. The mode is not currently selectable — `--width`/`--height`
influence it only indirectly, and there is no `--sensor-mode` flag. The full list
is published in `/info` under `sensor_modes` so a client can offer the choice.

## Expose the preview as a local video device

`bridge/pi_camera_v4l2_bridge.py` runs on the **workstation** and relays the
MJPEG preview into a `v4l2loopback` device, so any Linux application that expects
an ordinary webcam sees the Pi HQ Camera. This is the only reason to involve
`v4l2loopback` at all.

**It is optional, and it is independent of everything above.** The bridge only
reads `/stream.mjpg`, which the daemon serves regardless. It never writes to the
Pi, never changes the stream, and stopping it leaves the daemon, the control
endpoints and the still-capture path exactly as they were. Measurement does not
go through it — a relayed frame has been through ffmpeg's decode and re-layout,
and carries none of the `/capture` metadata. Use the bridge to see, the HTTP API
to measure.

It is also entirely separate from the GUI's **Pi Camera** entry. That entry talks
the HTTP API directly and needs no kernel module; the bridge exists for other
applications.

### Setup

```bash
# 1. Install the kernel module and the relay tool.
sudo pacman -S v4l2loopback-dkms v4l-utils     # Arch, EndeavourOS, Manjaro
sudo apt install v4l2loopback-dkms v4l-utils   # Debian, Ubuntu, Raspberry Pi OS

# 2. Create a device. Pick a number that is not already taken.
sudo modprobe v4l2loopback video_nr=10 card_label="Pi HQ Camera" exclusive_caps=1
```

The script prints the exact commands for the machine it is running on:

```bash
python3 bridge/pi_camera_v4l2_bridge.py --print-setup
python3 bridge/pi_camera_v4l2_bridge.py --list-devices
```

### Run it

```bash
python3 bridge/pi_camera_v4l2_bridge.py 192.168.1.39          # host or host:port
python3 bridge/pi_camera_v4l2_bridge.py 192.168.1.39 --fps 15
python3 bridge/pi_camera_v4l2_bridge.py http://pi.local:8000
```

Before touching ffmpeg the script checks the address really is a PiCameraService
(it validates the `service` field in `/info`), then prints which camera and size
it found together with the device path to select in the other application. Ctrl-C
stops the relay.

`--device /dev/video10` overrides the automatic choice. `--ffmpeg` overrides the
executable. The relay is delegated to ffmpeg because that is the well-trodden
path into `v4l2loopback`; nothing here re-encodes, the preview is already MJPEG
and is written out as YUV frames.

### It will not touch a real camera

Existing webcams are usually `/dev/video0` and `/dev/video1`, and writing video
into one is not something the kernel will stop you from attempting. The bridge
therefore refuses to run unless the target device really belongs to
`v4l2loopback`, checked through sysfs rather than trusted:

```
error: /dev/video0 is 'uvcvideo' (HP HD Camera: HP HD Camera), not a
v4l2loopback device. Refusing to write to a real camera.
```

That applies to an explicitly requested `--device` too. If no loopback device
exists, the script says so and prints the two commands that create one. This
safety property has dedicated tests.

### Caveats

- `exclusive_caps=1` is what makes most consumer applications accept the device.
  It is set in the commands above and was verified with both ffmpeg as the
  producer and OpenCV as the consumer; the GUI discovered and streamed from the
  resulting camera as `OpenCV Camera 10`.
- `/dev/videoN` numbering is not stable. Anything that cached a path must
  re-discover it — the same caveat as restarting the camera on the Pi.
- End-to-end relay verification: Linux kernel `7.2.9-zen1-1-zen`,
  v4l2loopback driver `7.2.9`, and ffmpeg `9.0.2`. OpenCV read twelve
  consecutive 1024x768 frames; the GUI also connected to the loopback entry and
  received a live frame. The physical webcams remained `/dev/video0` and
  `/dev/video1`, identified as `uvcvideo`, throughout. This verifies the tested
  setup; other kernel/module versions and applications may behave differently.

## Performance

Measured on the Raspberry Pi Zero 2 W with the hardware MJPEG encoder,
`--quality medium`:

| Requested | Negotiated | Target fps | Achieved fps | Mbit/s | Avg frame | Free RAM | Daemon RSS |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1024x768 | 1024x768 | 25 | 24.99 | 8.77 | 43.8 kB | 229 MB | 57 MB |
| 1280x960 | 1280x960 | 25 | 24.92 | 13.32 | 66.8 kB | 221 MB | 57 MB |
| 1600x1200 | 1600x1200 | 20 | 19.92 | 16.49 | 103.5 kB | 207 MB | 57 MB |
| 2028x1520 | **1920x1520** | 12 | 11.93 | 8.84 | 92.6 kB | 199 MB | 57 MB |
| 2028x1520 | **1920x1520** | 15 | 14.98 | 11.10 | 92.6 kB | 197 MB | 57 MB |

Two things to take from this table:

1. **Never trust the requested size.** 2028x1520 silently negotiates to
   1920x1520. Read back `stream.main.size` from `/info`, and note that a
   TCP/IP hop can hide upstream changes — check the actual JPEG dimensions
   (SOF marker) if the size matters.
2. **Bandwidth, not memory, is the constraint.** RSS stays flat at 57 MB and
   the Pi never came close to exhausting RAM. 1600x1200 is the practical
   ceiling for a smooth preview on this board; 1024x768 leaves headroom for
   everything else.

A still capture writes ~1.8 MB of JPEG and ~18.5 MB of DNG.

## Troubleshooting

**The daemon hangs on Ctrl-C or after `systemctl stop`.**
Fixed by exiting explicitly once the HTTP server returns. If you see it on an
older copy, update `picamera_daemon.py`.

**The preview is 640x480 no matter what I ask for.**
Also fixed. Enumerating picamera2's `sensor_modes` calls `configure()` once per
mode — and every one of those probes reconfigures the camera's *main* stream to
640x480, so the requested geometry was gone by the time streaming started. The
daemon now re-applies its configuration after the probe. You can see both halves
of this in the log: a run of `configuring streams: (0) 640x480-XBGR8888/sRGB`,
then a final `configuring streams: (0) 1024x768-RGB888/sRGB`.

If you hit this on an older copy, do not trust `/info`: it reports the size you
asked for, not the size being produced. Measure the JPEG headers instead —
`cap.set(cv2.CAP_PROP_FRAME_WIDTH/HEIGHT)` after opening the stream, or read the
SOF marker.

**`--quality` does nothing.**
Expected on the hardware encoder. Use `--bitrate`.

**The GUI cannot find the camera.**
Check the port: the service and the GUI both default to **8000**. Then check that
the daemon is listening on an interface the workstation can reach
(`--host 0.0.0.0`, not `127.0.0.1`), and that no firewall blocks it. A specific
address is always reachable by typing it into the camera box.

**`unknown control 'AeLocked' ignored` in the log.**
Expected. `AeLocked` is not a real libcamera control.

**Restarting the camera renumbers `/dev/videoN`.**
Anything that cached a device node path must re-discover it.

## Tests

The suite runs without a Pi. A duck-typed `picamera2`/`libcamera` stand-in is
installed into `sys.modules` before the service is imported, so the tests
exercise real service logic — including the `sensor_modes` bug above, which has a
dedicated regression test that fails if the fix is removed.

```bash
cd PiCameraService
python3 -m unittest discover -s . -t .
```

One test asserts that the default port here matches the GUI client's, so the two
cannot drift apart unnoticed.

The workstation-side bridge has its own offline suite. It fakes sysfs, so it can
run anywhere and needs neither `v4l2loopback` nor a real camera:

```bash
cd PiCameraService/bridge
python3 tests/test_bridge.py
```

Its central assertion is the safety property: no combination of arguments, and no
sysfs layout, may lead the bridge to write into a real camera.
