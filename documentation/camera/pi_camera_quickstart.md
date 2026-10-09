# Raspberry Pi HQ Camera — Step-by-Step Setup

A linear walkthrough, from a blank microSD card to the HQ camera appearing in the
GUI's camera dropdown: flash the card, confirm the sensor, install the service,
start it, then connect from a Linux workstation.

Each stage ends with a **checkpoint** — a command whose output tells you whether
to continue or stop and fix something. Do not skip them; every problem this
guide's [Troubleshooting](#troubleshooting) section lists is cheap to catch at
the checkpoint and expensive to debug later.

This document is the *how*. For the API reference, option list and performance
numbers see [`software/PiCameraService/README.md`](../../software/PiCameraService/README.md).
For the optics, the measurement rationale and the design decisions see
[the design notes](pi_zero_hq_microscope_setup.md).

**Verified against**

| Component | Version |
| --- | --- |
| Board | Raspberry Pi Zero 2 W Rev 1.0, 415 MB RAM |
| OS | Raspberry Pi OS / Debian 13 (trixie), arm64, kernel 6.18.50+rpt-rpi-v8 |
| Python (Pi) | 3.13.5 |
| picamera2 / libcamera | 0.3.37-1 / 0.7.2+rpt20260817-1, both from apt |
| Sensor | IMX477 (HQ Camera), auto-detected |
| Workstation | EndeavourOS, Python 3.14.7, OpenCV 5.0.0, PySide6 6.11.2, ffmpeg 9.0.2 |

---

## Contents

- [What you need](#what-you-need)
- [Part A — Set up the Pi](#part-a--set-up-the-pi)
  - [A1. Flash the card](#a1-flash-the-card)
  - [A2. First boot and confirm the sensor](#a2-first-boot-and-confirm-the-sensor)
  - [A3. Install the camera stack](#a3-install-the-camera-stack)
  - [A4. Copy the service to the Pi](#a4-copy-the-service-to-the-pi)
  - [A5. Sanity check and first run](#a5-sanity-check-and-first-run)
  - [A6. Start it automatically (optional)](#a6-start-it-automatically-optional)
- [Part B — Set up the workstation](#part-b--set-up-the-workstation)
  - [B1. Install the GUI's dependencies](#b1-install-the-guis-dependencies)
  - [B2. Work out the Pi's address](#b2-work-out-the-pis-address)
  - [B3. Connect from the GUI](#b3-connect-from-the-gui)
  - [B4. Check it from the shell (optional)](#b4-check-it-from-the-shell-optional)
  - [B5. Expose it as a video device (optional)](#b5-expose-it-as-a-video-device-optional)
- [Part C — USB gadget mode (optional, not verified)](#part-c--usb-gadget-mode-optional-not-verified)
- [Troubleshooting](#troubleshooting)
- [Where to go next](#where-to-go-next)

---

## What you need

- **Raspberry Pi Zero 2 W** — the reference board. A Zero 2 W has ~229 MB of RAM
  available after boot, which is enough for a 1024x768 preview but not much more.
- **Raspberry Pi HQ Camera** (Sony IMX477).
- **A Pi Zero compatible camera cable.** This is the most common thing to get
  wrong. The HQ Camera ships with a standard 15-pin cable, but a Pi Zero has a
  **narrower 22-pin** camera connector. You need the 22-pin Zero cable (or a
  15-to-22 adapter), not the cable in the box.
- **A C-mount to RMS adapter** for the 40x finite-conjugate objective, and the
  objective itself.
- **microSD card**, 8 GB or more.
- **A way to reach the Pi** — Wi-Fi on the same network as the workstation, or a
  micro-USB data cable for [gadget mode](#part-c--usb-gadget-mode-optional-not-verified).
  Note the Zero's two micro-USB ports: the one labelled **PWR IN** is power only,
  the other is the data/OTG port.

An Ethernet-cable-to-the-router setup is not available on a Zero 2 W; there is
no Ethernet port. Use Wi-Fi or gadget mode.

---

## Part A — Set up the Pi

### A1. Flash the card

Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) on the
workstation. The reference build was made with Imager's built-in settings
editor, which is why `/boot/firmware/cmdline.txt` on this Pi carries a
`ds=nocloud;i=rpi-imager-...` marker.

1. **Choose device** → Raspberry Pi Zero 2 W.
2. **Choose OS** → Raspberry Pi OS (64-bit), the current Debian-based release.
3. **Choose storage** → your microSD card.
4. Click **Next**, then **Edit Settings** when Imager offers to customise. Set:

   | Setting | What to put | Why |
   | --- | --- | --- |
   | Hostname | Anything, e.g. `pizero2w-olivos` | Only used for mDNS and the shell prompt. It does **not** have to be `raspberrypi`. |
   | Username / password | `pi` and a password you will remember | The GUI does not need it; you do, for `ssh` and `sudo`. |
   | Wireless LAN | SSID, password, **and country code** | The country code is mandatory — Wi-Fi is blocked without a regulatory domain. |
   | Locale | Timezone and keyboard | Small quality-of-life; sets the capture timestamps. |
   | SSH | **Enable**, "Use password authentication" | You will drive the Pi over SSH from the workstation. |

5. **Write** the card, then move it to the Pi and power it up.

> **Why the hostname does not matter.** Nothing in this project assumes the Pi is
> called `raspberrypi`, and its IP address will change between networks. The GUI
> discovers the camera by the address you last used or one you type in, so pick
> any hostname you like.

**Checkpoint:** the card writes without error and Imager reports the verification
passed.

### A2. First boot and confirm the sensor

Wait about 45 seconds for the first boot (it resizes the filesystem and can
reboot once), then SSH in. Replace the address below with your Pi's.

```bash
ssh pi@<pi-address>
```

Confirm the camera is detected **before installing anything**:

```bash
rpicam-hello --list-cameras
```

There is no `libcamera-hello` on current Raspberry Pi OS — the tools were renamed
to `rpicam-*`, and `vcgencmd get_camera` is a stale legacy command that reports
`detected=0` even on a perfectly working camera. `rpicam-hello --list-cameras` is
the one that matters.

**Checkpoint:** you should see the sensor named explicitly:

```text
Available cameras
-----------------
0 : imx477 [4056x3040 12-bit] (/base/soc/i2c0mux/i2c@1/imx477@1a)
    Modes: 'SBGGR10_CSI2P' : 1332x990 [30.00 fps - ...]
           ...
```

If the list is **empty**, stop here — this is a hardware problem and no amount of
software setup will fix it. See
[The camera is not detected](#the-camera-is-not-detected).

> **No `dtoverlay` line is needed.** `/boot/firmware/config.txt` auto-detects the
> sensor; the reference Pi's `[all]` block is empty and the IMX477 is found
> anyway. Only add `dtoverlay=imx477` if auto-detection genuinely fails.

### A3. Install the camera stack

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-libcamera
```

That is the whole dependency list. On Raspberry Pi OS the camera stack is part
of the base image.

> **Never `pip install picamera2`.** The PyPI wheels cannot talk to the VC4/PiSP
> hardware pipelines, so you lose the hardware MJPEG encoder and full sensor
> resolution while everything still appears to install correctly.

`zeroconf` is optional and only used for mDNS announcement and `--scan`:

```bash
sudo apt install -y python3-zeroconf   # optional
```

Skip it unless you intend to rely on mDNS. As
[B2](#b2-work-out-the-pis-address) explains, you should not rely on it.

**Checkpoint:**

```bash
python3 -c "import picamera2, libcamera; print(libcamera.__version__)"
```

`picamera2` has no `__version__` attribute, so ask `libcamera` instead or check
`dpkg -l | grep picamera2`. A clean `0.7.2+rpt20260817-1` (or similar) means the
stack is importable.

### A4. Copy the service to the Pi

The Pi has **no `git` installed** by default (verified: `git` is absent,
`rsync` and `scp` are present), so copy the directory rather than cloning. From
the root of this repository on the workstation:

```bash
rsync -av --exclude '__pycache__' --exclude 'captures' \
  software/PiCameraService/ pi@<pi-address>:/home/pi/PiCameraService/
```

`scp -r software/PiCameraService pi@<pi-address>:/home/pi/` works too, but `rsync`
is better for updates because it only sends what changed.

The reference deployment lives at `/home/pi/PiCameraService`. The `systemd` unit
and the README both assume that path, so keep it unless you are prepared to edit
them.

**Checkpoint** — on the Pi:

```bash
ls /home/pi/PiCameraService
```

You should see `picamera_daemon.py`, `picamera_service.py`, `requirements.txt`,
`README.md`, `systemd/` and `tests/`.

If you prefer a virtual environment, it must be created with
`--system-site-packages` so the apt-installed `picamera2` stays visible:

```bash
cd /home/pi/PiCameraService
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
```

`requirements.txt` lists only optional extras — there is nothing required in it.

### A5. Sanity check and first run

On the Pi, with **nothing else using the camera**:

```bash
cd /home/pi/PiCameraService
python3 picamera_daemon.py --list-controls
```

**Checkpoint:** this prints a long list of control names with their ranges and
exits. This is the single most valuable command in the whole process: it proves
libcamera can open the sensor *and* that the daemon's control layer can talk to
it.

> **This command fails if a daemon is already running**, with
> `Unable to set controls: Device or resource busy`. That is not a bug — only one
> process may own the sensor. Stop the running daemon first. Note that
> `rpicam-hello --list-cameras` still *works* in that state, because enumerating
> cameras does not open the controls, so a passing `--list-cameras` does not mean
> the camera is free.

Now start it:

```bash
# Foreground — watch the log, Ctrl-C to stop. Use this the first time.
python3 picamera_daemon.py

# Detached, for when you are done watching it
setsid nohup python3 picamera_daemon.py --log-level info > /tmp/daemon.log 2>&1 &
```

It listens on `0.0.0.0:8000` and streams 1024x768 at 25 fps by default. The full
option list is in the [README](../../software/PiCameraService/README.md#run-it);
the ones you are most likely to change are `--width`/`--height`, `--fps`,
`--capture-dir` and `--bitrate`.

`--log-level` values are **lowercase**; passing `INFO` is rejected by argparse.

**Checkpoint** — from the Pi itself, three escalating tests:

```bash
curl -s http://127.0.0.1:8000/status     # {"ok": true, "camera_open": true, ...}
curl -s http://127.0.0.1:8000/info       # full identity and control dump
curl -s -o /tmp/test.jpg http://127.0.0.1:8000/snapshot.jpg && file /tmp/test.jpg
```

That last line should report a JPEG of the configured size. If `/status` answers
but `camera_open` is `false`, the daemon started without the sensor and you should
return to [A2](#a2-first-boot-and-confirm-the-sensor).

### A6. Start it automatically (optional)

Only worth doing once [A5](#a5-sanity-check-and-first-run) is consistently clean —
a unit that restarts into a failing camera produces a log you have to read anyway.

```bash
sudo cp systemd/picamera.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now picamera
systemctl status picamera
journalctl -u picamera -f
```

The unit assumes `/home/pi/PiCameraService`; edit `WorkingDirectory` and
`ExecStart` if you installed it elsewhere. It waits for `network-online.target`
and restarts on failure.

**Checkpoint:**

```bash
systemctl is-enabled picamera   # enabled
systemctl is-active picamera    # active
```

> **Do not run the unit and a manual daemon at the same time.** The second one to
> start will find the camera busy. `systemctl is-active picamera` and
> `pgrep -af picamera_daemon` should never both show a process.

---

## Part B — Set up the workstation

### B1. Install the GUI's dependencies

The camera client needs the GUI's normal dependencies; nothing extra. From the
`software/OpenMicroManipulatorGUI` directory, either use the project's script:

```bash
./setup.sh          # creates .venv and installs requirements.txt
```

or do it by hand:

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

`requirements.txt` brings in `pyside6`, `opencv-python` and `numpy`. The camera
path uses `opencv-python` (via `cv2.VideoCapture`) and `pyside6` only; `pypylon`
is needed solely for Basler cameras and can fail to install without Pylon
installed — it is safe to leave out if you have a Basler camera.

**Checkpoint:**

```bash
.venv/bin/python -c "import cv2, PySide6; print(cv2.__version__, PySide6.__version__)"
```

### B2. Work out the Pi's address

**Find the address, then type it in.** This is the short version of this section,
and it is the reliable one.

The GUI probes, in order of cost:

1. **The address you used last**, remembered in `QSettings` under
   `Connections/last_pi_camera_address`. In practice this is what does the work.
2. **The built-in defaults** in
   [`camera_pi.py`](../../software/OpenMicroManipulatorGUI/source/hardware/camera_pi.py):
   `raspberrypi.local`, `picam.local`, and a literal reference address
   (`192.168.1.39`, the reference build's Pi). Change that literal if you like,
   or leave it — an unreachable address costs one short probe timeout.
3. **Whatever you type into the camera box**, which accepts `host`, `host:port`,
   `http://host:port`, `http://host:port/info` and mDNS names alike.

There is deliberately no network scan: a camera that does not answer simply does
not appear, and a typed address that fails gives you a dialog naming the exact
address rather than an empty dropdown.

To get the address:

```bash
ssh pi@<pi-address> 'hostname -I'     # e.g. 192.168.1.39
```

> **Do not count on `.local` names.** They are the nicest thing to type and they
> failed in the environment this guide was verified in — `raspberrypi.local`,
> `picam.local` and the Pi's real `pizero2w-olivos.local` all timed out from the
> workstation, even though `avahi-daemon` was running and enabled on the Pi and
> mDNS was listening on the workstation. Many routers block multicast between
> wireless clients. mDNS is best-effort: if it works, enjoy it, and if it does
> not, use the IP address. Also note the built-in defaults will not match a Pi
> you named anything other than `raspberrypi` or `picam`.

The address **will** change between networks, and over gadget mode it is
different again (see [Part C](#part-c--usb-gadget-mode-optional-not-verified)).
That is expected, not a fault — type the current one.

**Checkpoint** — from the workstation:

```bash
curl -s http://<pi-address>:8000/status
```

`{"ok": true, ...}` means the network path is good and any remaining problem is
in the GUI, not the Pi.

### B3. Connect from the GUI

```bash
cd software/OpenMicroManipulatorGUI
.venv/bin/python source/main.py
```

or `./start_micro_manipulator_gui.sh`.

1. Click **Refresh** next to the camera dropdown. A reachable Pi appears as
   `Pi Camera <address>:8000 (<sensor>, <width>x<height>)`, for example
   `Pi Camera 192.168.1.39:8000 (imx477, 1024x768)`. If it does not appear,
   **that is fine** — the next step is the normal path, not a fallback.
2. **Type the address into the camera box** and click **Connect**. The box is
   editable precisely because a Pi camera service has no guaranteed address. It
   accepts bare `host`, `host:port` and full URLs.
3. The address is remembered, so the next launch offers it again.

**Checkpoint:** the live preview appears in the camera panel, and the exposure,
gain and white balance sliders are enabled rather than greyed out.

Reasonably fast failure is by design: an unreachable address fails in about 3
seconds and a refused connection in about 0.02 seconds, because the client passes
explicit open and read timeouts to OpenCV. Were it not for those, OpenCV would
block for its internal 30 seconds and freeze the GUI while switching cameras.

### B4. Check it from the shell (optional)

Worth doing once, because it separates "the network works" from "the GUI works",
and it is how you will script measurements later.

```bash
curl -s http://<pi-address>:8000/info | python3 -m json.tool | head -40
curl -s -o shot.jpg http://<pi-address>:8000/snapshot.jpg
curl -s -X POST http://<pi-address>:8000/control \
     -H 'Content-Type: application/json' \
     -d '{"AeEnable": false, "ExposureTime": 2000}'
```

The API is documented in full in the
[README](../../software/PiCameraService/README.md#http-api).

> **This is the path that matters for measurements.** The preview stream is
> lossy — it goes through the hardware JPEG encoder for smoothness, not accuracy
> — so anything you intend to quantify should come from `POST /capture`, which
> writes a full-resolution DNG on the Pi. See
> [Preview vs measurement](../../software/PiCameraService/README.md#preview-vs-measurement).

### B5. Expose it as a video device (optional)

If you want **other** software — not the GUI — to see the camera as an ordinary
webcam, relay the preview into a `v4l2loopback` device. This is entirely optional
and independent of both the GUI and the measurement path, and the guide for it
lives in the README:
[Expose the preview as a local video device](../../software/PiCameraService/README.md#expose-the-preview-as-a-local-video-device).

Because the video device is optional, skip this entirely unless you have a
concrete application that needs it — the HTTP API is the primary interface.

---

## Part C — USB gadget mode (optional, not verified)

> **Not verified.** Everything in Parts A and B was exercised on real hardware.
> This section was not: the reference Pi is on Wi-Fi, `dwc2` is not enabled on it,
> and no `usb0` interface exists. Treat the steps below as a starting point and
> expect to iterate. If you get it working, correct this section.

Gadget mode lets a single micro-USB cable carry both power and network between
the Pi and the workstation. It is useful on a network you do not control.

On the Pi, edit the boot configuration:

```bash
sudo nano /boot/firmware/config.txt
```

Add under `[all]`:

```text
dtoverlay=dwc2
```

Then edit `/boot/firmware/cmdline.txt` — **one single line, do not add a newline**
— and add `modules-load=dwc2,g_ether` after `rootwait`:

```text
console=serial0,115200 console=tty1 root=PARTUUID=... rootfstype=ext4 fsck.repair=yes rootwait modules-load=dwc2,g_ether
```

Reboot, then connect the cable to the **data** port (not **PWR IN**).

### What address results

**Do not expect `192.168.7.2`.** That address appears in the
[README](../../software/PiCameraService/README.md#finding-the-pi-from-the-workstation)
and it is a BeagleBone convention, not a Raspberry Pi one. `g_ether` does not
assign addresses, so with no further configuration the realistic outcomes are:

- **Link-local.** Both ends self-assign `169.254.x.x` and lose the ability to
  choose which. Works, but you have to read the address off the Pi each time.
- **mDNS.** Usually the intended mechanism, but see
  [B2](#b2-work-out-the-pis-address) — mDNS failed in the verified environment and
  is not something to depend on.
- **A static address you assign**, which is the only option that gives a
  predictable result. Configure a static IPv4 address on the Pi's `usb0` in
  NetworkManager, and a matching one on the workstation's `usb0`/`enx...`
  interface in the same subnet.

The predictable-address option is the one to aim for; then you type that address
into the camera box exactly as in [B3](#b3-connect-from-the-gui).

---

## Troubleshooting

### The camera is not detected

`rpicam-hello --list-cameras` prints an empty list. Software cannot fix this.
Check, in order:

1. **The cable.** The Zero needs the 22-pin cable. The 15-pin cable the HQ Camera
   ships with **will not fit** the Zero's connector, and forcing it damages both.
2. **The connector seating.** The ribbon must be fully inserted and the locking
   tab closed, on both the Pi and the camera board.
3. **The contact orientation.** The exposed metal contacts face a specific way on
   each end — toward the board on the Pi, toward the sensor board on the camera.
4. **The power.** An underpowered supply causes the sensor to fail to probe.

If `dtoverlay=imx477` is already present and the camera is still missing, remove
it and let auto-detect run.

### `Unable to set controls: Device or resource busy`

Another process owns the camera. Stop the daemon (`systemctl stop picamera`, or
kill the process) and retry. Only one process at a time may use the sensor — this
includes `rpicam-hello`, `rpicam-still` and `--list-controls`.

### `vcgencmd get_camera` says `detected=0`

Ignore it. That command is legacy and reports `detected=0` on working
configurations. Use `rpicam-hello --list-cameras`.

### `libcamera-hello: command not found`

The tools were renamed. Use `rpicam-hello`, `rpicam-still`, etc.

### The preview is 640x480 no matter what I ask for

A known fixed bug: enumerating `sensor_modes` calls `configure()` once per mode,
and every probe reset the main stream to 640x480. It is fixed in the current
daemon. If you are running an older copy, update `picamera_daemon.py`.

Note that while it is broken, `/info` is misleading — it reports the size you
asked for, not the size being produced. Measure the JPEG SOF marker instead.

### The GUI does not list the camera

By design the GUI does not scan the network and does not show a camera that does
not answer. Work through:

1. Is the daemon running and bound to `0.0.0.0`, not `127.0.0.1`?
2. Does `curl http://<pi-address>:8000/status` work **from the workstation**? If
   not, it is a network problem, and the GUI cannot help.
3. Are the ports the same? Both default to **8000**.
4. Then just **type the address into the camera box** — do not wait for it to
   appear.

### Captures are being written somewhere unexpected

`--capture-dir` defaults to `captures` **relative to the current directory**, so a
manually-run daemon writes to `./captures` while the systemd unit passes an
absolute `/home/pi/PiCameraService/captures`. Check the `--capture-dir` in
whichever one you started.

### mDNS names do not resolve

Expected in many networks, and not fixable from this project. Use the IP address.

---

## Where to go next

- [`software/PiCameraService/README.md`](../../software/PiCameraService/README.md)
  — the service reference: HTTP API, every option, controls, performance
  measurements, the optional video-device bridge, and the test suites.
- [Design notes](pi_zero_hq_microscope_setup.md) — optics, sensor modes, DNG
  handling, defective-pixel correction, bandwidth, and *why* the preview and the
  measurement paths are deliberately separate.
