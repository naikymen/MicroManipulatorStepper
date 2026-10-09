# Raspberry Pi HQ Camera Microscope — Setup and Design Notes

## Purpose

This document summarizes the work completed so far toward building a microscope around the **Raspberry Pi HQ Camera (Sony IMX477)** using a **40× finite-conjugate RMS microscope objective**, with a Raspberry Pi Zero 2 W as the camera controller / intermediary to a Linux workstation.

It records the optical design, hardware setup, camera-service options, and
validation results for the Raspberry Pi HQ Camera microscope.

---

# 1. Optical concept under investigation

The target optical configuration is based on the OpenFlexure high-resolution optics approach.

Relevant reference pages supplied by the user:

- IO Rodeo 40× Plan Objective:
  https://iorodeo.com/products/openflexure-40x-objective
- OpenFlexure high-resolution optics assembly:
  https://build.openflexure.org/openflexure-microscope/v7.0.0-beta1/high_res_optics_module.html
- AmScope PA40X-V300:
  https://amscope.com/products/pa40x-v300?variant=40347660943535

The objective under consideration is a conventional finite biological microscope objective, approximately:

- 40×
- RMS thread
- 160 mm finite tube length
- 0.17 mm cover-glass correction
- Plan achromatic
- NA ≈ 0.65

The OpenFlexure high-resolution optics design uses this type of finite objective together with an approximately:

- 12.7 mm diameter achromatic doublet
- 50 mm focal length

The achromat acts as a reducing / relay optic so the objective image can be delivered over a much shorter mechanical distance than the nominal ~160 mm tube length and matched to a small camera sensor.

The Raspberry Pi HQ Camera has a much larger sensor than the Pi Camera V2:

- Sensor: Sony IMX477
- Resolution: 4056 × 3040
- Diagonal: ~7.9 mm
- Bayer CFA
- 12-bit sensor modes available

Because the IMX477 is larger than the sensor originally used by OpenFlexure, the 50 mm relay lens may not illuminate / correct the entire HQ sensor equally well. However, existing experimental OpenFlexure HQ-camera work shows that retaining the existing 50 mm relay arrangement is a practical option, with cropping of unusable corners if necessary.

---

# 2. Existing projects relevant to a Pi HQ microscope

The most relevant implementation found was experimental Raspberry Pi HQ Camera support in OpenFlexure.

## 2.1 OpenFlexure HQ-camera work

Main OpenFlexure project:

https://gitlab.com/openflexure/openflexure-microscope

Relevant experimental HQ-camera work discussed previously includes branches / merge requests designed to adapt the high-resolution RMS objective optics to the Pi HQ camera.

Useful OpenFlexure discussion threads previously identified:

- HQ Camera support / builds:
  https://openflexure.discourse.group/t/openflexure-with-hq-camera/2263
- Older HQ camera support discussion:
  https://openflexure.discourse.group/t/support-for-hq-pi-camera/168
- HQ sensor / optical configuration discussion:
  https://openflexure.discourse.group/t/optics-configuration-for-a-7-9-mm-diagonal-sensor/1513
- Tube-lens investigation using Pi HQ:
  https://openflexure.discourse.group/t/investigating-tube-lenses-for-infinity-optics-rpi-hq-camera-and-a-simple-upright-microscope/1476
- Later HQ build instructions / discussion:
  https://openflexure.discourse.group/t/using-a-raspberry-pi-hq-camera-build-instructions/2468

Important practical conclusion:

**The OpenFlexure experimental HQ-camera optics module is the closest existing open-source implementation to the intended microscope.**

The most relevant concept is:

```text
sample
  ↓
40× finite RMS objective
  ↓
~50 mm focal-length achromatic doublet
  ↓
Raspberry Pi HQ Camera / IMX477
```

A second experimental path tested direct finite-objective imaging without the 50 mm relay lens, using a much longer optical tube. This is mechanically simpler in optical principle but less compact and was not judged to be as attractive for the OpenFlexure architecture.

---

# 3. Camera-computer architecture

The preferred architecture is:

```text
Raspberry Pi HQ Camera
        │
        │ CSI-2
        ▼
Raspberry Pi Zero 2 W
        │
        │ libcamera / rpicam / Picamera2
        │
        │ USB OTG or network
        ▼
Linux workstation
```

The Pi Zero 2 W should remain the camera controller.

It should handle:

- sensor-mode selection
- exposure
- analogue gain
- raw Bayer capture
- still capture
- preview stream
- per-frame metadata
- optional DNG generation

The Linux workstation should ideally communicate with the Pi through:

- USB Ethernet gadget mode, or
- normal Ethernet/Wi-Fi during development

For microscopy / quantitative imaging, **do not use UVC as the primary acquisition interface**.

UVC can be added later to expose a conventional `/dev/videoN` preview device, but it is not ideal for:

- full IMX477 sensor control
- 12-bit Bayer transfer
- arbitrary sensor modes
- complete metadata
- scientific acquisition

Recommended final architecture:

```text
HQ Camera
   │
   ▼
Pi Zero 2 W
Picamera2 / libcamera daemon
   │
   ├── low-resolution preview
   ├── full-resolution Bayer acquisition
   ├── exposure / gain controls
   └── metadata
   │
   ▼
USB Ethernet
   │
   ▼
Linux workstation
```

If ordinary Linux applications need `/dev/videoN`, use `v4l2loopback` on the workstation and feed it the preview stream separately.

---

# 4. Hardware actually tested

The user is using:

- Raspberry Pi Zero 2 W
- Raspberry Pi HQ Camera
- Sony IMX477 sensor

Hostname during testing:

```text
pizero2w-olivos
```

The HQ camera is correctly detected by the current Raspberry Pi camera stack.

---

# 5. Verified camera detection

Command used:

```bash
rpicam-hello --list-cameras
```

Observed camera:

```text
0 : imx477 [4056x3040 12-bit RGGB]
```

Camera path:

```text
/base/soc/i2c0mux/i2c@1/imx477@1a
```

Observed sensor modes:

## 10-bit modes

```text
1332x990   @ 120.50 fps
2028x1080  @ 74.74 fps
2028x1520  @ 53.77 fps
4056x2160  @ 19.58 fps
4056x3040  @ 14.00 fps
```

## 12-bit modes

```text
1332x990   @ 101.68 fps
2028x1080  @ 62.81 fps
2028x1520  @ 45.19 fps
4056x2160  @ 16.39 fps
4056x3040  @ 11.72 fps
```

## 8-bit modes

```text
1332x990   @ 147.91 fps
2028x1080  @ 92.27 fps
2028x1520  @ 66.38 fps
4056x2160  @ 24.32 fps
4056x3040  @ 17.39 fps
```

The actual maximum rates reported by this installed driver / firmware should be considered authoritative for this device.

---

# 6. Verified still-image capture

Basic capture command:

```bash
rpicam-still -n -o test.jpg
```

This completed successfully.

The camera pipeline selected:

```text
4056x3040-SBGGR12_1X12/RAW
```

with a processed YUV output stream for JPEG encoding.

---

# 7. Verified RAW / DNG capture

Command used:

```bash
rpicam-still -n -r -o image.jpg
```

This successfully created:

```text
image.jpg
image.dng
```

Observed file sizes:

```text
image.dng  ~24.8 MB
image.jpg  ~1.28 MB
```

The capture reported:

```text
Bayer format is BGGR-12
```

Therefore full-resolution 12-bit Bayer acquisition through the normal Raspberry Pi camera stack is working.

---

# 8. Verified deterministic manual capture

The following command was tested successfully:

```bash
rpicam-still \
    -n \
    --immediate \
    --shutter 10000 \
    --gain 1 \
    --awbgains 1,1 \
    --denoise off \
    -r \
    -o microscope.jpg
```

Interpretation:

- `--shutter 10000`
  - exposure time = 10,000 µs = 10 ms
- `--gain 1`
  - analogue gain = 1×
- `--awbgains 1,1`
  - fixed red/blue white-balance gains
- `--denoise off`
  - disables normal denoising in the processed pipeline
- `-r`
  - also captures RAW / DNG
- `--immediate`
  - captures immediately rather than waiting for automatic algorithms to converge
- `-n`
  - no preview window

The command completed successfully and used:

```text
4056x3040-SBGGR12_1X12/RAW
```

The raw Bayer format was again reported as:

```text
BGGR-12
```

This is a good baseline command for quantitative microscopy.

---

# 9. On-sensor defective-pixel correction

The IMX477 driver exposes a defective-pixel-correction switch.

The following command was tested successfully:

```bash
echo 0 | sudo tee /sys/module/imx477/parameters/dpc_enable
```

Returned:

```text
0
```

This disables the IMX477 driver's defective-pixel correction.

This can be desirable for scientific imaging because it reduces hidden modification of sensor pixel values.

For quantitative acquisition, the preferred approach is likely:

```text
DPC disabled
+
dark-frame calibration
+
flat-field calibration
+
manual exposure
+
manual analogue gain
```

Whether DPC should remain disabled permanently should be evaluated experimentally; for normal visual imaging, enabling DPC usually produces cleaner images.

---

# 10. Recommended software interface

The preferred API is **Picamera2**, not UVC.

Picamera2 allows access to:

- sensor modes
- full-resolution raw stream
- processed main stream
- low-resolution preview stream
- exposure
- analogue gain
- frame duration
- white balance
- scaler crop / ROI
- frame metadata

Useful introspection calls:

```python
print(camera.sensor_modes)
print(camera.camera_controls)
```

This avoids hard-coding control limits.

---

# 11. Recommended Picamera2 configuration

A microscope application should ideally operate two simultaneous streams:

```text
low-resolution processed preview
+
full-resolution raw Bayer stream
```

Conceptually:

```python
from picamera2 import Picamera2

cam = Picamera2()

config = cam.create_still_configuration(
    main={"size": (1024, 768), "format": "RGB888"},
    raw={"size": cam.sensor_resolution},
)

cam.configure(config)

cam.set_controls({
    "ExposureTime": 10000,
    "AnalogueGain": 1.0,
    "ColourGains": (1.0, 1.0)
})

cam.start()
```

Then raw data can be obtained as a NumPy array:

```python
raw = cam.capture_array("raw")
```

For a real implementation, inspect the actual raw stream format first:

```python
print(cam.stream_configuration("raw"))
```

Do not assume raw packing or dtype without checking the returned configuration.

---

# 12. Scientific-imaging considerations

For microscopy where intensity values may be measured quantitatively, avoid relying on processed JPEG/YUV output.

Prefer:

```text
RAW Bayer
```

or:

```text
DNG
```

Important variables to fix or record:

- exposure time
- analogue gain
- sensor mode
- bit depth
- Bayer pattern
- ROI / crop
- illumination intensity
- black level
- sensor temperature if relevant
- DPC state

Avoid or disable, where possible:

- automatic exposure
- automatic gain
- automatic white balance
- denoising
- sharpening
- contrast manipulation
- saturation manipulation
- gamma / tone mapping for quantitative analysis

Processed preview can still use the ISP; it simply should not be used as the measurement image.

---

# 13. DNG versus raw arrays

## DNG

Advantages:

- preserves Bayer data
- embeds metadata
- convenient archival format
- easier to reopen correctly later
- useful for single frames / experiments

Current working command:

```bash
rpicam-still -n -r -o image.jpg
```

## Direct raw arrays through Picamera2

Advantages:

- ideal for software-controlled acquisition
- avoids writing intermediary files
- suitable for live analysis
- suitable for calibration
- can be sent directly to workstation software

This should be the preferred route for the final microscope application.

---

# 14. UVC / `/dev/videoN` option

The Raspberry Pi Zero 2 W can operate as a USB OTG gadget.

It is possible to expose the camera as a USB UVC webcam and make it appear on Linux as:

```text
/dev/videoN
```

However, this should be considered a compatibility / preview interface.

Limitations relative to Picamera2:

- generic UVC controls do not map cleanly to every libcamera control
- raw 12-bit Bayer is not normally exposed in a convenient way
- metadata handling is poorer
- high-resolution raw bandwidth becomes problematic
- UVC emphasizes webcam/video use rather than scientific acquisition

Recommended strategy:

```text
Picamera2 remains authoritative
```

and optionally:

```text
preview → UVC
```

or:

```text
preview over network → v4l2loopback on workstation
```

---

# 15. Workstation-side virtual camera option

On an Arch / EndeavourOS workstation, a preview stream can later be exposed as a local Linux camera using:

```bash
sudo pacman -S v4l2loopback-dkms v4l-utils
```

This would allow generic applications to see a device such as:

```text
/dev/video10
```

while scientific camera control still happens through a separate Picamera2 interface.

This separation is preferable to trying to push all scientific controls through UVC.

---

# 16. Recommended next implementation step

The next practical task is to implement a small **camera service on the Pi Zero 2 W**.

Suggested capabilities:

```text
GET_SENSOR_MODES
GET_CONTROLS
SET_EXPOSURE
SET_GAIN
SET_ROI
SET_SENSOR_MODE
START_PREVIEW
STOP_PREVIEW
CAPTURE_RAW
CAPTURE_DNG
GET_METADATA
```

Transport should initially be simple and reliable.

Recommended development sequence:

1. Use normal network access first.
2. Build the Picamera2 control daemon.
3. Confirm preview + raw simultaneous capture.
4. Add metadata transfer.
5. Add USB Ethernet gadget mode.
6. Optionally add a local `v4l2loopback` preview device on the workstation.
7. Only add UVC gadget mode if compatibility with generic webcam software is genuinely required.

---

# 17. Bandwidth considerations

Full-resolution raw frames are large.

The tested DNG was approximately:

```text
24.8 MB
```

A raw stream at full sensor resolution and high frame rate is therefore impractical to push continuously through a constrained Pi Zero 2 W / USB 2.0 pipeline without careful packing and bandwidth management.

Recommended microscope behavior:

```text
continuous low-resolution preview
+
full-resolution RAW capture on demand
```

For some experiments, a lower-resolution raw sensor mode may also be useful.

The 2028×1520 12-bit mode is particularly attractive because it allows much higher frame rates than full 4056×3040 acquisition, but the exact sensor readout/binning behavior should be verified before using it for quantitative measurements.

---

# 18. Current known-good state

The following have been confirmed experimentally:

- IMX477 detected correctly
- sensor mode enumeration works
- full-resolution still capture works
- 12-bit RAW acquisition works
- DNG output works
- manual shutter works
- manual gain works
- fixed AWB gains work
- processed denoising can be disabled
- sensor DPC can be disabled
- Pi Zero 2 W is suitable as the intermediary camera computer

No errors were observed in the supplied command outputs.

---

# 19. Important distinction: Bayer format naming

Camera enumeration reports:

```text
12-bit RGGB
```

while the selected libcamera / Unicam stream reports:

```text
SBGGR12
```

and `rpicam-still` reports:

```text
Bayer format is BGGR-12
```

Do not assume this is necessarily a contradiction.

Bayer-order naming can depend on:

- physical sensor mosaic orientation
- image transforms
- driver representation
- stream coordinate conventions

The actual raw stream metadata / DNG tags should be treated as authoritative for decoding.

When developing custom raw processing, explicitly inspect:

- stream format
- DNG CFA tags
- libcamera metadata

before hard-coding a Bayer order.

---

# 20. Key references

Raspberry Pi camera software documentation:

https://www.raspberrypi.com/documentation/computers/camera_software.html

Picamera2 source:

https://github.com/raspberrypi/picamera2

Picamera2 manual:

https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf

Raspberry Pi HQ Camera product documentation:

https://www.raspberrypi.com/products/raspberry-pi-high-quality-camera/

OpenFlexure microscope:

https://openflexure.org/projects/microscope/

OpenFlexure microscope source:

https://gitlab.com/openflexure/openflexure-microscope

OpenFlexure high-resolution optics module:

https://build.openflexure.org/openflexure-microscope/v7.0.0-beta1/high_res_optics_module.html

IO Rodeo OpenFlexure 40× objective:

https://iorodeo.com/products/openflexure-40x-objective

AmScope PA40X-V300:

https://amscope.com/products/pa40x-v300?variant=40347660943535

---

# 21. Recommended design philosophy going forward

Treat the Pi HQ camera as a **scientific camera head**, not as a webcam.

The Pi Zero 2 W should perform low-level sensor control.

The workstation should perform:

- GUI
- image visualization
- acquisition sequencing
- calibration
- measurement
- image storage
- optional analysis

Maintain two distinct data paths:

```text
PREVIEW PATH
IMX477 → ISP → reduced RGB/YUV → workstation display
```

and:

```text
MEASUREMENT PATH
IMX477 → raw Bayer → workstation / archival storage
```

This architecture preserves convenience without sacrificing access to the original sensor data.
