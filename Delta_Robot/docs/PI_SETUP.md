# Raspberry Pi setup (Phase 2): ToF camera + remote viewing

Everything here is run **by you, on the Pi** (over SSH). The assistant that wrote this could not reach the Pi, so
**none of it has been run yet** — treat it as a checklist, and paste the outputs back so the next steps can be based on facts.

Commands marked **[sudo]** change the system. Read them first; none is required for steps 1–3 except where noted.

---

## UPDATE: what the Pi actually found (read this first)

The Pi's own inspection showed **no ToF camera**. The only camera is a **USB stereo camera**: `lsusb` ID `32e4:9282`
"3D Global Shutter Camera", a standard UVC device (`/dev/video0`) with two ~1 MP global-shutter eyes side by side
(modes up to 2560x800, i.e. 1280x800 per eye). No CSI camera is present. The Pi runs Debian 13 (trixie), Python 3.13.
Unless you do own a ToF camera that is simply not connected or powered, **the stereo camera is the path forward**,
and the ToF steps below (sections 2–4) can be skipped.

### Stereo path — step by step

1. **On your PC** (PowerShell, normal window), get the latest files onto the Pi:
   ```
   cd $HOME\Irrigation-Robot-Code
   git pull
   scp -r Delta_Robot deltarobot@192.168.137.128:~/
   ```
2. **On the Pi** (ssh in). First fix Windows line endings (files that went through Windows Git/`scp` can have `CRLF`,
   which makes shell scripts fail with `set: Illegal option -`; the repo now has a `.gitattributes` to prevent it, but files
   copied earlier still need this once):
   ```
   find ~/Delta_Robot -type f \( -name "*.sh" -o -name "*.service" -o -name "*.yaml" -o -name "*.py" -o -name "*.ino" \) -exec sed -i 's/\r$//' {} +
   ```
   Then create the environment and check the camera:
   ```
   sh ~/Delta_Robot/pi/tof/setup_venv.sh        # venv + numpy/OpenCV (the ToF SDK it also installs is simply unused)
   source ~/Delta_Robot/venv/bin/activate
   cd ~/Delta_Robot/pi
   python camera_check.py --only 1280x480       # quick: three frame rates for one size
   python camera_check.py                       # full table (about 1-2 minutes)
   ```
   Paste the table. The important columns: **corrupt** (should be 0) and **real fps**. If `corrupt` is not 0 at a given rate, that
   combination delivers broken pictures. Rows with `BROKEN FRAMES` are not usable. Also run these read-only checks and paste them:
   ```
   lsusb -t                       # shows the camera's USB speed: 480M = USB 2.0, 5000M = USB 3.0
   vcgencmd get_throttled         # 0x0 = fine; anything else can mean under-voltage (weak power supply)
   dmesg | grep -iE "usb|uvc|under-voltage" | tail -20
   ```
   Things worth trying if frames are broken at every rate: a different USB port (the blue USB 3.0 ones), a shorter or better USB
   cable, and the official Pi 5 power supply (27 W). The camera draws up to 500 mA. To look at the pictures, run on your PC: `scp deltarobot@192.168.137.128:Delta_Robot/pi/camera_check_out/*.png .`
3. **Test the depth pipeline without YOLO** (no big download yet):
   ```
   python pi_camera_server.py --no-yolo
   ```
   Open `http://192.168.137.128:5000` in a browser on the PC: left = camera, right = depth (it says UNCALIBRATED until step 4).
4. **Calibrate** (needs the printed checkerboard; steps in `pi/README.md`, section 5). Do it at the final mode (1280x480).
5. **YOLO** comes last: `pip install ultralytics` (large) and a strawberry model; with the stock model the system
   correctly refuses to pick anything.

Nothing in the stereo path needs `sudo`, a boot-file edit or a reboot.

---

## 1. Tell me what is plugged in (read-only, no sudo)

```
cat /proc/device-tree/model; grep PRETTY /etc/os-release; python3 --version
lsusb
v4l2-ctl --list-devices
ls /dev/video* /dev/media* 2>&1
rpicam-hello --list-cameras 2>&1 | head -20
dmesg | grep -iE "arducam|pivariety|tof|uvc" | tail -20
grep -E "dtoverlay|camera_auto_detect" /boot/firmware/config.txt
```

How to read it:

* If `lsusb` shows an Arducam/ToF device → the camera is on **USB**. No boot-file change is needed.
* If nothing shows in `lsusb` and the camera is on a flat ribbon cable → **CSI** (needs a boot overlay, step 4).
* The 1.0-megapixel USB camera (model no. 202511028880000100) will show up in `lsusb` / `v4l2-ctl` as a normal UVC camera.
* The ToF camera needs its **own 5 V / 2 A supply** (Arducam README). A weak supply gives "open failed" or random dropouts.

## 2. Install the ToF SDK in a virtual environment (no sudo)

```
sh ~/Delta_Robot/pi/tof/setup_venv.sh
source ~/Delta_Robot/venv/bin/activate
```

This uses the vendor's self-contained Python wheel. **Do not run the vendor's `Install_dependencies.sh`**: it
edits `/boot/firmware/config.txt`, sets `camera_auto_detect=0` (which stops official Pi cameras from being detected),
adds a third-party apt repository, and installs with `sudo pip` into system Python.

## 3. Test the camera (headless)

```
python ~/Delta_Robot/pi/tof/tof_check.py                # tries CSI, then USB
python ~/Delta_Robot/pi/tof/tof_check.py --connection usb
```

It prints the device type, resolution, intrinsics, the frame rate, **the distance at the centre pixel**, a flat-wall
check, and saves `pi/tof/tof_check_out/depth_color.png`, `confidence.png`, `amplitude.png`. Copy them to your PC to look:

```
scp deltarobot@deltapi.local:Delta_Robot/pi/tof/tof_check_out/*.png .      # run this on your PC
```

Run it a few times and tell me what it says. In particular:

* Point it at a flat wall ~0.5–1 m away → do the five region distances agree? (decides how X/Y/Z are computed)
* Try `--range 4000` and then `--range 4` → which one the camera accepts (the vendor's own examples disagree).
* Check the picture is **not mirrored** (left of the camera = left of the image). If it is, `--hflip 1` fixes it.
* Hold a hand over the lens (too close) and then aim at a dark/shiny object, to see what invalid depth looks like.

## 4. CSI only: the boot overlay  **[sudo] + reboot — only if step 3 failed on CSI**

The CSI camera needs a device-tree overlay. This is what the vendor script does, spelled out so you can decide:

```
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.bak      # backup first
sudo nano /boot/firmware/config.txt
```

Add one line at the end, for the CSI connector you used (**tell me which: cam0 or cam1**):

```
dtoverlay=arducam-pivariety,cam0
```

The vendor script additionally changes `camera_auto_detect=1` to `0`. Try **without** that first; only set it to 0 if the
camera still is not found (it disables auto-detection of any other Pi camera). Then `sudo reboot`, and rerun step 3.
To undo: `sudo cp /boot/firmware/config.txt.bak /boot/firmware/config.txt` and reboot.

## 5. VNC (see the Pi's desktop from your PC)  **[sudo]**

```
sudo raspi-config nonint do_vnc 0                    # enable the VNC server
sudo raspi-config nonint do_vnc_resolution 1280x720  # only needed if no monitor is plugged in (headless)
```

On the Windows PC: install **RealVNC Viewer**, connect to `deltapi.local`, and log in with the Pi user name
`deltarobot` and its password. Keep VNC on your local network only; do not forward it to the internet.
(These commands are from general Raspberry Pi OS knowledge and were not run on your Pi. If `raspi-config` rejects one,
send me the message.)

*Optional, only if you ask for it:* **Raspberry Pi Connect** (remote desktop through your browser). Signing in needs
your Raspberry Pi ID and two-factor code, so the assistant can only give you the link, never sign in for you.

## 6. What to send back

Paste: the step-1 output, and the full output of `tof_check.py` (all runs), plus the saved PNGs if anything looks odd.
With that, the vision server (Phase 3) can be written against the real camera instead of guesses.
