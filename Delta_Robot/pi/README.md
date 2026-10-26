# Pi vision server — setup and use

The Pi does **vision only**: camera → stereo depth → YOLO → stream. It never moves the robot.
Everything below runs **on the Pi** (over SSH). Nothing here needs `sudo` except where marked.

## 1. Copy the code to the Pi (from your Windows PC, in this project folder)

```
scp -r pi deltarobot@deltapi.local:Delta_Robot/
```

## 2. Create the Python environment (once, on the Pi)

```
cd ~/Delta_Robot
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r pi/requirements.txt        # ultralytics pulls in PyTorch: several minutes
```

Packages go into the venv only — never `sudo pip`.

## 3. Make sure nothing else is holding the camera

Only one program can stream from `/dev/video0`. The old `main_harvester.py` must not run.

```
pgrep -af main_harvester.py      # should print nothing
fuser -v /dev/video0             # should print nothing   (apt: psmisc)
```

## 3b. Check what the camera delivers

```
python camera_check.py                 # table of modes: real size, real fps, grayscale?
python camera_check.py --save 1280x480 # also saves left/right/pair pictures to camera_check_out/
```

## 4. First run

```
cd ~/Delta_Robot/pi
python pi_camera_server.py --fake      # no camera: synthetic test scene, checks the software
python pi_camera_server.py             # real camera
```

Open `http://deltapi.local:5000` in a browser: left = camera + detections, right = depth
(red near, blue far). Green box = allowed class, red box = detected but NOT allowed to pick.
The log prints fps every 10 s. Until you calibrate, the image says `UNCALIBRATED`.

Self-tests (no hardware): `python tests/test_server_fake.py`, `tests/test_calibration_synthetic.py`,
`tests/test_yolo_wrapper_stub.py`.

If the frame rate is low, try `fourcc: MJPG` (or `YUYV`) in `config.yaml` → `camera:`.

## 4b. Stream the camera to the GUI on your PC

1. **On the Pi** (ssh in):
   ```
   source ~/Delta_Robot/venv/bin/activate
   cd ~/Delta_Robot/pi
   python pi_camera_server.py
   ```
   It prints `Listening: TCP 8485, browser http://deltapi.local:5000` and then a status line every 10 seconds
   (frames per second, milliseconds spent on depth). Leave it running; Ctrl+C stops it. Only one program can use the camera.
2. **Quick check without the GUI:** open `http://192.168.137.128:5000` in a browser on your PC (left = camera,
   right = depth).
3. **In the GUI** (on your PC): `pip install -r gui/requirements.txt`, then `python gui/Delta_Robot_GUI.py`, open the
   **Live Camera (Pi)** tab, check the address (`192.168.137.128`), click **Connect**. It shows the video, the depth picture,
   and a table of detections. The tab is view-only: it cannot move the robot.
4. If the Pi's address changes, type the new one in the box and reconnect. If the link drops, the tab reconnects by itself.

Until the stereo calibration is done the tab warns "UNCALIBRATED: depth values are NOT trustworthy", and until a strawberry
YOLO model exists there are no detections to pick (with `detection.enabled: false` there are none at all).

## 5. Stereo calibration (needed for real millimeters)

1. On the PC, print the board: `python make_checkerboard.py` writes
   `checkerboard_9x6_25mm_letter.svg` (already included). Open it in a browser, print at
   **100 % / Actual size** (not "fit to page"). Glue it flat on a rigid board.
   **Measure one square** with a ruler; use that number as `--square-mm` below.
2. Stop the server so the camera is free.
3. `python calibrate_stereo.py capture --square-mm 25` and move the board slowly through the
   picture — corners and edges, near and far, tilted ±30°. See the header of
   `calibrate_stereo.py` for the full checklist. To see what the camera sees over SSH:
   `scp deltarobot@deltapi.local:Delta_Robot/pi/calib_preview.jpg .`
4. `python calibrate_stereo.py solve --square-mm 25` — aim for stereo RMS < ~0.5 px.
   It prints the measured **baseline** (compare with a ruler) and the nearest measurable distance
   for your `num_disparities`; it saves `calibration/stereo_calib.npz`, which the server loads.
5. **Check against a ruler**: put a flat textured object at a measured distance (e.g. 400 mm
   from the camera) and open `http://deltapi.local:5000/status` and read `center_probe` (depth at the image center),
   or look at the `Z` label on a detection. If it is off by more than ~2 %, redo the calibration.

## 6. Run at boot (uses sudo — run it yourself)

```
sh install_service.sh        # copies delta-vision.service, enables and starts it
journalctl -u delta-vision -f
sudo systemctl stop delta-vision     # before calibrating or dataset capture by hand
```

## 7. Collect training images

```
python pi_camera_server.py --capture-dataset            # one image / 2 s into pi/dataset_images
python pi_camera_server.py --capture-dataset ~/berries --interval 1
```

Saves the left-eye **grayscale** PNG that YOLO actually sees (rectified if calibrated). Copy the folder
to your PC for labeling (Roboflow etc.): `scp -r deltarobot@deltapi.local:Delta_Robot/pi/dataset_images .`

> Reminder: the camera is monochrome, so a model trained on these images can find strawberries
> but **cannot tell ripe from unripe**. A colour camera would be needed for ripeness.

## 8. Swap in your own trained model

Edit one line in `config.yaml`:

```
detection:
  model: models/strawberry.pt          # or an exported NCNN folder
  allowed_classes: [strawberry]        # names exactly as in the model
```

The server reports `model_has_allowed_class`. With the stock `yolov8n.pt` it is `false`
(COCO has no strawberry), and the GUI must refuse to run automated picking.
