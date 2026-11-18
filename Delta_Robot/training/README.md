# Training (runs on the PC, not the Pi)

Two models make up the picking system:

```
stereo camera (ELP-USB3DGS800P-V83, mono) ─► Pi: YOLO finds berries + stereo depth gives X,Y,Z ─► PC GUI: camera→robot calibration
   ─► BerryMemory + build_observation (gui/delta_gui/core/sac_policy.py) ─► SAC policy: small X/Y/Z step ─► Robot.move_xyz safety checks ─► ESP32
```

| Part | What it learns | Trained on | Code |
|---|---|---|---|
| **Detector** (YOLO11n, 1 class `strawberry`) | where berries are in the left-eye picture | public photos converted to grayscale | `yolo/` |
| **Policy** (SAC, 24 inputs → 3 outputs) | how to steer the tip onto a berry, frame by frame | a simulator of this robot + camera | `sac/` |

## Why YOLO stays, and why SAC does not look at pixels
SAC learns by trial and error (hundreds of thousands of attempts), so it has to be trained in simulation. Simulating *numbers*
(berry positions with realistic stereo noise, missed and false detections, calibration error) is easy and faithful; simulating
*pictures* that look like this grayscale stereo camera is not, and pixel policies trained on fake pictures usually fail on the real camera.
So the detector turns pictures into numbers and SAC works on the numbers. The same `build_observation` and `BerryMemory`
code runs in the simulator and in the GUI, so the policy sees the same kind of input in both.

**Limit (honest):** the tip position comes from kinematics and the berry position from the camera, so a camera→robot
calibration error cannot be *seen* by the policy -- it is trained to tolerate a few mm, not to remove it. True visual servoing
would need the camera to also detect the effector (e.g. a marker on it, or a second YOLO class) -- a possible next step.

## Set-up (once)
```
cd Delta_Robot/training
python3 -m venv .venv
.venv/bin/pip install ultralytics stable-baselines3 gymnasium tensorboard
```

## Detector
```
# 1. data: CC BY 4.0 "Strawberry dataset for object detection" (Zenodo 6126677, ripe/unripe/peduncle, YOLO format)
mkdir -p datasets/zenodo_6126677 && cd datasets/zenodo_6126677
curl -L -o strawberries.zip https://zenodo.org/api/records/6126677/files/strawberries.zip/content && unzip -q strawberries.zip -d raw
cd ../..
# 2. one class, grayscale like our camera, train/val/test split
.venv/bin/python yolo/prepare_dataset.py datasets/zenodo_6126677/raw datasets/strawberry_gray
# 3. train + test + export for the Pi
.venv/bin/python yolo/train_yolo.py datasets/strawberry_gray/data.yaml
```
Then follow `docs/YOLO_TRAINING.md` step 4 (copy to the Pi, `imgsz: 640` in `pi/config.yaml`, enable detection).
Datasets and weights are never committed. Dataset credit: Lemsalu & Bloch, *Real-Time CNN-based Computer Vision System for
Open-Field Strawberry Harvesting Robot*, IFAC 2022, doi:10.1016/j.ifacol.2022.11.109 (CC BY 4.0).

The public photos are colour field photos; ours are mono, indoors, at 70-93 cm. Expect a drop on the real camera. Fix it by adding
our own pictures (fake strawberries are fine): `pi_camera_server.py --capture-dataset`, label, add to the dataset, retrain.

## Policy
```
.venv/bin/python sac/train_sac.py --steps 300000 --out sac/runs/v1     # trains, exports sac_policy.npz, writes eval.json
.venv/bin/python sac/evaluate.py sac/runs/v1/sac_policy.npz --noise 1.5   # robustness: 50% more noise than trained on
.venv/bin/python sac/evaluate.py                                          # the hand-written baseline, same seeds
```
`sac_policy.npz` is what the GUI loads (`NumpyPolicy`, numpy only). If `OBS_VERSION` in `sac_policy.py` changes, retrain.

### Simulator numbers to replace with measurements
In `sac/delta_pick_env.py`: disparity noise 0.25 px and detection-centre noise 0.5 px (measure with the ruler test), miss rate 8 %
(65 % when the effector hides the berry), false detections 3 % of frames, calibration error 3 mm per axis (use the camera wizard's RMS),
berry sway 0.5 mm per step, success radius 10 mm (set from the vacuum cup size). Retrain after changing them.

## Status and results (2026-11-18) -- simulation and public photos only, nothing run on the real camera or robot
Trained models live only on the training PC (`training/yolo/runs/`, `training/sac/runs/`, git-ignored).

**Detector** (YOLO11n, 640 px, 93 epochs, early stop): test split (79 unseen photos) precision 0.88, recall 0.78, mAP50 0.84,
mAP50-95 0.52. Pi model: `yolo/runs/strawberry_gray/weights/best_ncnn_model/`. Not yet tried on the ELP camera.

**SAC policy** (500 episodes per row, same seeds; "knocks" = runs that pushed through a berry):

| Version | Change | Noise 1x: success / moves / knocks | 2x noise: success / knocks |
|---|---|---|---|
| v1 | first run (knock = -2, carry on) | 100 % / 6.6 / 10 | 98.3 % / 39 (old rules) |
| v2 | knock ends the attempt, -10 | 97.8 % / 7.0 / 10 | 87.6 % / 49 |
| v3 | + near-berry speed limit (5 mm steps within 30 mm), OBS_VERSION 2 | 97.6 % / 8.6 / 11 | 86.0 % / 57 |
| scripted baseline (with speed limit) | hover above, then descend | 98.2 % / 15.5 / 4 | 76.0 % / 24 |

v1/v2 were trained with OBS_VERSION 1 and no longer load. SAC is ~2x faster and more robust to noise than the baseline, but knocks
2-3x more berries. **Cause found:** in every v3 knock the tip was level with or below the berry and moved into it sideways -- success
was defined as "within 10 mm in any direction", so side approaches were rewarded.

**Next (v4, not started):** success only when the tip is centred over the berry (~8 mm sideways) and at or just above its top
(how a vacuum cup picks); penalty for being beside/below a berry while close to it; keep the speed limit. Then: GUI integration
(SacStepper behind Robot.move_xyz on the Harvest page, dry run first), and replace the simulator's noise guesses with ruler-test values.
