# Training a strawberry detector

**Status: no strawberry model exists yet.** The stock YOLO weights know 80 everyday classes (people, cars...) and **no strawberry**. The vision server reports
`model_has_allowed_class: false` for them and the GUI **refuses to start picking**. This guide produces a model that fixes that.

## Important limits of this camera
* The camera is **grayscale**. A model trained on its pictures can find strawberries, but **cannot tell ripe from unripe**. Ripeness needs a colour camera.
* Strawberries are small: about 31 px wide at 70 cm in a 640x480 eye. `config.yaml` has `imgsz: 320` (fast); try 640 if small berries are missed.

## 1. Collect pictures (on the Pi, through the real camera)
Train on pictures that look exactly like what the robot sees: same camera, same height and angle, same lighting, grayscale.
```
sudo systemctl stop delta-vision                       # only one program may use the camera
python pi_camera_server.py --capture-dataset ~/berries --interval 1
```
Move plants/berries around, vary light (morning, cloudy, shade), include leaves, soil, hands and empty scenes (negatives). Aim for **300+** images to start, 1000+ for good results.
Copy them to your PC: `scp -r deltarobot@<pi-ip>:berries .`  Check no pictures contain people's faces or anything private before uploading anywhere.

## 2. Label
Use a labelling tool (Roboflow, CVAT or Label Studio). One class named exactly **`strawberry`** (it must match `allowed_classes` in `pi/config.yaml` and the GUI Harvest page).
Draw a tight box around every berry, including partly hidden ones. Export in **YOLO format** with a train/validation split (about 80/20). Never put near-identical frames in both sets.

## 3. Train (Google Colab, free GPU)
In Colab: Runtime -> Change runtime type -> GPU. Upload your dataset folder (with `data.yaml`) to Drive, then:
```
!pip install ultralytics
from ultralytics import YOLO
model = YOLO("yolo11n.pt")            # small and fast; starts from the stock weights
model.train(data="/content/drive/MyDrive/berries/data.yaml", epochs=100, imgsz=640, batch=16)
metrics = model.val()                 # look at mAP50 and, more importantly, the validation pictures
```
Download `runs/detect/train/weights/best.pt`. **Do not commit it to git** (`*.pt` is in `.gitignore`; weights are large and your data may be private).

## 4. Put it on the Pi
1. `scp best.pt deltarobot@<pi-ip>:~/Delta_Robot/pi/models/strawberry.pt`
2. In the Pi's venv: `pip install ultralytics` (large download; pulls in PyTorch; see the note in `pi/requirements.txt` about `opencv-python` vs headless).
3. Optional, for speed on the Pi CPU, export to NCNN: `yolo export model=models/strawberry.pt format=ncnn imgsz=320` and point `model:` at the exported `*_ncnn_model` folder. Compare the speed and the accuracy before trusting it.
4. Edit `pi/config.yaml`: `detection: { enabled: true, model: models/strawberry.pt, allowed_classes: [strawberry], imgsz: 320 }`, restart the service.
5. In the GUI Camera page the boxes turn green ("allowed"), and the Harvest checklist item "Detection model has a strawberry class" turns green.

## 5. Test before trusting
* Look at it live: false boxes on leaves, shadows, fingers? Add those pictures as negatives and retrain.
* Check the **centre distance** and each detection's Z against a ruler.
* Start the Harvest page in **DRY RUN** (no vacuum) with the robot at low speed first.
* Check the confidence threshold (`min_confidence`, default 0.5): too low picks ghosts, too high misses berries.

## Licence note
Ultralytics YOLO is **AGPL-3.0**. Training and running it for your own project is fine; if you distribute software that includes it, or offer it as a network service, AGPL
obligations apply (publish corresponding source, keep the licence). Your trained weights are derived from AGPL tooling: see `LICENSES.md`.
