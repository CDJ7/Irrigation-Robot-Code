# Licences and credits  (not legal advice)

**This folder is licensed under the GNU GPL v3** (see `LICENSE` in this folder; chosen by the project owner). The rest of the `Irrigation-Robot-Code` repository has no licence and is not covered.
The original files in `archive/original_files/` come from `greatroboticslab/deltarobot`: confirm you may republish them (they keep their original terms).

| Component | Licence | How it is used here |
|---|---|---|
| **Ultralytics YOLO** (`ultralytics` package, YOLO weights) | **AGPL-3.0** (an Enterprise licence is sold separately) | Imported as a library by `pi/pi_camera_server.py` when detection is enabled; used to train your model. Using it privately is fine. If you **distribute** a program that includes it, or let others use it **over a network**, AGPL requires you to offer the complete corresponding source under AGPL-3.0. Weights you train from Ultralytics models are best treated as AGPL-3.0 too. |
| **PyQt6** | GPL v3 / commercial (Riverbank) | The GUI imports PyQt6. Distributing the GUI (e.g. as an .exe) means the GPL v3 (or a commercial PyQt licence) applies. Qt for Python (**PySide6**, LGPL) is the usual alternative if this matters. |
| NumPy, OpenCV (`opencv-python`), PyYAML, pyserial, Pillow | BSD / Apache-2.0 / MIT / HPND style, permissive | Normal dependencies. Keep their notices if you redistribute. |
| **PyQt-on-camera / "PC CamTouch"** (hoangnv170752, PC CamTouch Team) | MIT | Used only as a *reference* for GUI structure/style; no code copied. |
| **Arducam_tof_camera** (ArduCAM) | **No licence file** | Reference only; **no code copied**. The `ArducamDepthCamera` wheel is installed from PyPI under its own terms (only needed if a ToF camera is ever connected). |
| **greatroboticslab/deltarobot** | see that repository | The original Tkinter GUI, camera client, Pi server and ESP32 firmware this work upgrades (kept untouched in `archive/original_files/`). Confirm you have the right to republish them. |
| **Roboflow, CVAT, Google Colab** | their terms | Only if you use them for labelling/training. Do not upload private images. |

## About the choice of GPL-3.0
* Because of **Ultralytics AGPL-3.0** and **PyQt6 GPL-3.0**, GPL-3.0 is consistent for the GUI. Note that AGPL-3.0 (Ultralytics, used on the Pi) additionally covers *network use*:
  if you offer the Pi vision server to other people over a network, offer them its source as AGPL-3.0 requires. GPL-3.0 and AGPL-3.0 code can be combined.
* A permissive licence (MIT/Apache-2.0) is possible only if you keep the AGPL/GPL parts replaceable (e.g. ship without ultralytics, use PySide6) and say so clearly.
* If this is coursework or lab work, check the rules of your school or lab before choosing.

## Never committed
`.gitignore` blocks model weights (`*.pt`, `.onnx`, ...), datasets, videos, keys, `.env` and `third_party/`. The unrelated `irrBot-version1.ino` in the repo root contains Wi-Fi
credentials that were already committed by someone else: that file is **not part of this project** and was not touched, but those credentials should be rotated and removed from history.
