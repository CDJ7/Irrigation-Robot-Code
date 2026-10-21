# Protocols

Two separate links:

1. **PC ⇄ ESP32** over USB serial (text lines).
2. **Pi → PC** over TCP (binary messages: video, depth, detections).

---

## 1. PC ⇄ ESP32 serial protocol (current firmware)

115200 baud, 8N1, one command per line (`\n`-terminated). The firmware prints `READY DeltaRobotSlave`
when it boots. The ESP32 does **no kinematics**: it only turns step counts into pulses.

| Command | Meaning | Reply |
|---|---|---|
| `M s1 s2 s3 duration_ms` | Move the three motors to **absolute** step positions, arriving together after `duration_ms`. | `OK` when finished · `ERR BUSY` if already moving · `ERR BAD_M_ARGS` |
| `H` | Home all axes against the limit switches (blocking). | `HOMED` then `POS …` · `ERR HOMING_TIMEOUT_AXIS_n` · `ERR HOMING_ABORTED` · `ERR BUSY` |
| `Z` | Declare the current position to be step 0 (no motion). | `HOMED` |
| `EN 1` / `EN 0` | Enable / disable all three drivers. | `OK` |
| `STOP` | Abort the move in progress. | `ERR STOPPED` (if it was moving) then `POS s1 s2 s3` |
| `P?` | Report position. | `POS s1 s2 s3` |

Things to know (see `PHASE1_REVIEW.md`):

* `OK` means both "move finished" and "EN command accepted". A host must not treat every `OK` as a move completion.
* `EN 0` removes motor holding torque; arms can fall.
* `Z` and `EN` are accepted even while moving.
* Homing is blocking and only reacts to `STOP`; other input during homing is thrown away.
* There is no acceleration; the move is a straight line **in motor-step space**, not a straight line in XYZ.

---

## 2. Pi → PC network protocol (TCP port 8485)

Implemented by `pi/pi_camera_server.py` (the full, authoritative description is the comment at the top of that file).

Every message: **1 byte tag · 4 byte big-endian length N · N bytes payload**.

| Tag | Payload |
|---|---|
| `J` | UTF-8 JSON for one frame: `seq`, `t`, image `width`/`height`, `calibrated`, `intrinsics`, `model`, `model_has_allowed_class`, `allowed_classes`, `server_fps`, `timing_ms`, `center_probe`, and `detections[]` |
| `V` | JPEG of the picture the detector saw |
| `D` | JPEG of the colourised depth picture |

Per processed frame the server sends **`J`, `V`, `D` in that order**; the `V` and `D` after a `J` belong to that `J`'s `seq`.
Programs that only know `V` and `D` keep working: they ignore tags they do not know.

Each detection: `class`, `class_id`, `confidence`, `allowed` (is the class on the allowed list), `bbox [x1,y1,x2,y2]`,
`center [u,v]` (pixels), `xyz_mm [X,Y,Z]` or `null` when depth is invalid, `disparity`, `depth_valid_fraction`.

Camera frame: **X right, Y down, Z out of the lens, millimetres.** A client must refuse to pick when
`calibrated` is false or `model_has_allowed_class` is false.

Browser viewer (optional): `http://deltapi.local:5000` (`/stream`, `/snapshot.jpg`, `/status`).

*Note:* this protocol was designed with the stereo camera. The ToF version will keep the same messages; `disparity`
will be replaced by a ToF depth/confidence value in a later phase and this file updated.
