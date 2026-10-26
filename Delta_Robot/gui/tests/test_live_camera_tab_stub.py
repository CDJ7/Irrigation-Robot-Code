"""
Tests the camera overlay for real, and the Tkinter camera tab + its hook in Delta_Robot_GUI.py against a STAND-IN for
tkinter (this machine has no display toolkit). This checks the tab's logic (status text, picture/detection matching, table,
cleanup, import-failure message) -- NOT how it looks on a real screen. Run: python gui/tests/test_live_camera_tab_stub.py
"""
import os
import queue
import sys
import types
from unittest import mock

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GUI = os.path.dirname(HERE)
sys.path.insert(0, GUI)

# ---------------------------------------------------------------- overlay (real code, no stand-ins)
import camera_overlay as ov   # noqa: E402

img = np.zeros((240, 320, 3), np.uint8)
dets = [{"class": "strawberry", "confidence": 0.9, "allowed": True, "bbox": [50, 50, 120, 120], "xyz_mm": [10.4, -5.0, 420.2]},
        {"class": "person", "confidence": 0.6, "allowed": False, "bbox": [200, 60, 260, 200], "xyz_mm": None}]
out = ov.draw_detections(img, dets)
assert out is not img and img.sum() == 0, "must draw on a COPY"
assert tuple(out[50, 80]) == ov.GREEN and tuple(out[60, 200]) == ov.RED, "allowed=green, not allowed=red"
assert ov.detection_label(dets[0]) == "strawberry 90%  X10 Y-5 Z420"
assert ov.detection_label(dets[1]) == "person 60%  no depth"
assert ov.draw_detections(img, [{"bbox": "garbage"}, {}]).shape == img.shape        # bad data must not crash
assert ov.fit_inside(np.zeros((480, 640, 3), np.uint8), 460, 345).shape[:2] == (345, 460)
assert ov.fit_inside(img, 460, 345).shape == img.shape                              # never enlarges
txt, ok = ov.summarize_frame({"seq": 7, "server_fps": 12.3, "calibrated": False, "model": "yolov8n.pt",
                              "model_has_allowed_class": False})
assert not ok and "UNCALIBRATED" in txt and "no allowed" in txt
txt, ok = ov.summarize_frame({"seq": 8, "calibrated": True, "model": "s.pt", "model_has_allowed_class": True})
assert ok and "UNCALIBRATED" not in txt
print("overlay OK")

# ---------------------------------------------------------------- stand-in tkinter + ImageTk
class FakeVar:
    def __init__(self, value=""): self.v = value
    def get(self): return self.v
    def set(self, v): self.v = v

def Widget(*args, **kwargs):          # any widget: accepts anything, records calls
    return mock.MagicMock()

fake_tk = types.ModuleType("tkinter")
fake_tk.StringVar, fake_tk.Misc, fake_tk.Tk = FakeVar, object, object
fake_ttk = types.ModuleType("tkinter.ttk")
for name in ("Frame", "Label", "LabelFrame", "Entry", "Button", "Treeview", "Notebook", "Combobox", "Checkbutton", "Scrollbar"):
    setattr(fake_ttk, name, Widget)
fake_tk.ttk = fake_ttk
fake_tk.messagebox = types.ModuleType("tkinter.messagebox")
fake_tk.BooleanVar = FakeVar
fake_tk.Text = Widget
sys.modules.update({"tkinter": fake_tk, "tkinter.ttk": fake_ttk, "tkinter.messagebox": fake_tk.messagebox})
fake_imagetk = types.ModuleType("PIL.ImageTk")
fake_imagetk.PhotoImage = lambda image: ("photo", image.size)
import PIL                                                                          # noqa: E402
PIL.ImageTk = fake_imagetk
sys.modules["PIL.ImageTk"] = fake_imagetk

import live_camera_tab as lct   # noqa: E402

class FakeRoot:
    def __init__(self): self.after_calls, self.cancelled = [], []
    def after(self, ms, fn): self.after_calls.append((ms, fn)); return len(self.after_calls)
    def after_cancel(self, i): self.cancelled.append(i)

class FakeClient:
    def __init__(self, q): self.q, self.started, self.stopped = q, None, 0
    def start(self, host, port): self.started = (host, port)
    def stop(self): self.stopped += 1

logs = []
root = FakeRoot()
tab = lct.LiveCameraTab(mock.MagicMock(), root, log=logs.append, client_factory=FakeClient)
assert root.after_calls and root.after_calls[0][0] == 40, "polling must be scheduled"
assert tab.frame_queue.maxsize > 0, "the GUI queue must be bounded"

# connect button uses the address/port boxes, and does not block
tab.host_var.set("192.168.137.50"); tab.port_var.set("8485")
tab._toggle()
assert tab.client.started == ("192.168.137.50", 8485) and tab._running
tab.port_var.set("abc"); tab._toggle()                                               # now disconnects first
assert tab.client.stopped == 1 and not tab._running
tab.port_var.set("oops"); tab._toggle()                                              # bad port: no crash, no start
assert not tab._running

# a burst of messages is handled; pictures get THEIR OWN detections
rgb = np.zeros((480, 640, 3), np.uint8)
j5 = {"seq": 5, "calibrated": False, "model": None, "server_fps": 9.0,
      "detections": [{"class": "strawberry", "confidence": 0.8, "allowed": True, "bbox": [10, 10, 60, 60], "xyz_mm": [1, 2, 300]}]}
j6 = {"seq": 6, "calibrated": False, "model": None, "server_fps": 9.0, "detections": []}
for item in [("STATUS", {"state": "connected", "message": "receiving data"}), ("DET", j5), ("V", rgb, 5), ("D", rgb, 5),
             ("DET", j6), ("V", rgb, 6)]:
    tab.frame_queue.put(item)
drawn = []
with mock.patch.object(lct, "draw_detections", side_effect=lambda im, d: drawn.append(d) or im):
    tab._poll()
assert tab._video_photo and tab._depth_photo, "pictures must be shown"
assert tab._video_photo == ("photo", (460, 345)), tab._video_photo
assert drawn == [] , "newest picture is seq 6, which has NO detections -> nothing drawn on it"
with mock.patch.object(lct, "draw_detections", side_effect=lambda im, d: drawn.append(d) or im):
    tab.frame_queue.put(("V", rgb, 5)); tab._poll()
assert drawn and drawn[0][0]["class"] == "strawberry", "a seq-5 picture must get the seq-5 detections"
assert root.after_calls[-1][0] == 40, "polling re-scheduled"

# status messages for each state
for st, word in [({"state": "connecting", "message": "x"}, "Connecting"), ({"state": "disconnected", "message": "lost"}, "Disconnected"),
                 ({"state": "connected", "message": "ok"}, "Connected")]:
    tab._show_status(st)
    assert tab.status_lbl.config.call_args.kwargs["text"].startswith(word), (st, tab.status_lbl.config.call_args)

# detection table
tab.table.insert.reset_mock()
tab._update_table(j5)
vals = tab.table.insert.call_args.kwargs["values"]
assert vals[0] == "strawberry" and vals[2] == "yes" and vals[5] == "300", vals

tab.shutdown()
assert tab.client.stopped >= 2 and root.cancelled
print("tab logic OK")

# ---------------------------------------------------------------- the hook in the real GUI file
sys.modules["serial"] = mock.MagicMock(); sys.modules["serial.tools"] = mock.MagicMock(); sys.modules["serial.tools.list_ports"] = mock.MagicMock()
import Delta_Robot_GUI as gui   # noqa: E402
app = gui.DeltaRobotApp.__new__(gui.DeltaRobotApp)
app.tab_camera, app.root, app.live_camera = mock.MagicMock(), FakeRoot(), None
app._log = logs.append
app._build_camera_tab()
assert isinstance(app.live_camera, lct.LiveCameraTab), "tab must be created"
# if the camera libraries are missing the GUI must still start and explain
app2 = gui.DeltaRobotApp.__new__(gui.DeltaRobotApp)
app2.tab_camera, app2.root, app2.live_camera, app2._log = mock.MagicMock(), FakeRoot(), None, logs.append
sys.modules.pop("live_camera_tab")
with mock.patch.dict(sys.modules, {"live_camera_tab": None}):                       # makes "from live_camera_tab import" fail
    app2._build_camera_tab()
assert app2.live_camera is None
print("GUI hook OK (tab created; graceful message when libraries are missing)")
print("\nALL TESTS PASSED")
