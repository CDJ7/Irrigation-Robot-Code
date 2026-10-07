"""
Tests YoloDetector (our wrapper) against a STAND-IN for the ultralytics results API, because
ultralytics/PyTorch cannot be installed in the dev sandbox. This checks OUR code (class names,
box parsing, config use), NOT the real library -- the real YOLO run is verified on the Pi.
Run:  python tests/test_yolo_wrapper_stub.py
"""
import os, sys, types
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))


class Scalar:                      # behaves like a 1-element torch tensor for int()/float()
    def __init__(self, v): self.v = v
    def __int__(self): return int(self.v)
    def __float__(self): return float(self.v)


class Box:
    def __init__(self, cls, conf, xyxy):
        self.cls, self.conf, self.xyxy = Scalar(cls), Scalar(conf), [list(xyxy)]


class FakeYOLO:
    calls = []
    def __init__(self, path):
        self.path = path
        self.names = {0: "strawberry", 1: "person"}
    def __call__(self, img, **kw):
        FakeYOLO.calls.append((img.shape, kw))
        res = types.SimpleNamespace(boxes=[Box(0, 0.8, (10, 20, 50, 60)), Box(1, 0.4, (100, 100, 150, 200))])
        return [res]


sys.modules["ultralytics"] = types.SimpleNamespace(YOLO=FakeYOLO)
import pi_camera_server as srv   # noqa: E402

cfg = srv.load_config(os.path.join(os.path.dirname(HERE), "config.yaml"))
det = srv.YoloDetector(cfg["detection"])
out = det.detect(np.zeros((240, 320), np.uint8))
assert [d["class"] for d in out] == ["strawberry", "person"], out
assert out[0]["bbox"] == (10.0, 20.0, 50.0, 60.0) and abs(out[0]["confidence"] - 0.8) < 1e-9
shape, kw = FakeYOLO.calls[-1]
assert shape == (240, 320, 3), "YOLO must get a 3-channel image"
assert kw["imgsz"] == cfg["detection"]["imgsz"] and kw["conf"] == cfg["detection"]["confidence"]
print("YoloDetector wrapper OK (stand-in library):", [(d["class"], round(d["confidence"], 2)) for d in out])

# model_has_allowed_class logic: stock-like model without 'strawberry' must be flagged False
class StockNames(FakeYOLO):
    def __init__(self, p): super().__init__(p); self.names = {0: "person", 1: "bicycle"}
sys.modules["ultralytics"] = types.SimpleNamespace(YOLO=StockNames)
det2 = srv.YoloDetector(cfg["detection"])
hub = srv.Hub()
pipe = srv.VisionPipeline(cfg, hub, camera=None, detector=det2, dataset_cfg=cfg["dataset"])
assert pipe._model_has_allowed() is False
det3 = srv.YoloDetector(cfg["detection"]) if False else None
print("stock-like model (no strawberry class) -> model_has_allowed_class = False OK")
print("\nALL TESTS PASSED")
