"""
prepare_dataset.py -- turn a colour strawberry dataset (YOLO format) into what OUR camera sees: one class, grayscale.

    .venv/bin/python yolo/prepare_dataset.py SRC_DIR OUT_DIR [--val 0.15] [--test 0.05]

Why:
  * The ELP-USB3DGS800P-V83 stereo camera (OV9281 sensor) is MONOCHROME, so ripe and unripe cannot be told apart -> every
    strawberry class becomes class 0 "strawberry" (the name the Pi's allowed_classes and the GUI's Harvest page require).
    Non-berry classes (e.g. peduncle / stem, flower) are dropped.
  * A mono sensor's brightness is NOT the standard RGB->gray formula (it sees more red and near-infrared, where leaves are
    bright). Each image gets RANDOM channel weights around the usual formula, sometimes red-heavy, so the detector does not
    rely on one particular gray rendering. Validation images get the same treatment with a fixed seed.

SRC_DIR is searched recursively: every image with a same-name .txt label file is used. Class names come from data.yaml /
*.names / classes.txt; pass --keep "name1,name2" to choose the berry classes yourself.
The split is done by image name prefix groups when possible (frames from one sequence stay in one split).
"""
import argparse
import os
import random
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np

IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp"}
BERRY_WORDS = ("strawberr", "ripe", "unripe", "berry", "fruit", "green", "white", "pink", "red", "turning")
NOT_BERRY_WORDS = ("peduncle", "stem", "flower", "calyx", "leaf", "pedicel")


def find_names(src: Path) -> Optional[List[str]]:
    for p in list(src.rglob("data.yaml")) + list(src.rglob("*.yaml")):
        txt = p.read_text(errors="ignore")
        m = re.search(r"names\s*:\s*\[(.*?)\]", txt, re.S)
        if m:
            return [s.strip().strip("'\"") for s in m.group(1).split(",") if s.strip()]
        m = re.search(r"names\s*:\s*\n((?:\s+\d+\s*:.*\n?)+)", txt)
        if m:
            return [line.split(":", 1)[1].strip().strip("'\"") for line in m.group(1).strip().splitlines()]
    for pat in ("*.names", "classes.txt", "obj.names"):
        for p in src.rglob(pat):
            names = [l.strip() for l in p.read_text(errors="ignore").splitlines() if l.strip()]
            if names:
                return names
    return None


def berry_ids(names: List[str], keep: Optional[str]) -> Dict[int, str]:
    if keep:
        wanted = {k.strip().lower() for k in keep.split(",")}
        return {i: n for i, n in enumerate(names) if n.lower() in wanted}
    out = {}
    for i, n in enumerate(names):
        low = n.lower()
        if any(w in low for w in NOT_BERRY_WORDS):
            continue
        if any(w in low for w in BERRY_WORDS):
            out[i] = n
    return out


def to_gray(img: np.ndarray, rng: random.Random) -> np.ndarray:
    b, g, r = cv2.split(img.astype(np.float32))
    if rng.random() < 0.3:                                    # red / near-infrared heavy (mono sensors without IR-cut filter)
        w = np.array([rng.uniform(0.0, 0.15), rng.uniform(0.2, 0.45), rng.uniform(0.5, 0.8)])
    else:                                                     # around the standard formula 0.114 B + 0.587 G + 0.299 R
        w = np.array([0.114, 0.587, 0.299]) * np.array([rng.uniform(0.6, 1.4) for _ in range(3)])
    w /= w.sum()
    gray = w[0] * b + w[1] * g + w[2] * r
    gamma = rng.uniform(0.8, 1.25)                            # exposure / gamma differences
    gray = 255.0 * (np.clip(gray, 0, 255) / 255.0) ** gamma
    return np.clip(gray, 0, 255).astype(np.uint8)


def group_key(stem: str) -> str:
    """Images from the same sequence share a prefix (e.g. 'rowA_0012' -> 'rowA'); keep groups in one split."""
    m = re.match(r"(.*?)[_\-\s]?\d+$", stem)
    return m.group(1) if m and m.group(1) else stem


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--val", type=float, default=0.15)
    ap.add_argument("--test", type=float, default=0.05)
    ap.add_argument("--keep", help="comma-separated class names that are strawberries (default: guessed from the names)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max-side", type=int, default=1280, help="shrink bigger pictures to this many pixels on the long side")
    a = ap.parse_args()
    src, out = Path(a.src), Path(a.out)

    names = find_names(src)
    if not names:
        sys.exit("no class names found (data.yaml / *.names / classes.txt) -- cannot tell which classes are berries")
    keep = berry_ids(names, a.keep)
    print("classes in the source:", dict(enumerate(names)))
    print("kept as 'strawberry'  :", keep)
    if not keep:
        sys.exit("no berry class recognised -- pass --keep")

    pairs = []
    for img in src.rglob("*"):
        if img.suffix.lower() in IMG_EXT:
            lab = img.with_suffix(".txt")
            if not lab.exists():                               # images/... and labels/... side by side
                parts = list(img.parts)
                if "images" in parts:
                    parts[len(parts) - 1 - parts[::-1].index("images")] = "labels"
                    lab = Path(*parts).with_suffix(".txt")
            if lab.exists():
                pairs.append((img, lab))
    if not pairs:
        sys.exit("no image/label pairs found")

    rng = random.Random(a.seed)
    groups: Dict[str, list] = {}
    for p in sorted(pairs):
        groups.setdefault(str(p[0].parent) + "/" + group_key(p[0].stem), []).append(p)
    keys = sorted(groups)
    rng.shuffle(keys)
    if len(keys) < 10:                                         # too few groups to split by group: split by image instead
        keys = [str(p[0]) for p in sorted(pairs)]
        groups = {str(p[0]): [p] for p in pairs}
        rng.shuffle(keys)

    def source_split(k: str) -> Optional[str]:
        parts = {s.lower() for s in Path(groups[k][0][0]).parts[:-1]}
        if parts & {"training", "train"}:
            return "train"
        if parts & {"validation", "valid", "val"}:
            return "val"
        if "test" in parts:
            return "test"
        return None

    if all(source_split(k) for k in keys):
        # the source already has a split (made by its authors, who know which frames are near-duplicates): keep it.
        # Without a test set, half of the source's validation groups become our test set.
        split_of = {k: source_split(k) for k in keys}
        if not any(v == "test" for v in split_of.values()):
            vals = [k for k in keys if split_of[k] == "val"]
            for k in vals[: len(vals) // 2]:
                split_of[k] = "test"
        print("using the source's own train/validation split")
    else:
        n = len(keys)
        n_test, n_val = int(round(n * a.test)), int(round(n * a.val))
        split_of = {k: ("test" if i < n_test else "val" if i < n_test + n_val else "train") for i, k in enumerate(keys)}

    counts = {s: [0, 0] for s in ("train", "val", "test")}
    for s in counts:
        (out / "images" / s).mkdir(parents=True, exist_ok=True)
        (out / "labels" / s).mkdir(parents=True, exist_ok=True)
    for k in keys:
        s = split_of[k]
        for img_p, lab_p in groups[k]:
            img = cv2.imread(str(img_p), cv2.IMREAD_COLOR)
            if img is None:
                continue
            scale = a.max_side / max(img.shape[:2])                # labels are normalised, so resizing keeps them valid
            if scale < 1.0:
                img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            lines = []
            for line in lab_p.read_text(errors="ignore").splitlines():
                f = line.split()
                if len(f) >= 5 and int(float(f[0])) in keep:
                    lines.append("0 " + " ".join(f[1:5]))
            stem = f"{group_key(img_p.stem)}__{img_p.stem}"[:150]
            img_rng = random.Random(f"{a.seed}-{img_p}")
            cv2.imwrite(str(out / "images" / s / f"{stem}.jpg"), to_gray(img, img_rng), [cv2.IMWRITE_JPEG_QUALITY, 92])
            (out / "labels" / s / f"{stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
            counts[s][0] += 1
            counts[s][1] += len(lines)
    (out / "data.yaml").write_text(f"path: {out.resolve()}\ntrain: images/train\nval: images/val\ntest: images/test\n"
                                   "names:\n  0: strawberry\n")
    for s, (ni, nb) in counts.items():
        print(f"{s:5s}: {ni:5d} images, {nb:6d} strawberry boxes")
    print("wrote", out / "data.yaml")


if __name__ == "__main__":
    main()
