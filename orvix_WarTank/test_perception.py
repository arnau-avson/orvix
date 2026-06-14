"""Run the traffic-light detection + classification pipeline on a single image.

Usage:
    python test_perception.py [image_path] [--model n|s|m|l|x] [--conf 0.30]
                              [--imgsz 1280] [--no-sv-mask]

Defaults are tuned for Google Street View screenshots:
  model 'm', conf 0.30, imgsz 1280, Street View overlays masked out.
For real camera frames, pass --no-sv-mask.
"""
import argparse
import sys
from pathlib import Path

import cv2

from delivery_robot.perception import (
    YOLOTrafficLightDetector,
    classify_state,
    streetview_overlay_regions,
)


_STATE_COLOR = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "green": (0, 255, 0),
    "unknown": (200, 200, 200),
}


def main(image_path: str, model_size: str, conf: float, imgsz: int, sv_mask: bool, show_unknown: bool) -> int:
    img = cv2.imread(image_path)
    if img is None:
        print(f"ERROR: could not read image: {image_path}")
        return 1

    h, w = img.shape[:2]
    print(f"Image: {image_path}  ({w}x{h})")
    print(f"Model: yolov8{model_size}.pt  conf={conf}  imgsz={imgsz}  sv_mask={sv_mask}")

    ignore_regions = streetview_overlay_regions(img.shape) if sv_mask else []
    if ignore_regions:
        print(f"Masking {len(ignore_regions)} Street View overlay region(s)")

    detector = YOLOTrafficLightDetector(
        model_size=model_size,
        min_confidence=conf,
        imgsz=imgsz,
        ignore_regions=ignore_regions,
    )
    candidates = detector.detect(img)

    classified = [(d, classify_state(d.crop(img))) for d in candidates]
    validated = [(d, s) for d, s in classified if s != "unknown"]
    rejected = [(d, s) for d, s in classified if s == "unknown"]

    print(f"YOLO candidates: {len(candidates)}")
    print(f"Validated traffic lights: {len(validated)}")
    if rejected and show_unknown:
        print(f"Rejected by classifier: {len(rejected)}")

    if not validated and not (rejected and show_unknown):
        print("(no traffic lights survived)")
        return 0

    annotated = img.copy()
    # Draw masked regions in faint gray for visual debugging.
    for rx1, ry1, rx2, ry2 in ignore_regions:
        cv2.rectangle(annotated, (rx1, ry1), (rx2, ry2), (80, 80, 80), 1)

    rows = validated + (rejected if show_unknown else [])
    for i, (d, state) in enumerate(rows, 1):
        color = _STATE_COLOR[state]
        tag = "REJECTED " if state == "unknown" else ""
        print(
            f"  [{i}] {tag}bbox=({d.x1},{d.y1})-({d.x2},{d.y2})  "
            f"size={d.width}x{d.height}  ar={d.aspect_ratio:.2f}  "
            f"conf={d.confidence:.2f}  state={state}"
        )
        cv2.rectangle(annotated, (d.x1, d.y1), (d.x2, d.y2), color, 2)
        cv2.putText(
            annotated,
            f"{state} {d.confidence:.2f}",
            (d.x1, max(d.y1 - 6, 14)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )

    out = Path(image_path).with_name(Path(image_path).stem + "_detected.png")
    cv2.imwrite(str(out), annotated)
    print(f"Annotated image: {out}")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("image", nargs="?", default="image.png")
    parser.add_argument("--model", default="m", choices=["n", "s", "m", "l", "x"])
    parser.add_argument("--conf", type=float, default=0.30)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument(
        "--no-sv-mask",
        action="store_true",
        help="Disable Street View UI overlay masking (use for real camera frames).",
    )
    parser.add_argument(
        "--show-unknown",
        action="store_true",
        help="Also list/draw detections that the classifier rejected as unknown.",
    )
    args = parser.parse_args()
    sys.exit(main(args.image, args.model, args.conf, args.imgsz, not args.no_sv_mask, args.show_unknown))
