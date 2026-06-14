"""Run obstacle detection on a single image and decide if the robot should stop.

Usage:
    python test_obstacles.py [image_path]
"""
import sys
from pathlib import Path

import cv2

from delivery_robot.perception import ObstacleDetector, should_stop


_CLASS_COLOR = {
    "person": (0, 0, 255),
    "dog": (0, 0, 255),
    "cat": (0, 0, 255),
    "horse": (0, 0, 255),
    "bicycle": (0, 165, 255),
    "motorcycle": (0, 165, 255),
    "car": (0, 255, 255),
    "bus": (0, 255, 255),
    "truck": (0, 255, 255),
}
_DEFAULT_COLOR = (200, 200, 200)


def main(image_path: str) -> int:
    img = cv2.imread(image_path)
    if img is None:
        print(f"ERROR: could not read image: {image_path}")
        return 1

    h, w = img.shape[:2]
    print(f"Image: {image_path}  ({w}x{h})")

    detector = ObstacleDetector()
    obstacles = detector.detect(img)

    print(f"Obstacles detected: {len(obstacles)}")
    for i, o in enumerate(obstacles, 1):
        print(
            f"  [{i}] {o.class_name:10}  bbox=({o.x1},{o.y1})-({o.x2},{o.y2})  "
            f"size={o.width}x{o.height}  conf={o.confidence:.2f}  sev={o.severity}"
        )

    blocker = should_stop(obstacles, img.shape)
    print(f"\nDecision: {'STOP — ' + blocker.class_name if blocker else 'GO (path clear)'}")

    annotated = img.copy()
    cx_min = int(0.30 * w)
    cx_max = int(0.70 * w)
    cv2.line(annotated, (cx_min, 0), (cx_min, h), (100, 100, 100), 1)
    cv2.line(annotated, (cx_max, 0), (cx_max, h), (100, 100, 100), 1)

    for o in obstacles:
        is_blocker = blocker is not None and (o.x1, o.y1, o.x2, o.y2) == (
            blocker.x1, blocker.y1, blocker.x2, blocker.y2
        )
        color = _CLASS_COLOR.get(o.class_name, _DEFAULT_COLOR)
        thickness = 4 if is_blocker else 2
        cv2.rectangle(annotated, (o.x1, o.y1), (o.x2, o.y2), color, thickness)
        label = f"{o.class_name} {o.confidence:.2f}"
        if is_blocker:
            label = "BLOCK " + label
        cv2.putText(
            annotated, label, (o.x1, max(o.y1 - 6, 14)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
        )

    out = Path(image_path).with_name(Path(image_path).stem + "_obstacles.png")
    cv2.imwrite(str(out), annotated)
    print(f"Annotated image: {out}")
    return 0


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "image2.png"
    sys.exit(main(path))
