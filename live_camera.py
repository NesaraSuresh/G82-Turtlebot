"""Live TurtleBot camera demo: dock target in green, wall landmarks in orange.

Runs the normal pipeline and opens an OpenCV window showing the robot's own
camera while it drives: the dock TARGET (marker 0) is boxed in green, wall
landmarks (markers 1-6) in orange.
Nothing in src/ is modified: it wraps the detector at run time.

    python live_camera.py
    python live_camera.py --scenario occlusion --seed 2
    python live_camera.py --viewer        # also open the 3-D MuJoCo viewer

Press q in the camera window to stop.
"""
import argparse
import time

import cv2

import src  # noqa: F401  (selects the OpenGL backend)
from src.config import SCENARIOS, make_config
from src.perception.aruco_detector import ArucoLandmarkDetector
from src.runner import run_episode

ap = argparse.ArgumentParser()
ap.add_argument("--scenario", default="nominal", choices=list(SCENARIOS))
ap.add_argument("--seed", type=int, default=1)
ap.add_argument("--viewer", action="store_true")
ap.add_argument("--speed", type=float, default=1.0, help="playback speed (1 = real time)")
args = ap.parse_args()

_detect = ArucoLandmarkDetector.detect
_last = [time.time()]


class _Stop(Exception):
    pass


DOCK_ID = 0
GREEN, ORANGE = (0, 255, 0), (0, 165, 255)          # OpenCV colours are B, G, R


def draw(image_bgr, meas):
    """Dock target in GREEN, wall landmarks in ORANGE."""
    img = image_bgr.copy()
    for m in meas:
        pts = m.corners.astype(int)
        col, name, thick = (GREEN, "TARGET (dock)", 3) if m.id == DOCK_ID else (ORANGE, f"landmark {m.id}", 2)
        cv2.polylines(img, [pts], True, col, thick)
        label = f"{name}  {m.z[0]:.2f} m"
        (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        x = min(int(pts[:, 0].min()), img.shape[1] - tw - 5)     # keep the label inside the image
        y = max(int(pts[:, 1].min()) - 8, 50)
        cv2.rectangle(img, (x - 3, y - 18), (x + tw + 3, y + 5), (0, 0, 0), -1)
        cv2.putText(img, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)
    return img


def detect_and_show(self, image_bgr, *a, **kw):
    meas = _detect(self, image_bgr, *a, **kw)
    img = draw(image_bgr, meas)
    if any(m.id == DOCK_ID for m in meas):
        txt = "TARGET in view"
    else:
        txt = f"{len(meas)} landmark(s)" if meas else "no marker"
    cv2.putText(img, f"TurtleBot camera - {txt}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.imshow("TurtleBot camera (q to quit)", img)
    # camera runs at 10 Hz in sim time -> sleep so the window plays in real time
    wait = 0.1 / args.speed - (time.time() - _last[0])
    if cv2.waitKey(max(1, int(wait * 1000))) & 0xFF == ord("q"):
        raise _Stop
    _last[0] = time.time()
    return meas


ArucoLandmarkDetector.detect = detect_and_show

cfg = make_config(args.scenario, args.seed)
try:
    run_episode(cfg, viewer=args.viewer)
except _Stop:
    print("stopped by user")
finally:
    cv2.destroyAllWindows()
