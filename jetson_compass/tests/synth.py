"""Synthetic compass images for tests (no camera needed).

Draws a top-down dial with a red/white needle, then warps it into a camera image
with perspective, rotation and optional mirroring - like a real oblique mount.
Returns the image and the 4 rim points (nose, right, tail, left) in image pixels.
"""

import math

import cv2
import numpy as np

TOP = 600  # top-down drawing size


def draw_dial(needle_deg, needle_len=0.8, needle_halfwidth=0.06, red_n_marking=False):
    c = TOP / 2.0
    R = 0.45 * TOP
    img = np.full((TOP, TOP, 3), (40, 60, 40), np.uint8)  # airframe-ish background
    cv2.circle(img, (int(c), int(c)), int(R), (30, 30, 30), -1)          # bezel
    cv2.circle(img, (int(c), int(c)), int(R * 0.95), (225, 225, 220), -1)  # dial face
    for k in range(36):  # tick marks fixed to the airframe
        a = math.radians(k * 10)
        r0 = R * (0.86 if k % 9 else 0.80)
        p0 = (int(c + r0 * math.sin(a)), int(c - r0 * math.cos(a)))
        p1 = (int(c + R * 0.93 * math.sin(a)), int(c - R * 0.93 * math.cos(a)))
        cv2.line(img, p0, p1, (20, 20, 20), 2)
    if red_n_marking:  # printed red mark on the dial ring (outside the annulus)
        cv2.circle(img, (int(c), int(c - R * 0.9)), 8, (0, 0, 200), -1)

    a = math.radians(needle_deg)
    ux, uy = math.sin(a), -math.cos(a)   # toward north tip (y is down)
    px, py = -uy, ux                     # perpendicular
    L, W = needle_len * R, needle_halfwidth * R
    tip_n = (c + L * ux, c + L * uy)
    tip_s = (c - L * ux, c - L * uy)
    b1 = (c + W * px, c + W * py)
    b2 = (c - W * px, c - W * py)
    cv2.fillPoly(img, [np.int32([tip_n, b1, b2])], (30, 30, 210))   # north: red
    cv2.fillPoly(img, [np.int32([tip_s, b1, b2])], (235, 235, 235))  # south: white
    cv2.circle(img, (int(c), int(c)), int(0.07 * R), (60, 60, 60), -1)  # pivot hub
    return img


def rim_points_top():
    c = TOP / 2.0
    R = 0.45 * TOP
    return np.float32([[c, c - R], [c + R, c], [c, c + R], [c - R, c]])


def camera_view(top_img, out_size=(1280, 720), rot_deg=37.0, tilt=0.25, mirror=False,
                noise=4.0, blur=True, seed=0):
    """Warp the top-down dial into a 'camera' image. Returns (image, rim_points)."""
    w, h = out_size
    c = TOP / 2.0
    # Destination quad: rotated square, squashed on one side for perspective.
    corners = np.float32([[0, 0], [TOP, 0], [TOP, TOP], [0, TOP]])
    s = 0.6 * h
    a = math.radians(rot_deg)
    quad = []
    for x, y in corners:
        x0, y0 = (x - c) / TOP * s, (y - c) / TOP * s
        if y0 < 0:  # far edge looks smaller
            x0 *= (1.0 - tilt)
            y0 *= (1.0 - tilt * 0.5)
        if mirror:
            x0 = -x0
        xr = x0 * math.cos(a) - y0 * math.sin(a)
        yr = x0 * math.sin(a) + y0 * math.cos(a)
        quad.append([w * 0.5 + xr, h * 0.5 + yr])
    H = cv2.getPerspectiveTransform(corners, np.float32(quad))
    img = cv2.warpPerspective(top_img, H, (w, h), borderValue=(50, 70, 50))
    if blur:
        img = cv2.GaussianBlur(img, (3, 3), 0)
    if noise:
        rng = np.random.RandomState(seed)
        img = np.clip(img.astype(np.float32) + rng.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    pts = cv2.perspectiveTransform(rim_points_top().reshape(-1, 1, 2), H).reshape(-1, 2)
    return img, pts.tolist()
