"""Seeded sea-region segmentation + anomaly detection inside it.

1. Pick a seed in open water (auto: deepest point of the biggest dark colour
   cluster in the lower part of the frame, or pass --seed x,y).
2. Model water colour from a patch around the seed (Lab mean/cov), score every
   pixel by Mahalanobis distance, keep the connected region containing the seed.
3. Fill holes via the outer contour, so ice floes and the boat count as "sea".
4. Fit the "normal" colour model on sea pixels only and flag anomalies there.

Usage: python sea_detect.py image.png [--seed x,y] [--out DIR] [--min-score S]
"""
import argparse
import os
import re

import cv2
import numpy as np
from sklearn.mixture import GaussianMixture

PATCH = 41            # px, water sample patch around the seed
WATER_DIST = 6.0      # Mahalanobis distance cut-off for "looks like the seed water"
NOISE_FLOOR = 4.0     # added to the covariance diagonal (render/JPEG noise, Lab units)
N_COMPONENTS = 6
FIT_SAMPLES = 60_000
SMOOTH_SIGMA = 1.5
THRESH_PCTL = 99.95   # of pixels *inside the sea*
MERGE_KERNEL = 9
MIN_AREA = 20
MAX_AREA = 20_000


def auto_seed(lab):
    """Deepest point of the largest colour cluster in the bottom 40% of the frame."""
    h, w, _ = lab.shape
    band = lab[int(h * 0.6):].reshape(-1, 3)
    rng = np.random.default_rng(0)
    sample = band[rng.choice(len(band), min(20_000, len(band)), replace=False)]
    _, _, centers = cv2.kmeans(sample, 5, None,
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.5),
                               3, cv2.KMEANS_PP_CENTERS)
    # assign band pixels to centres, pick the biggest cluster
    dist = np.linalg.norm(band[:, None, :] - centers[None], axis=2)
    biggest = np.bincount(dist.argmin(1), minlength=len(centers)).argmax()
    full = np.linalg.norm(lab - centers[biggest], axis=2)
    mask = (full < 8).astype(np.uint8)
    mask[: int(h * 0.6)] = 0
    padded = cv2.copyMakeBorder(mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)  # border counts as an edge
    depth = cv2.distanceTransform(padded, cv2.DIST_L2, 5)[1:-1, 1:-1]
    y, x = np.unravel_index(depth.argmax(), depth.shape)
    return int(x), int(y)


def water_model_from_seed(lab, seed):
    """(mean, inverse covariance) of the Lab colours in a patch around `seed`."""
    x, y = seed
    r = PATCH // 2
    patch = lab[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1].reshape(-1, 3)
    cov = np.cov(patch.T) + NOISE_FLOOR * np.eye(3)
    return patch.mean(0), np.linalg.inv(cov)


def fit_water_model(img_bgr, seed=None):
    """Learn what water looks like from a sample image (seed given, or auto-picked)."""
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    return water_model_from_seed(lab, seed or auto_seed(lab))


def sea_region(img_bgr, seed=None, model=None):
    """Water + enclosed ice as a mask.

    With no `model`, the water colour comes from a patch around the seed (auto-picked
    if not given) and the region grown is the one containing the seed. With a `model`
    from fit_water_model(), that fixed colour is looked for instead and the largest
    matching region is used, so a frame with no water simply yields a tiny/empty mask
    (this is how "is there water here?" gets answered without trusting a guessed seed).
    """
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    h, w, _ = lab.shape
    if model is None:
        seed = seed or auto_seed(lab)
        model = water_model_from_seed(lab, seed)
    mu, inv = model
    d = lab.reshape(-1, 3) - mu
    maha = np.sqrt(np.einsum("ij,jk,ik->i", d, inv, d)).reshape(h, w)

    water = (maha < WATER_DIST).astype(np.uint8) * 255
    water = cv2.morphologyEx(water, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    water = cv2.morphologyEx(water, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    n, labels, stats, _ = cv2.connectedComponentsWithStats(water)
    if seed is not None and labels[seed[1], seed[0]]:
        comp = (labels == labels[seed[1], seed[0]]).astype(np.uint8) * 255
    elif n > 1:
        comp = (labels == 1 + stats[1:, cv2.CC_STAT_AREA].argmax()).astype(np.uint8) * 255
    else:
        return np.zeros_like(water), water, seed
    cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sea = np.zeros_like(comp)
    cv2.drawContours(sea, [max(cnts, key=cv2.contourArea)], -1, 255, cv2.FILLED)
    return sea, comp, seed


def anomalies(img_bgr, sea):
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
    h, w, _ = lab.shape
    inside = sea > 0
    pix = lab[inside]
    rng = np.random.default_rng(0)
    gmm = GaussianMixture(N_COMPONENTS, covariance_type="full", random_state=0).fit(
        pix[rng.choice(len(pix), min(FIT_SAMPLES, len(pix)), replace=False)])
    score = np.zeros((h, w), np.float32)
    score[inside] = -gmm.score_samples(pix)
    score = cv2.GaussianBlur(score, (0, 0), SMOOTH_SIGMA)
    score[~inside] = 0

    thresh = np.percentile(score[inside], THRESH_PCTL)
    mask = (score > thresh).astype(np.uint8) * 255
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MERGE_KERNEL,) * 2))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    blobs = []
    for i in range(1, n):
        bx, by, bw, bh, area = stats[i]
        if MIN_AREA <= area <= MAX_AREA:
            blobs.append((float(score[labels == i].max()), (int(bx), int(by), int(bw), int(bh))))
    return sorted(blobs, reverse=True), score


def render(img, sea, seed, blobs, min_score):
    out = img.copy()
    tint = np.zeros_like(img)
    tint[sea > 0] = (255, 200, 0)
    out = cv2.addWeighted(out, 1.0, tint, 0.18, 0)
    cnts, _ = cv2.findContours(sea, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    thick = max(2, img.shape[1] // 700)
    cv2.drawContours(out, cnts, -1, (255, 200, 0), thick)
    cv2.drawMarker(out, seed, (0, 255, 255), cv2.MARKER_CROSS, 30, thick)
    scale = img.shape[1] / 1600
    for strength, (x, y, w, h) in blobs:
        if strength < min_score:
            continue
        p = 6
        cv2.rectangle(out, (x - p, y - p), (x + w + p, y + h + p), (0, 255, 0), thick + 1)
        cv2.putText(out, f"{strength:.0f}", (x - p, y - p - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (0, 255, 0), thick)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--seed", help="x,y of a point in open water (default: auto)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "output"))
    ap.add_argument("--min-score", type=float, default=0)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"could not read {args.image}")
    seed = tuple(int(v) for v in args.seed.split(",")) if args.seed else None
    sea, water, seed = sea_region(img, seed)
    blobs, _ = anomalies(img, sea)

    name = re.sub(r"\s+", "_", os.path.splitext(os.path.basename(args.image))[0])
    cv2.imwrite(os.path.join(args.out, f"{name}_sea.png"), render(img, sea, seed, blobs[:6], args.min_score))
    print(f"seed={seed}  sea covers {100 * (sea > 0).mean():.0f}% of frame "
          f"(water-only {100 * (water > 0).mean():.0f}%)")
    for s, b in blobs[:6]:
        print(f"  score={s:.1f} box={b}")


if __name__ == "__main__":
    main()
