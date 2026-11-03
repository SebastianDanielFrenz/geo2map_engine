#!/usr/bin/env python
"""imagery_qa.py - compare the served road model with DOP20 aerial photos (dev tool, not part of the build).

Needs numpy and Pillow. Work in a scratch directory (WORK) that holds, per 1 km DOP20 tile E_N (km, e.g. 465_5552):
  dop20_32_<E>_<N>_1_he.jpg   the photo (HVBG DOP20, 20 cm, 5000x5000, north up; pixel centre of the upper-left = E+0.1 / N+1000-0.1)
  tri_<E>_<N>.txt             g2m_ribbon_census --box ... --dump <cE> <cN> 800 output (road ribbon triangles with attributes)
Subcommands:
  raster  WORK E_N ...  tri_*.txt -> K_*.npy (0 none, 1 verge, 2 paved, 4 unpaved patch), L_*.npy (lanes; 0 = junction patch)
  shift   WORK E_N ...  translation by cross-correlation of the paved mask with the photo's asphalt score in 250 m blocks near roads
                        (median and spread over the blocks; validated: an imposed 1 m shift of the mask is recovered as 0.9 m).
                        Positive = the photo lies east/north of the model. Noise: ~1.2 m MAD per block, so use medians over many blocks.
  fit     WORK E_N ...  edge-profile fit (half-width bias only). NOT a translation estimator: the same fit run on a mask shifted by 1-2 m
                        returned the same dE (validated 2026-11-03), so its dE/dN are noise. The half-width bias is unvalidated.
  cells   WORK E_N ...  the 50 m cells with the largest local translation
  overlay WORK E_N cE cN half_m out.png [scale]   the model's paved outline (red) and verge (yellow) over the photo

Method of `fit` (see above for what it can and cannot do): along the model's paved boundary (every 4th pixel, outward normal from the blurred mask) the asphalt score of the
photo (low saturation, mid brightness, not vegetation) is sampled over +-2.4 m; the edge position is where the mean of the
inner 1.6 m minus the mean of the outer 1.6 m peaks (contrast > 0.25). A robust least squares fits s = w0 + dE*nx + dN*ny to
those offsets s (w0 half-width bias). Roofs, shadows and trees add about 1 m of noise per sample, so only aggregates over
thousands of samples are meaningful; a single cell is a lead to look at (overlay), not a measurement.
"""
import re
import sys

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None
PAT = re.compile(r"E ([\d.]+) N ([\d.]+) z ([\d.\-]+) terr ([\d.\-]+) v ([\d.\-]+) fl (\S+) vg (\S+) u (\S+) w (\S+) ln (\S+)")


def photo(work, e, n):
    return np.asarray(Image.open(f'{work}/dop20_32_{e}_{n}_1_he.jpg').convert('RGB'))


def kind(fl, vg, ln):
    if int(float(fl)) & (16 | 32):
        return 0                        # paint
    if float(ln) == 0 and int(float(fl)) & 8192:
        return 4                        # unpaved junction patch piece
    return 1 if float(vg) > 0.5 else 2  # verge / paved (a lanes=0 triangle without flags is a junction patch)


def raster(work, e, n, res=0.2):
    size = 1000.0
    m = int(size / res)
    K = np.zeros((m, m), np.uint8)
    L = np.zeros((m, m), np.uint8)
    e0, n0 = e * 1000, n * 1000
    for line in open(f'{work}/tri_{e}_{n}.txt', errors='replace'):
        if not line.startswith('   tri'):
            continue
        v = PAT.findall(line)
        if len(v) != 3:
            continue
        k = kind(v[0][5], v[0][6], v[0][9])
        if k == 0:
            continue
        P = np.array([[float(a[0]), float(a[1])] for a in v])
        if P[:, 0].max() < e0 or P[:, 0].min() > e0 + size or P[:, 1].max() < n0 or P[:, 1].min() > n0 + size:
            continue
        c0 = max(0, int((P[:, 0].min() - e0) / res))
        c1 = min(m - 1, int((P[:, 0].max() - e0) / res))
        r0 = max(0, int((n0 + size - P[:, 1].max()) / res))
        r1 = min(m - 1, int((n0 + size - P[:, 1].min()) / res))
        if c1 < c0 or r1 < r0:
            continue
        cc, rr = np.meshgrid(np.arange(c0, c1 + 1), np.arange(r0, r1 + 1))
        x = e0 + (cc + .5) * res
        y = n0 + size - (rr + .5) * res
        (x1, y1), (x2, y2), (x3, y3) = P
        d = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
        if abs(d) < 1e-9:
            continue
        a = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / d
        b = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / d
        inside = (a >= -1e-9) & (b >= -1e-9) & (1 - a - b >= -1e-9)
        sub = K[r0:r1 + 1, c0:c1 + 1]
        upd = inside & (k >= sub)
        sub[upd] = k
        L[r0:r1 + 1, c0:c1 + 1][upd] = min(int(float(v[0][9])), 255)
    np.save(f'{work}/K_{e}_{n}.npy', K)
    np.save(f'{work}/L_{e}_{n}.npy', L)


def box_blur(a, k):
    r = k // 2
    p = np.pad(a, ((r, r), (r, r)), mode='edge')
    c = np.cumsum(p, 0, dtype=np.float64)
    c = np.vstack([np.zeros((1, c.shape[1])), c])
    a1 = c[k:] - c[:-k]
    c = np.cumsum(a1, 1)
    c = np.hstack([np.zeros((c.shape[0], 1)), c])
    a2 = c[:, k:] - c[:, :-k]
    return (a2 / (k * k)).astype(np.float32)


def asphalt_score(im):
    f = im.astype(np.float32)
    mx = f.max(2)
    mn = f.min(2)
    grey = np.clip(1 - ((mx - mn) / np.maximum(mx, 1)) / 0.25, 0, 1)
    bright = np.clip((mx - 40) / 30, 0, 1) * np.clip((185 - mx) / 30, 0, 1)
    veg = np.clip((2 * f[:, :, 1] - f[:, :, 0] - f[:, :, 2] - 10) / 25, 0, 1)
    return grey * bright * (1 - veg)


def samples(work, e, n):
    K = np.load(f'{work}/K_{e}_{n}.npy')
    L = np.load(f'{work}/L_{e}_{n}.npy')
    A = box_blur(asphalt_score(photo(work, e, n)), 3)
    Mp = (K == 2).astype(np.float32)
    gy, gx = np.gradient(box_blur(Mp, 9))
    bnd = (Mp > 0.5) & (box_blur(Mp, 3) < 0.99)
    rr, cc = np.nonzero(bnd)
    sel = (rr % 4 == 0) & (cc % 4 == 0)
    rr, cc = rr[sel], cc[sel]
    nrm = np.hypot(gx[rr, cc], gy[rr, cc])
    ok = nrm > 1e-4
    rr, cc = rr[ok], cc[ok]
    nx = -gx[rr, cc] / nrm[ok]
    ny = -gy[rr, cc] / nrm[ok]
    offs = np.arange(-12, 13)
    H, W = A.shape
    prof = np.empty((len(rr), len(offs)), np.float32)
    for k, o in enumerate(offs):
        prof[:, k] = A[np.clip(np.round(rr + ny * o).astype(int), 0, H - 1), np.clip(np.round(cc + nx * o).astype(int), 0, W - 1)]
    w = 8
    cs = np.cumsum(np.concatenate([np.zeros((len(rr), 1)), prof], 1), 1)
    cand = list(range(w, len(offs) - w + 1))
    sc = np.stack([(cs[:, k] - cs[:, k - w]) / w - (cs[:, k + w] - cs[:, k]) / w for k in cand], 1)
    s = (np.array(cand)[sc.argmax(1)] - 12) * 0.2
    con = sc.max(1)
    good = (con > 0.25) & (np.abs(s) < 2.2)
    return dict(s=s[good], nx=nx[good], ny=ny[good], lanes=L[rr, cc][good], rr=rr[good], cc=cc[good], e=e, n=n)


def fit(s, nx, ny):
    X = np.stack([np.ones(len(s)), nx, -ny], 1)
    w = np.ones(len(s))
    for _ in range(10):
        b = np.linalg.lstsq(X * w[:, None], s * w, rcond=None)[0]
        r = s - X @ b
        sd = 1.4826 * np.median(np.abs(r)) + 1e-3
        w = 1 / np.maximum(1, np.abs(r) / (1.5 * sd))
    return b, sd


def cmd_fit(work, tiles):
    allv = []
    for e, n in tiles:
        d = samples(work, e, n)
        allv.append(d)
        b, sd = fit(d['s'], d['nx'], d['ny'])
        print(f'tile {e}/{n}: n={len(d["s"])} half-width {b[0]:+.2f} m  dE {b[1]:+.2f}  dN {b[2]:+.2f}  robust sd {sd:.2f}')
    s, nx, ny, ln = (np.concatenate([d[k] for d in allv]) for k in ('s', 'nx', 'ny', 'lanes'))
    b, sd = fit(s, nx, ny)
    print(f'ALL: n={len(s)} half-width {b[0]:+.2f}  dE {b[1]:+.2f}  dN {b[2]:+.2f}  sd {sd:.2f}')
    for lanes in sorted(set(ln.tolist())):
        m = ln == lanes
        if m.sum() >= 150:
            b, sd = fit(s[m], nx[m], ny[m])
            tag = ' (junction patch)' if lanes == 0 else ''
            print(f'  lanes={lanes}{tag}: n={m.sum()} half-width {b[0]:+.2f} m  dE {b[1]:+.2f}  dN {b[2]:+.2f}  sd {sd:.2f}')


def cmd_shift(work, tiles, block=1250):
    allr = []
    for e, n in tiles:
        K = np.load(f'{work}/K_{e}_{n}.npy')
        A = asphalt_score(photo(work, e, n))
        M = ((K == 2) | (K == 4)).astype(np.float32)
        near = box_blur(M, 61) > 0.002
        res = []
        for br in range(0, K.shape[0], block):
            for bc in range(0, K.shape[1], block):
                z = np.zeros_like(near)
                z[br:br + block, bc:bc + block] = True
                w = (near & z)
                if w.sum() < 20000:
                    continue
                wf = w.astype(np.float32)
                a = (A - A[w].mean()) * wf
                m = (M - M[w].mean()) * wf
                c = np.fft.fftshift(np.fft.irfft2(np.fft.rfft2(a) * np.conj(np.fft.rfft2(m)), s=a.shape))
                h, mx = a.shape[0] // 2, 30
                sub = c[h - mx:h + mx + 1, h - mx:h + mx + 1]
                r, k = np.unravel_index(sub.argmax(), sub.shape)

                def par(y0, y1, y2):
                    d = y0 - 2 * y1 + y2
                    return 0 if d == 0 else 0.5 * (y0 - y2) / d
                dx = k + (par(sub[r, k - 1], sub[r, k], sub[r, k + 1]) if 0 < k < 2 * mx else 0) - mx
                dy = r + (par(sub[r - 1, k], sub[r, k], sub[r + 1, k]) if 0 < r < 2 * mx else 0) - mx
                res.append((dx * .2, -dy * .2))
        R = np.array(res)
        allr.append(R)
        print(f'tile {e}/{n}: {len(R)} blocks, median dE {np.median(R[:, 0]):+.2f} dN {np.median(R[:, 1]):+.2f}, '
              f'MAD {np.median(abs(R[:, 0] - np.median(R[:, 0]))):.2f} / {np.median(abs(R[:, 1] - np.median(R[:, 1]))):.2f}')
    R = np.concatenate(allr)
    print(f'ALL: {len(R)} blocks, median dE {np.median(R[:, 0]):+.2f} dN {np.median(R[:, 1]):+.2f} (std error of the median ~ 1.25 * MAD / sqrt(n))')


def cmd_cells(work, tiles, cell=250):
    rows = []
    for e, n in tiles:
        d = samples(work, e, n)
        ci = d['cc'] // cell
        ri = d['rr'] // cell
        for a in np.unique(ri * 100 + ci):
            m = (ri * 100 + ci) == a
            if m.sum() < 60:
                continue
            b, sd = fit(d['s'][m], d['nx'][m], d['ny'][m])
            rows.append((e * 1000 + (a % 100 + .5) * cell * .2, (n + 1) * 1000 - (a // 100 + .5) * cell * .2, int(m.sum()), b[0], b[1], b[2], sd))
    rows.sort(key=lambda r: -np.hypot(r[4], r[5]))
    print('cells', len(rows), '| translation > 0.5 m:', sum(np.hypot(r[4], r[5]) > .5 for r in rows), '> 1 m:', sum(np.hypot(r[4], r[5]) > 1 for r in rows))
    for r in rows[:15]:
        print('E %.0f N %.0f n=%d halfw %+.2f dE %+.2f dN %+.2f sd %.2f' % r)


def cmd_overlay(work, e, n, ce, cn, half, out, sc=3):
    K = np.load(f'{work}/K_{e}_{n}.npy')
    im = photo(work, e, n)
    c0 = int((ce - half - e * 1000) / .2)
    c1 = int((ce + half - e * 1000) / .2)
    r0 = int(((n + 1) * 1000 - (cn + half)) / .2)
    r1 = int(((n + 1) * 1000 - (cn - half)) / .2)
    crop = im[r0:r1, c0:c1].copy()
    k = K[r0:r1, c0:c1]

    def edge(m):
        o = np.zeros_like(m, bool)
        o[1:, :] |= m[1:, :] != m[:-1, :]
        o[:, 1:] |= m[:, 1:] != m[:, :-1]
        return o
    crop[edge(k == 1)] = (255, 255, 0)
    crop[edge(k == 2)] = (255, 0, 0)
    Image.fromarray(crop).resize(((c1 - c0) * sc, (r1 - r0) * sc), Image.NEAREST).save(out)


def main(argv):
    if len(argv) < 4:
        sys.exit(__doc__)
    cmd, work = argv[1], argv[2]

    def tile(t):
        return tuple(int(x) for x in t.split('_'))
    if cmd == 'raster':
        for t in argv[3:]:
            raster(work, *tile(t))
    elif cmd == 'fit':
        cmd_fit(work, [tile(t) for t in argv[3:]])
    elif cmd == 'shift':
        cmd_shift(work, [tile(t) for t in argv[3:]])
    elif cmd == 'cells':
        cmd_cells(work, [tile(t) for t in argv[3:]])
    elif cmd == 'overlay':
        cmd_overlay(work, *tile(argv[3]), float(argv[4]), float(argv[5]), float(argv[6]), argv[7], int(argv[8]) if len(argv) > 8 else 3)
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    main(sys.argv)
