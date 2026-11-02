#!/usr/bin/env python3
"""Plot a g2m_ribbon_census --dump text file as an SVG (top view).

    plot_dump.py DUMP.txt OUT.svg CENTRE_E CENTRE_N HALF_M [--z LO HI]

Each `tri` line of the dump is three vertices `E N z terr v flags verge u width lanes forward fade`.
Triangles are drawn filled by surface kind (bed / verge / patch / paint) or, with --z, by ribbon height between
LO and HI (blue low, red high). Grid lines every 10 m, the centre marked. Dev tool only; stdlib.
"""
import re
import sys

pat = re.compile(r"E ([\d.]+) N ([\d.]+) z ([\d.\-]+) terr ([\d.\-]+) v [\d.\-]+ fl (\S+) vg (\S+) u \S+ w \S+ ln (\S+)")


def kind(fl, vg, ln):
    if int(float(fl)) & (16 | 32):
        return "paint"
    if float(ln) == 0:
        return "jverge" if float(vg) > 0.5 else "patch"
    return "verge" if float(vg) > 0.5 else "bed"


COL = {"bed": "#555a66", "verge": "#8a8f5a", "patch": "#a05a8a", "jverge": "#c09ac0", "paint": "#f0f0f0"}


def zcol(z, lo, hi):
    t = max(0.0, min(1.0, (z - lo) / (hi - lo)))
    return "#%02x%02x%02x" % (int(255 * t), int(80 * (1 - abs(2 * t - 1))), int(255 * (1 - t)))


def main():
    src, out = sys.argv[1], sys.argv[2]
    ce, cn, half = float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5])
    zr = None
    if "--z" in sys.argv:
        i = sys.argv.index("--z")
        zr = (float(sys.argv[i + 1]), float(sys.argv[i + 2]))
    scale = 900.0 / (2 * half)
    tris = []
    for line in open(src, encoding="utf-8", errors="replace"):
        if not line.startswith("   tri"):
            continue
        vs = pat.findall(line)
        if len(vs) != 3:
            continue
        tris.append(vs)
    # Draw order: bed, verge below; patch above; paint last.
    order = {"verge": 0, "jverge": 1, "bed": 2, "patch": 3, "paint": 4}
    tris.sort(key=lambda v: order[kind(v[0][4], v[0][5], v[0][6])])
    o = ['<svg xmlns="http://www.w3.org/2000/svg" width="900" height="900" viewBox="0 0 900 900"><rect width="900" height="900" fill="#1b1d22"/>']
    for g in range(-int(half // 10) * 10, int(half) + 1, 10):
        x = (g + half) * scale
        o.append('<line x1="%.1f" y1="0" x2="%.1f" y2="900" stroke="#333" stroke-width="1"/><line x1="0" y1="%.1f" x2="900" y2="%.1f" stroke="#333" stroke-width="1"/>' % (x, x, x, x))
    zs = []
    for v in tris:
        pts = []
        for (e, n, z, t, fl, vg, ln) in v:
            zs.append(float(z))
            pts.append("%.1f,%.1f" % ((float(e) - ce + half) * scale, 900 - (float(n) - cn + half) * scale))
        k = kind(v[0][4], v[0][5], v[0][6])
        fill = zcol(sum(float(p[2]) for p in v) / 3, *zr) if zr and k != "paint" else COL[k]
        o.append('<polygon points="%s" fill="%s" fill-opacity="0.85" stroke="#000" stroke-opacity="0.25" stroke-width="0.4"/>' % (" ".join(pts), fill))
    cx, cy = half * scale, 900 - half * scale
    o.append('<circle cx="%.1f" cy="%.1f" r="6" fill="none" stroke="#ff0" stroke-width="2"/>' % (cx, cy))
    if zs:
        o.append('<text x="8" y="18" fill="#ddd" font-family="monospace" font-size="14">%d tris  z %.2f..%.2f  centre E %.1f N %.1f  grid 10 m</text>' % (len(tris), min(zs), max(zs), ce, cn))
    o.append("</svg>")
    open(out, "w", encoding="utf-8").write("\n".join(o))
    print(len(tris), "triangles; z range", (min(zs), max(zs)) if zs else None)


main()
