"""Draw served junction rings over the approach edges (the dump of tests/layer/test_junction_dump.cpp).

usage: python render.py DUMP.json OUT_DIR [--pick N] [--node ID ...]
Writes one PNG per junction: approach road beds in grey, paved band edges thin, the junction road bed ring
filled dark, paved ring and outer ring as outlines, OSM nodes as dots.
"""
import json, sys, os, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

def main():
    dump, out = sys.argv[1], sys.argv[2]
    args = sys.argv[3:]
    pick = int(args[args.index("--pick") + 1]) if "--pick" in args else 12
    nodes = [int(a) for a in args[args.index("--node") + 1:]] if "--node" in args else None
    d = json.load(open(dump))
    os.makedirs(out, exist_ok=True)
    js = d["junctions"]
    if nodes:
        js = [j for j in js if j["nodes"][0] in nodes]
    else:
        # spread over sizes: sort by outer ring area, take evenly spaced ones
        def area(r):
            return abs(sum(r[i][0] * r[(i + 1) % len(r)][1] - r[(i + 1) % len(r)][0] * r[i][1] for i in range(len(r)))) / 2e6
        js = sorted(js, key=lambda j: area(j["rings"][2]))
        step = max(1, len(js) // pick)
        js = js[::step][:pick]
    ents = d["entries"]
    for j in js:
        ring = [(x / 1000, y / 1000) for x, y in j["rings"][2]]
        cx = sum(p[0] for p in ring) / len(ring); cy = sum(p[1] for p in ring) / len(ring)
        half = max(max(abs(p[0] - cx), abs(p[1] - cy)) for p in ring) + 15
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.set_facecolor("#8fae6b")
        for e in ents:
            rl, rr, pl, pr = e["edges"]
            pts = rl + rr[::-1]
            if not any(abs(p[0] - cx) < half * 1.5 and abs(p[1] - cy) < half * 1.5 for p in pts):
                continue
            ax.add_patch(Polygon(pl + pr[::-1], closed=True, fc="#c9c3b5", ec="none", zorder=1))
            ax.add_patch(Polygon(pts, closed=True, fc="#5d5d62", ec="#333", lw=0.4, zorder=2))
        for k, (fc, ec, z) in enumerate([("#3f3f44", "#ff3", 4), (None, "#0af", 5), (None, "#f60", 6)]):
            r = [(x / 1000, y / 1000) for x, y in j["rings"][k]]
            ax.add_patch(Polygon(r, closed=True, fc=fc if fc else "none", ec=ec, lw=1.2, zorder=z, alpha=0.9 if fc else 1))
        ax.set_xlim(cx - half, cx + half); ax.set_ylim(cy - half, cy + half); ax.set_aspect("equal")
        ax.set_title(f"node {j['nodes'][0]} ({len(j['nodes'])} nodes, flags {j['flags']}, {len(j['approaches'])} approaches)", fontsize=9)
        ax.set_xticks([]); ax.set_yticks([])
        fig.savefig(os.path.join(out, f"j_{j['nodes'][0]}.png"), dpi=70, bbox_inches="tight")
        plt.close(fig)

if __name__ == "__main__":
    main()
