#!/usr/bin/env python
"""Fetch the soil source files for one region (g2m.src.soil input).  Python standard library only.

  python -I fetch_soilgrids.py --bbox LON0 LAT0 LON1 LAT1 --out DIR

Writes into DIR (a new, empty folder outside the source tree, e.g. S:\\geo_data\\soil\\soilgrids_rmh):

  <prop>_<depth>_mean.tif   SoilGrids 2.0 (ISRIC, CC BY 4.0), 7 properties x 6 depths, int16, EPSG:4326, 1/480 degree
                            cells aligned to multiples of 1/480 degree, fetched from the ISRIC WCS
                            (maps.isric.org).  The server reprojects the native Interrupted-Goode-Homolosine 250 m
                            cells to the requested geographic grid by nearest neighbour; cells outside the soil mask
                            (water, built-up) come back as 0.
  manifest.json             URLs, request parameters, SHA-256 and size of every file, fetch date.

Nothing here needs an account; both servers answer anonymously.  The importer (`g2m_tiler import-soil`) reads the
files; this script is not part of any build.
"""
import argparse, hashlib, json, math, os, time, urllib.request

PROPS = ["sand", "silt", "clay", "soc", "phh2o", "cfvo", "bdod"]
DEPTHS = ["0-5cm", "5-15cm", "15-30cm", "30-60cm", "60-100cm", "100-200cm"]
N = 480  # cells per degree of the requested grid
WCS = ("https://maps.isric.org/mapserv?map=/map/{p}.map&SERVICE=WCS&VERSION=2.0.1&REQUEST=GetCoverage"
       "&COVERAGEID={p}_{d}_mean&FORMAT=image/tiff&SUBSET=long({x0},{x1})&SUBSET=lat({y0},{y1})"
       "&SUBSETTINGCRS=http://www.opengis.net/def/crs/EPSG/0/4326&OUTPUTCRS=http://www.opengis.net/def/crs/EPSG/0/4326"
       "&SCALESIZE=long({w}),lat({h})")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def get(url, rng=None, tries=4):
    for i in range(tries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "geo2map-soil-fetch/1"})
            if rng:
                req.add_header("Range", "bytes=%d-%d" % rng)
            with urllib.request.urlopen(req, timeout=180) as r:
                return r.read()
        except Exception:
            if i == tries - 1:
                raise
            time.sleep(2 * (i + 1))


def fetch_wcs(bbox, out, manifest):
    lon0, lat0, lon1, lat1 = bbox
    eps = 1e-6  # 8.2 * 480 is 3935.9999999999995 in binary floating point
    c0, c1 = math.floor(lon0 * N + eps), math.ceil(lon1 * N - eps)
    r0, r1 = math.floor(lat0 * N + eps), math.ceil(lat1 * N - eps)
    x0, x1, y0, y1 = c0 / N, c1 / N, r0 / N, r1 / N
    w, h = c1 - c0, r1 - r0
    for p in PROPS:
        for d in DEPTHS:
            name = "%s_%s_mean.tif" % (p, d)
            path = os.path.join(out, name)
            url = WCS.format(p=p, d=d, x0=repr(x0), x1=repr(x1), y0=repr(y0), y1=repr(y1), w=w, h=h)
            if not os.path.exists(path):
                data = get(url)
                if data[:4] not in (b"II*\x00", b"MM\x00*"):
                    raise RuntimeError("not a TIFF: " + name + " " + data[:200].decode("latin1"))
                with open(path, "wb") as f:
                    f.write(data)
            manifest["files"].append({"name": name, "url": url, "sha256": sha256(path), "bytes": os.path.getsize(path)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bbox", nargs=4, type=float, required=True, metavar=("LON0", "LAT0", "LON1", "LAT1"))
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    manifest = {"format": "g2m.soil_fetch/1", "bbox": a.bbox, "files": [],
                "fetched": time.strftime("%Y-%m-%d", time.gmtime()),
                "sources": {"soilgrids2": "ISRIC SoilGrids 250 m v2.0, CC BY 4.0, https://maps.isric.org (WCS)"}}
    fetch_wcs(a.bbox, a.out, manifest)
    with open(os.path.join(a.out, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=1)
    print("ok", len(manifest["files"]), "files")


if __name__ == "__main__":
    main()
