#!/usr/bin/env python3
"""Acquire a bounded GLO-30 extract, retaining exact bytes in a SHA256 lock.

First use: --bbox WEST SOUTH EAST NORTH --destination DIR (up to 9 degree files).
Later use: --destination DIR [--verify]; existing locks never silently repin.
Ocean/missing public tiles are recorded as unavailable, never interpreted as 0m.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import tempfile
import time
import urllib.error
import urllib.request

ORIGIN = 'https://copernicus-dem-30m.s3.amazonaws.com/'
LIMIT = 128 * 1024 * 1024

def tile_name(lat, lon):
    return f'Copernicus_DSM_COG_10_{"N" if lat >= 0 else "S"}{abs(lat):02d}_00_{"E" if lon >= 0 else "W"}{abs(lon):03d}_00_DEM'

def entries(bbox):
    west, south, east, north = bbox
    if not all(math.isfinite(x) for x in bbox) or not (-180 <= west < east <= 180 and -80 <= south < north <= 84):
        raise ValueError('bbox must be ordered within UTM coverage; split antimeridian extracts')
    rows = range(math.floor(south), math.ceil(north))
    cols = range(math.floor(west), math.ceil(east))
    if len(rows) * len(cols) > 9:
        raise ValueError('extract exceeds 9 degree files; use smaller regions')
    return [{'path': tile_name(y, x)+'.tif', 'url': ORIGIN+tile_name(y,x)+'/'+tile_name(y,x)+'.tif'} for y in rows for x in cols]

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()

def acquire(entry, destination, verify=False):
    name = entry['path']
    if Path(name).name != name or '/' in name or '\\' in name:
        raise ValueError('source path must be a basename')
    url = entry['url']
    if not url.startswith(ORIGIN) or '?' in url or '#' in url:
        raise ValueError('unexpected source URL')
    path = destination / name
    if entry.get('status') == 'unavailable':
        return entry
    pinned = 'sha256' in entry
    if pinned and path.is_file() and path.stat().st_size == entry['size'] and digest(path) == entry['sha256']:
        return entry
    if verify:
        raise ValueError('source bytes missing/changed: '+name)
    for attempt in range(3):
        temp = None
        try:
            request = urllib.request.Request(url, headers={'User-Agent':'geo2map-bounded-extract/1'})
            with urllib.request.urlopen(request, timeout=45) as response:
                if response.geturl() != url:
                    raise ValueError('unexpected source redirect')
                if response.status != 200:
                    raise ValueError('expected full source response')
                length = response.headers.get('Content-Length')
                if length and not 0 < int(length) <= LIMIT:
                    raise ValueError('source exceeds byte budget')
                h = hashlib.sha256(); size = 0
                with tempfile.NamedTemporaryFile(dir=destination, suffix='.part', delete=False) as f:
                    temp = Path(f.name)
                    while True:
                        chunk = response.read(min(1024*1024, LIMIT+1-size))
                        if not chunk:
                            break
                        size += len(chunk)
                        if size > LIMIT:
                            raise ValueError('source exceeds byte budget')
                        h.update(chunk); f.write(chunk)
                if size == 0 or (length and size != int(length)):
                    raise ValueError('truncated source response')
                if pinned and (size != entry['size'] or h.hexdigest() != entry['sha256']):
                    raise ValueError('upstream bytes changed; use a new extract/release')
                temp.replace(path)
                return dict(entry, status='available', size=size, sha256=h.hexdigest(), etag=response.headers.get('ETag'))
        except urllib.error.HTTPError as error:
            if error.code == 404 and not pinned:
                return dict(entry, status='unavailable')
            if error.code not in (429,500,502,503,504) or attempt == 2:
                raise
            time.sleep(2**attempt)
        except (TimeoutError, urllib.error.URLError):
            if attempt == 2:
                raise
            time.sleep(2**attempt)
        finally:
            if temp is not None and temp.exists():
                temp.unlink()

def run(destination, bbox=None, verify=False):
    destination.mkdir(parents=True, exist_ok=True)
    lock = destination / 'sources.json'
    if lock.exists():
        manifest = json.loads(lock.read_text(encoding='utf-8'))
        if manifest.get('format') != 'g2m.glo30_extract/1':
            raise ValueError('unknown source lock format')
        if bbox is not None and list(bbox) != manifest['bbox']:
            raise ValueError('existing extract has a different bbox; use another destination')
    else:
        if verify or bbox is None:
            raise ValueError('new extract requires --bbox')
        manifest = {'format':'g2m.glo30_extract/1','bbox':list(bbox),'vertical_reference':'EGM2008','surface':'DSM',
                    'source_documentation':ORIGIN+'readme.html',
                    'attribution':'Copernicus DEM; retain the Copernicus DEM licence and attribution when distributing extracts.',
                    'files':entries(bbox)}
    if not isinstance(manifest.get('files'), list) or not 0 < len(manifest['files']) <= 9:
        raise ValueError('invalid extract source count')
    for i, entry in enumerate(manifest['files']):
        manifest['files'][i] = acquire(entry, destination, verify)
        if not verify:
            temporary = lock.with_suffix('.json.part')
            temporary.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
            temporary.replace(lock)
    return manifest

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bbox', nargs=4, type=float)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    result = run(args.destination, args.bbox, args.verify)
    for item in result['files']:
        print(item['status'],item['path'],item.get('sha256',''))
