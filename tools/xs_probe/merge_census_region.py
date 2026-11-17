# Runs g2m_xs_probe --merge-census ... list over the whole home region in 10 row strips in parallel: python merge_census_region.py OUT_DIR
import subprocess, sys, os
from concurrent.futures import ThreadPoolExecutor
exe = r'S:\claude_code\geo2map_ramps\out\build\release\apps\g2m_tiler\g2m_xs_probe.exe'
out_dir = sys.argv[1]
os.makedirs(out_dir, exist_ok=True)
E0, E1 = 448512, 482304
N0, rows = 5406 * 1024, 50
strips = 10
per = rows // strips
def run(k):
    n0 = N0 + k * per * 1024
    n1 = n0 + per * 1024 - 0.001
    path = os.path.join(out_dir, f'strip{k:02d}.txt')
    with open(path, 'w') as f:
        r = subprocess.run([exe, r'S:\claude_code\geo2map_cache\home-r1', 'home-2026-09r1', 'utm32n/2/455/5421', '--merge-census',
                            str(E0), str(n0), str(E1 - 0.001), str(n1), '--merge-rules', 'on', 'list'], stdout=f, stderr=subprocess.PIPE, text=True)
    return k, r.returncode, r.stderr[-300:]
with ThreadPoolExecutor(strips) as ex:
    for k, rc, err in ex.map(run, range(strips)):
        print(k, rc, err.strip())
