"""Render the fixed view set for one stage: python render_views.py STAGE_NAME
Same views, framing and line-width factor every stage (BRIEF procedure)."""
import subprocess, sys, os
from concurrent.futures import ThreadPoolExecutor
stage = sys.argv[1]
wt = r'S:\claude_code\geo2map_ramps'
exe = wt + r'\out\build\release\apps\g2m_tiler\g2m_ribbon_census.exe'
shade = wt + r'\tools\ribbon_census\shade.py'
out = r'S:\claude_code\physics_sim\out\mtz_ramps\after' + '\\' + stage
dumps = wt + r'\out\views\\' + stage
os.makedirs(out, exist_ok=True); os.makedirs(dumps, exist_ok=True)
LW = '1.5'
VIEWS = [  # file, centre E, N, half
    ('1_a66_lane_add_N', 466815, 5551675, 60),
    ('2_a66_exit_and_onramp_G', 466490, 5551590, 90),
    ('3_b8_over_a66_bridge', 466630, 5551575, 60),
    ('4_b8_split_M_I', 466520, 5551710, 75),
    ('5_b8_signal_junction_L', 466690, 5551475, 60),
    ('6_mtz_split_A_D', 465905, 5552100, 75),
    ('7_mtz_overlap', 465880, 5552170, 60),
    ('8_b8_exit_F', 465920, 5552380, 75),
    ('9_out_w_landing_far', 451790, 5555100, 90),
    ('10_out_sw_lanes_541', 457800, 5543580, 90),
    ('11_out_sw_offset_hold', 455750, 5545000, 90),
]
def one(v):
    name, e, n, h = v
    dump = os.path.join(dumps, name + '.txt')
    derived = os.path.join(wt, 'out', 'derived', stage, name)
    os.makedirs(derived, exist_ok=True)
    with open(dump, 'w') as f:
        r = subprocess.run([exe, r'S:\claude_code\geo2map_cache\home-r1', 'home-2026-09r1', derived, '32',
                            '--box', str(e - h - 20), str(n - h - 20), str(e + h + 20), str(n + h + 20),
                            '--drape', 'lattice', '--dump', str(e), str(n), str(h * 1.5)], stdout=f, stderr=subprocess.PIPE, text=True)
    if r.returncode: return name, 'census rc %d %s' % (r.returncode, r.stderr[-200:])
    r2 = subprocess.run([sys.executable, shade, dump, os.path.join(out, name + '.png'), str(e), str(n), str(h), LW], capture_output=True, text=True)
    return name, 'ok' if r2.returncode == 0 else 'shade rc %d %s' % (r2.returncode, r2.stderr[-300:])
with ThreadPoolExecutor(6) as ex:
    for name, st in ex.map(one, VIEWS):
        print(name, st)
