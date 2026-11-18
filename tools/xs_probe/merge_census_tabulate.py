# Tabulates the reason lines of merge_census_region.py: python merge_census_tabulate.py OUT_DIR reasons.txt
import glob, re, sys, collections
src, out = sys.argv[1], sys.argv[2]
tot = collections.Counter()
keys = ['tiles', 'degree3_nodes', 'candidates', 'landed', 'candidates_with_lane_taper', 'deg2_lane_count_changes', 'deg2_with_taper']
sub = collections.Counter()
rows = collections.defaultdict(list)
for f in sorted(glob.glob(src + '/strip*.txt')):
    for line in open(f):
        if line.startswith('merge_census'):
            for k in keys:
                tot[k] += int(re.search(r'\b' + k + r'=(\d+)', line).group(1))
            m = re.search(r'\(merge (\d+), diverge (\d+)\) landed=\d+ \(merge (\d+), diverge (\d+)\)', line)
            for name, v in zip(['cand_merge', 'cand_diverge', 'landed_merge', 'landed_diverge'], m.groups()):
                sub[name] += int(v)
        elif line.startswith('reason '):
            p = line.split()
            kind, d, code = p[1], p[2], p[3]
            node, e, n = p[5], p[7], p[9]
            rest = ' '.join(p[10:])
            rows[(kind, d, code)].append((node, e, n, rest))
L = []
L.append('Merge-census reason table, whole home region (store home-r1, scope home-2026-09r1, L2 tiles x 438..470, y 5406..5455,')
L.append('E 448512-482304, N 5535744-5586944), branch claude/ramp-lanes stage 0, rules on. Behaviour = main 4f02b77.')
L.append('')
L.append('Totals: ' + ' '.join(f'{k}={tot[k]}' for k in keys))
L.append(f"  candidates merge {sub['cand_merge']} / diverge {sub['cand_diverge']}; landed merge {sub['landed_merge']} / diverge {sub['landed_diverge']}")
L.append('')
L.append('KIND: cand_landing = candidate (one-way arms, single = through + ramp counted lanes, a *_link beside the single arm) without a')
L.append('      landing; cand_taper = candidate whose through/single join is not a LaneChange join (the census\'s "taper");')
L.append('      near = degree-3 node with a *_link arm that is no candidate (classification refusal); d2 = degree-2 lane-count')
L.append('      change whose join kind is not LaneChange.')
L.append('CODE: merge_<x> = classify_merge_node refused with x; join_<kind>_ramped|_unramped = a join was resolved with that kind,')
L.append('      with or without band width ramps at that end; stretch_fit_declined:<msg> = the arm\'s stretch has no reference')
L.append('      line (fit declined), so no section at all; landing_* = resolve_cross_section skipped the landing.')
L.append('      lane_sum_single_eq_max = the lone arm has as many lanes as the larger of the other two (ramp lane not added).')
L.append('')
for kind in ['cand_landing', 'cand_taper', 'near', 'd2']:
    for d in ['merge', 'diverge', '-']:
        sel = sorted(((k, v) for k, v in rows.items() if k[0] == kind and k[1] == d), key=lambda kv: (-len(kv[1]), kv[0][2]))
        if not sel: continue
        total = sum(len(v) for _, v in sel)
        L.append(f'== {kind} {d}: {total}')
        for (k, d2, code), v in sel:
            ex = '; '.join(f'{a} ({float(e):.1f}, {float(n):.1f}) {r}' for a, e, n, r in v[:3])
            L.append(f'  {len(v):5d}  {code}')
            L.append(f'         e.g. {ex}')
        L.append('')
open(out, 'w').write('\n'.join(L) + '\n')
print('\n'.join(L[:6]))
