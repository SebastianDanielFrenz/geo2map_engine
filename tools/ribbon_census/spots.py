"""Stage-0c diagnostics: read the records of `g2m_ribbon_census ... --box E0 N0 E1 N1 --spots` (format in spots.cpp).

  python spots.py RECORDS [--view NAME E N HALF] [--summary]

Prints per stretch end in the view the served join, bands, ramps, offset and the ribbon's road bed at 0/5/10/20/40 m
from that end; marks square steps (a road-bed edge that moves more than 0.5 m within 1 m of travel, inside a stretch or
across a join) and slivers (two road beds overlapping more than 0.5 m wide over more than 10 m, from the OV records).
`import spots` gives the same parse for shade.py's annotation overlay.
"""
import bisect, collections, math, sys

KIND = ['driving', 'turn', 'bus', 'cycle', 'parking', 'shoulder', 'verge', 'sidewalk']
DIRN = ['fwd', 'bwd', 'both', 'none']
STEP_M = 0.5


def parse(path):
    d = {'ways': {}, 'entries': {}, 'nodes': {}, 'ovs': [], 'ends': [], 'landed': {}}
    E = d['entries']
    def ent(w, sr):
        return E.setdefault((w, sr), {'way': w, 'sr': sr, 'bands': [], 'rows': [], 'ends': {}, 'off': None, 'len': 0, 'hw': '?', 'counted': 0, 'xs': False})
    for line in open(path, encoding='utf-8', errors='replace'):
        p = line.split()
        if not p: continue
        t = p[0]
        if t == 'W':
            n = int(p[5]); pts = [(float(p[6 + 2 * i]), float(p[7 + 2 * i])) for i in range(n)]
            d['ways'][int(p[1])] = {'hw': p[2], 'lanes': p[3], 'oneway': p[4], 'pts': pts}
        elif t == 'X':
            e = ent(int(p[1]), int(p[2])); e['er'] = int(p[3]); e['len'] = float(p[4]); e['hw'] = p[5]; e['counted'] = int(p[6]); e['xs'] = p[7] == 'xs'
        elif t == 'B':
            ent(int(p[1]), int(p[2]))['bands'].append({'kind': KIND[int(p[4])], 'dir': DIRN[int(p[5])], 'counted': p[6] == '1', 'w': int(p[7]),
                                                      'r': [(int(p[8]), int(p[9])), (int(p[10]), int(p[11]))]})
        elif t == 'O':
            ent(int(p[1]), int(p[2]))['off'] = {'mode': p[3], 'mm': [int(p[4]), int(p[6])], 'len': [int(p[5]), int(p[7])]}
        elif t == 'END':
            e = ent(int(p[1]), int(p[2])); at = int(p[3])
            rec = {'way': int(p[1]), 'sr': int(p[2]), 'at': at, 'node': int(p[4]), 'deg': int(p[5]), 'E': float(p[6]), 'N': float(p[7]),
                   'kind': p[8], 'nb': int(p[9]), 'delta': int(p[10])}
            e['ends'][at] = rec; d['ends'].append(rec)
        elif t == 'R':
            ent(int(p[1]), int(p[2]))['rows'].append({'v': float(p[3]), 'ref': (float(p[4]), float(p[5])), 'lo': float(p[6]), 'hi': float(p[7]),
                                                     'l': (float(p[8]), float(p[9])), 'r': (float(p[10]), float(p[11])), 'w': float(p[12]), 'ln': float(p[13]), 'fl': int(float(p[14]))})
        elif t == 'NODE':
            d['nodes'][int(p[1])] = {'E': float(p[2]), 'N': float(p[3]), 'deg': int(p[4]), 'codes': p[5:]}
        elif t == 'E' and len(p) >= 19:   # a landed merge node with all three arms (spots.cpp, ROADS stage 3a)
            d['landed'][int(p[1])] = {'S': int(p[3]), 'T': int(p[4]), 'R': int(p[5]), 'dLane': (int(p[11]), int(p[12])),
                                      'dBed': (int(p[13]), int(p[14])), 'rside': p[18]}
        elif t == 'OV':
            d['ovs'].append({'a': (int(p[1]), int(p[2])), 'b': (int(p[3]), int(p[4])), 's0': int(p[5]), 's1': int(p[6]), 'maxw': float(p[7]), 'meanw': float(p[8]),
                             'E': float(p[9]), 'N': float(p[10])})
    for e in E.values():
        rows = {}
        for r in e['rows']: rows[round(r['v'], 3)] = r   # a row owned by two tiles' records only once
        e['rows'] = [rows[k] for k in sorted(rows)]
        e['vs'] = [r['v'] for r in e['rows']]
    return d


def ramp_len(e, at):
    """Longest band width ramp at that end (m)."""
    return max([b['r'][at][0] for b in e['bands']] + [0]) / 1000.0


def offset_len(e, at):
    o = e['off']
    if not o or o['mode'] == 'none': return 0.0
    if o['mode'] in ('hold', 'transition'): return e['len']
    return o['len'][at] / 1000.0


def sample(e, s_from_end, at):
    """The road-bed row at s metres from an end (linear between rows); None outside the rows."""
    if not e['rows']: return None
    v = s_from_end if at == 0 else e['len'] - s_from_end
    vs = e['vs']
    if v < vs[0] - 0.05 or v > vs[-1] + 0.05: return None
    i = bisect.bisect_left(vs, v)
    if i <= 0: return e['rows'][0]
    if i >= len(vs): return e['rows'][-1]
    a, b = e['rows'][i - 1], e['rows'][i]
    f = 0 if b['v'] == a['v'] else (v - a['v']) / (b['v'] - a['v'])
    return {'v': v, 'lo': a['lo'] + f * (b['lo'] - a['lo']), 'hi': a['hi'] + f * (b['hi'] - a['hi']), 'w': a['w'] + f * (b['w'] - a['w']),
            'ln': a['ln'] if f < 0.5 else b['ln'], 'fl': a['fl'] if f < 0.5 else b['fl']}


def interior_steps(e):
    """Consecutive rows (at most 1 m apart) whose road-bed edges move by more than STEP_M."""
    out = []
    rows = e['rows']
    for a, b in zip(rows, rows[1:]):
        if b['v'] - a['v'] > 1.05: continue
        jump = max(abs(b['lo'] - a['lo']), abs(b['hi'] - a['hi']))
        if jump > STEP_M:
            out.append({'v': (a['v'] + b['v']) / 2, 'jump': jump, 'dw': (b['hi'] - b['lo']) - (a['hi'] - a['lo']), 'pos': a['ref']})
    return out


def end_row(e, at):
    if not e['rows']: return None
    r = e['rows'][0] if at == 0 else e['rows'][-1]
    v_end = 0 if at == 0 else e['len']
    return r if abs(r['v'] - v_end) < 0.15 else None


def joint_steps(d):
    """At a node where two stretch ends meet (degree 2, or the through pair of a join): the road-bed edge points of the
    two end rows, paired by orientation; a step when either pair is more than STEP_M apart. At a landed merge node (an E
    record; ROADS stage 3b) the single arm S is compared with the through arm T and the ramp R together: a step when S's
    outer road-bed edge on either side lies more than STEP_M from that of T and R (the E record's dBedL/dBedR)."""
    E = d['entries']
    out, seen = [], set()
    by_node = collections.defaultdict(list)
    for x in d['ends']: by_node[x['node']].append(x)
    for node, rec in d.get('landed', {}).items():
        jump = max(abs(v) for v in rec['dBed']) / 1000.0
        if jump > STEP_M:
            x = next((e for e in by_node.get(node, []) if e['way'] == rec['S']), None) or {'way': rec['S'], 'kind': '?', 'deg': 3, 'E': 0.0, 'N': 0.0}
            out.append({'node': node, 'a': x, 'b': {'way': rec['T'], 'kind': '?', 'deg': 3}, 'jump': jump, 'wa': 0.0, 'wb': 0.0, 'landed': True,
                        'E': x['E'], 'N': x['N']})
    for node, ends in by_node.items():
        if node in d.get('landed', {}): continue
        for x in ends:
            if x['nb'] == 0 or x['kind'] == 'landing': continue
            for y in ends:
                if y is x or y['way'] != x['nb']: continue
                key = tuple(sorted([(x['way'], x['sr'], x['at']), (y['way'], y['sr'], y['at'])]))
                if key in seen: continue
                seen.add(key)
                ea, eb = E[(x['way'], x['sr'])], E[(y['way'], y['sr'])]
                ra, rb = end_row(ea, x['at']), end_row(eb, y['at'])
                if not ra or not rb: continue
                dist = lambda p, q: math.hypot(p[0] - q[0], p[1] - q[1])
                # same travel direction across the node: A's end meets B's start (or the reverse) -> left with left
                same = x['at'] != y['at']
                pairs = [(ra['l'], rb['l']), (ra['r'], rb['r'])] if same else [(ra['l'], rb['r']), (ra['r'], rb['l'])]
                jump = max(dist(*pairs[0]), dist(*pairs[1]))
                if jump > STEP_M:
                    out.append({'node': node, 'a': x, 'b': y, 'jump': jump, 'wa': ra['hi'] - ra['lo'], 'wb': rb['hi'] - rb['lo'],
                                'E': x['E'], 'N': x['N']})
    return out


def sliver_cause(d, ov):
    E = d['entries']
    a, b = E.get(ov['a']), E.get(ov['b'])
    if not a or not b: return 'entry missing'
    na = {x['node']: x for x in a['ends'].values()}
    nb = {x['node']: x for x in b['ends'].values()}
    shared = set(na) & set(nb)
    if shared:
        n = sorted(shared)[0]
        ka, kb = na[n]['kind'], nb[n]['kind']
        lab = 'shared node %d (%s/%s, deg %d)' % (n, ka, kb, na[n]['deg'])
        if 'landing' in (ka, kb): lab += ': ramp landed beside its single arm'
        return lab
    # a common neighbour within one join
    return 'no shared node (%s %s / %s %s)' % (a['hw'], d['ways'].get(a['way'], {}).get('oneway', '?'), b['hw'], d['ways'].get(b['way'], {}).get('oneway', '?'))


def fmt_bands(e, at):
    s = []
    for b in e['bands']:
        r = b['r'][at]
        s.append('%s%s%s %.2f%s' % (b['kind'][:2], b['dir'][0], '*' if b['counted'] else '', b['w'] / 1000.0,
                                     ' ramp %.0f m from %.2f' % (r[0] / 1000.0, r[1] / 1000.0) if r[0] else ''))
    return '; '.join(s)


def view_report(d, name, ce, cn, half, steps_j, out):
    E = d['entries']
    inside = lambda x, y: abs(x - ce) <= half and abs(y - cn) <= half
    out.append('== %s  (centre %.0f %.0f, half %d m)' % (name, ce, cn, half))
    ends = sorted([x for x in d['ends'] if inside(x['E'], x['N'])], key=lambda x: (x['node'], x['way'], x['sr'], x['at']))
    for x in ends:
        e = E[(x['way'], x['sr'])]
        w = d['ways'].get(x['way'], {})
        o = e['off'] or {'mode': '-', 'mm': [0, 0], 'len': [0, 0]}
        out.append('  way %d %s lanes=%s oneway=%s  %s at node %d (deg %d)  counted %d  len %.1f m' % (
            x['way'], e['hw'], w.get('lanes', '?'), w.get('oneway', '?'), 'end' if x['at'] else 'start', x['node'], x['deg'], e['counted'], e['len']))
        out.append('    join %s neighbour %d delta %d mm  landing %s  offset %s start %d mm/%d mm end %d mm/%d mm' % (
            x['kind'], x['nb'], x['delta'], 'yes' if x['kind'] == 'landing' else 'no', o['mode'], o['mm'][0], o['len'][0], o['mm'][1], o['len'][1]))
        out.append('    bands: ' + fmt_bands(e, x['at']))
        smp = []
        for s in (0, 5, 10, 20, 40):
            r = sample(e, s, x['at'])
            smp.append('s%d %s' % (s, '-' if r is None else 'bed %.2f lanes-w %.2f ln %g fl %d' % (r['hi'] - r['lo'], r['w'], r['ln'], r['fl'])))
        out.append('    ribbon: ' + ' | '.join(smp))
        if e['rows'] and x['at'] == 0 and e['rows'][0]['v'] > 0.15: out.append('    ribbon starts at %.1f m (junction trim)' % e['rows'][0]['v'])
        if e['rows'] and x['at'] == 1 and e['rows'][-1]['v'] < e['len'] - 0.15: out.append('    ribbon ends %.1f m before the node (junction trim)' % (e['len'] - e['rows'][-1]['v']))
        for st in interior_steps(e):
            dist = st['v'] if x['at'] == 0 else e['len'] - st['v']
            if dist <= 60:
                out.append('    SQUARE STEP %.2f m (width %+.2f) at %.1f m from this end; width ramp here %.1f m, offset ramp %.1f m' % (
                    st['jump'], st['dw'], dist, ramp_len(e, x['at']), offset_len(e, x['at'])))
        for j in steps_j:
            if (j['a'] is x or j['b'] is x):
                other = j['b'] if j['a'] is x else j['a']
                if j.get('landed'): out.append('    SQUARE STEP at the landed node: S outer bed edge %.2f m from T+R' % j['jump']); continue
                out.append('    SQUARE STEP across the join: edges %.2f m apart (bed %.2f vs %.2f m) to way %d' % (j['jump'], j['wa'], j['wb'], other['way']))
    for ov in d['ovs']:
        if not inside(ov['E'], ov['N']): continue
        out.append('  SLIVER way %d / way %d: %.0f m (A stations %d-%d), overlap max %.2f m mean %.2f m at %.1f %.1f; cause: %s' % (
            ov['a'][0], ov['b'][0], ov['s1'] - ov['s0'], ov['s0'], ov['s1'], ov['maxw'], ov['meanw'], ov['E'], ov['N'], sliver_cause(d, ov)))
    out.append('')


def summary(d):
    E = d['entries']
    js = joint_steps(d)
    ist = [(e, s) for e in E.values() for s in interior_steps(e)]
    out = ['joint steps (edge jump > %.1f m across a join): %d' % (STEP_M, len(js))]
    c = collections.Counter(('landed node (S vs T+R)' if j.get('landed') else j['a']['kind'], j['a']['deg']) for j in js)
    for k, n in c.most_common(): out.append('  %-24s deg %d: %d' % (k[0], k[1], n))
    out.append('interior steps (edge moves > %.1f m within 1 m inside a stretch): %d rows in %d stretches' % (STEP_M, len(ist), len({(e['way'], e['sr']) for e, _ in ist})))
    def where(e, s):
        d0, d1 = s['v'], e['len'] - s['v']
        at = 0 if d0 <= d1 else 1
        dist = min(d0, d1)
        rl, ol = ramp_len(e, at), offset_len(e, at)
        if dist > 60: return 'mid-stretch'
        if rl > 0 and dist <= rl + 1: return 'inside a width ramp (%s)' % ('<5 m' if rl < 5 else '>=5 m')
        if ol > 0 and dist <= ol + 1: return 'inside an offset ramp'
        return 'near an end, no ramp'
    c = collections.Counter(where(e, s) for e, s in ist)
    for k, n in c.most_common(): out.append('  %-30s %d' % (k, n))
    out.append('slivers (overlap > 0.5 m wide over > 10 m): %d' % len(d['ovs']))
    c = collections.Counter(sliver_cause(d, ov).split(' (')[0].split(':')[-1].strip() if 'landed' in sliver_cause(d, ov) else
                            ('shared node ' + sliver_cause(d, ov).split('(')[1].split(',')[0] if sliver_cause(d, ov).startswith('shared') else 'no shared node')
                            for ov in d['ovs'])
    for k, n in c.most_common(): out.append('  %-40s %d' % (k, n))
    return out, js, ist


if __name__ == '__main__':
    d = parse(sys.argv[1])
    args = sys.argv[2:]
    out = []
    js = joint_steps(d)
    while args:
        if args[0] == '--view':
            view_report(d, args[1], float(args[2]), float(args[3]), float(args[4]), js, out); args = args[5:]
        elif args[0] == '--summary':
            out += summary(d)[0]; args = args[1:]
        else:
            sys.exit('unknown argument ' + args[0])
    print('\n'.join(out))
