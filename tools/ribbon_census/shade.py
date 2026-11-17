import re,sys,math,os
import numpy as np
from PIL import Image,ImageDraw,ImageFont
# options (before or after the positional arguments): --size N output pixels (default 1300); --ann RECORDS draws the
# stage-0c annotation overlay from `g2m_ribbon_census ... --spots` records (spots.py): OSM way lines and ids, stretch-end
# nodes with their served codes, width ramps (orange), offset ramps (magenta), landings (red), square steps, slivers.
argv=list(sys.argv[1:]); OUT_PX=1300; ANN=None
while '--size' in argv: i=argv.index('--size'); OUT_PX=int(argv[i+1]); del argv[i:i+2]
while '--ann' in argv: i=argv.index('--ann'); ANN=argv[i+1]; del argv[i:i+2]
src,out=argv[0],argv[1]; ce,cn,half=map(float,argv[2:5]); LW=float(argv[5])
marks=[(float(a),float(b),c) for a,b,c in (m.split(',') for m in argv[6:])]
W=max(2000,round(OUT_PX*2000/1300)); sc=W/(2*half); FS=W/2000
pat=re.compile(r"E ([\d.]+) N ([\d.]+) z \S+ terr \S+ v (\S+) fl (\S+) vg (\S+) u (\S+) w (\S+) ln (\S+) fw (\S+) fd (\S+)")
tris=[]
for line in open(src,encoding='utf-8',errors='replace'):
    if not line.startswith('   tri'): continue
    vs=pat.findall(line)
    if len(vs)!=3: continue
    e0,n0=float(vs[0][0]),float(vs[0][1])
    if abs(e0-ce)>half+5 or abs(n0-cn)>half+5: continue
    a=np.array(vs,dtype=float)  # E N v fl vg u w ln fw fd
    fl=int(a[0,3]); vg=a[0,4]; ln=a[0,7]
    if fl&16: k=5
    elif fl&32: k=4
    elif ln==0: k=1 if vg>.5 else 3
    else: k=0 if vg>.5 else 2
    tris.append((k,a))
tris.sort(key=lambda t:t[0])
img=np.zeros((W,W,3),np.float32); img[:]=(60,90,50)
BASE={0:(120,125,80),1:(150,130,150),3:(95,80,100)}
PAINT=np.array((240,240,235),np.float32)
def stripe(d,w): return (np.abs(d)<w*0.5).astype(np.float32)
for k,a in tris:
    x=(a[:,0]-ce+half)*sc; y=W-(a[:,1]-cn+half)*sc
    x0=max(int(math.floor(x.min())),0); x1=min(int(math.ceil(x.max())),W-1); y0=max(int(math.floor(y.min())),0); y1=min(int(math.ceil(y.max())),W-1)
    if x1<x0 or y1<y0: continue
    X,Y=np.meshgrid(np.arange(x0,x1+1)+.5,np.arange(y0,y1+1)+.5)
    den=(y[1]-y[2])*(x[0]-x[2])+(x[2]-x[1])*(y[0]-y[2])
    if abs(den)<1e-9: continue
    l0=((y[1]-y[2])*(X-x[2])+(x[2]-x[1])*(Y-y[2]))/den; l1=((y[2]-y[0])*(X-x[2])+(x[0]-x[2])*(Y-y[2]))/den; l2=1-l0-l1
    m=(l0>=-1e-3)&(l1>=-1e-3)&(l2>=-1e-3)
    if not m.any(): continue
    fl=int(a[0,3]); width=a[0,6]; lanes=a[0,7]; fw=a[0,8]
    u=l0*a[0,5]+l1*a[1,5]+l2*a[2,5]; v=l0*a[0,2]+l1*a[1,2]+l2*a[2,2]; fd=l0*a[0,9]+l1*a[1,9]+l2*a[2,9]
    paint=np.zeros_like(u,dtype=np.float32); hw=width*.5
    if k in (2,4,5) and fl&1:
        if fl&2:
            fast=bool(fl&8)
            for side,bb,hb in ((1,64,512),(-1,128,1024)):
                sm=(u>0) if side>0 else (u<0)
                broken=bool(fl&bb); ew=(0.30 if broken and fast else 0.15)*LW
                ep=stripe(np.abs(u)-(hw-0.25),ew)
                if broken:
                    dash=6.0 if fast else 1.5; ep=ep*(np.mod(v,dash*2)<dash)
                if fl&hb: ep=ep*0
                paint=np.where(sm,ep,paint)
        if lanes>1:
            lw=width/lanes; b=np.round((u+hw)/lw); ok=(b>0)&(b<lanes); dm=u+hw-b*lw
            sep=(fw>0)&(fw<lanes)&(np.abs(b-fw)<.25); solid=sep&bool(fl&4)
            dash=6.0 if fl&8 else 3.0; dashed=(np.mod(v,dash*3)<dash)
            paint=np.maximum(paint,np.where(ok,stripe(dm,0.15*LW)*np.where(solid,1,dashed),0))
    paint=paint*fd
    if k==0 or (fl&256): paint=paint*0
    if fl&16: paint=np.ones_like(paint)
    if k==2:
        g=62+10*min(lanes,5); base=np.array((g,g+3,g+12),np.float32)
    elif k in (4,5): base=np.array((80,84,96),np.float32)
    else: base=np.array(BASE[k],np.float32)
    if fl&32: m=m&(paint>=.5)
    col=base[None,None,:]*(1-paint[...,None])+PAINT*paint[...,None]
    sub=img[y0:y1+1,x0:x1+1]; sub[m]=col[m]
im=Image.fromarray(img.astype(np.uint8)); d=ImageDraw.Draw(im)
try: f=ImageFont.truetype('arial.ttf',round(24*FS))
except: f=None
g=20 if half<=120 else 50
x=math.ceil((ce-half)/g)*g
while x<ce+half:
    px=(x-ce+half)*sc; d.line([(px,0),(px,12*FS)],fill=(255,255,0),width=round(2*FS)); d.text((px+3,12*FS),str(int(x)),fill=(255,255,0),font=f); x+=g
yy=math.ceil((cn-half)/g)*g
while yy<cn+half:
    py=W-(yy-cn+half)*sc; d.line([(0,py),(12*FS,py)],fill=(255,255,0),width=round(2*FS)); d.text((14*FS,py-12*FS),str(int(yy)),fill=(255,255,0),font=f); yy+=g
for e,n,lab in marks:
    px,py=(e-ce+half)*sc,W-(n-cn+half)*sc
    d.ellipse([px-10,py-10,px+10,py+10],outline=(255,40,40),width=3); d.text((px+13,py-13),lab,fill=(255,70,70),font=f)
if ANN:
    sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
    import spots
    D=spots.parse(ANN)
    P=lambda e,n:((e-ce+half)*sc,W-(n-cn+half)*sc)
    inside=lambda e,n,m=0:abs(e-ce)<=half+m and abs(n-cn)<=half+m
    try: fs=ImageFont.truetype('arial.ttf',round(17*FS)); fb=ImageFont.truetype('arialbd.ttf',round(19*FS))
    except: fs=fb=None
    def label(x,y,text,col,font):
        bb=d.textbbox((x,y),text,font=font); d.rectangle([bb[0]-3,bb[1]-2,bb[2]+3,bb[3]+2],fill=(0,0,0)); d.text((x,y),text,fill=col,font=font)
    # OSM way lines, way id once per way at the vertex nearest the view centre
    for wid,w in D['ways'].items():
        pts=w['pts']
        if len(pts)>1: d.line([P(*p) for p in pts],fill=(0,230,255),width=max(1,round(1.5*FS)))
    # ramps along the stretch's reference (from the ribbon rows' reference points)
    def along(e,at,length,col,width,shift):
        rows=e['rows']
        if not rows or length<=0: return
        L=e['len']; sel=[r for r in rows if (r['v']<=length if at==0 else r['v']>=L-length)]
        if len(sel)<2: return
        pts=[]
        for r in sel:
            k=rows.index(r); a=rows[max(k-1,0)]['ref']; b=rows[min(k+1,len(rows)-1)]['ref']; hx,hy=b[0]-a[0],b[1]-a[1]; hl=math.hypot(hx,hy) or 1
            pts.append(P(r['ref'][0]-hy/hl*shift,r['ref'][1]+hx/hl*shift))
        d.line(pts,fill=col,width=round(width*FS))
    for e in D['entries'].values():
        for at,x in e['ends'].items():
            if not inside(x['E'],x['N'],half*0.5): continue
            along(e,at,spots.ramp_len(e,at),(255,150,0),7,0)
            if x['kind']=='landing': along(e,at,spots.offset_len(e,at),(255,40,40),5,1.2)
            elif spots.offset_len(e,at)>0: along(e,at,spots.offset_len(e,at),(230,0,230),4,1.2)
    # square steps and slivers
    for e in D['entries'].values():
        for st in spots.interior_steps(e):
            x,y=P(*st['pos']); r=7*FS; d.rectangle([x-r,y-r,x+r,y+r],outline=(255,0,0),width=round(3*FS))
    for j in spots.joint_steps(D):
        if inside(j['E'],j['N']):
            x,y=P(j['E'],j['N']); r=14*FS; d.rectangle([x-r,y-r,x+r,y+r],outline=(255,0,0),width=round(4*FS)); label(x+r,y+r,'STEP %.1f m'%j['jump'],(255,90,90),fs)
    for ov in D['ovs']:
        if inside(ov['E'],ov['N']):
            x,y=P(ov['E'],ov['N']); r=12*FS; d.ellipse([x-r,y-r,x+r,y+r],outline=(255,255,0),width=round(4*FS))
            label(x+r,y-r,'SLIVER %d/%d %.1f m x %d m'%(ov['a'][0],ov['b'][0],ov['maxw'],ov['s1']-ov['s0']),(255,255,0),fs)
    # stretch-end nodes: a dot each; labels for degree >= 3 and for degree-2 lane-count changes
    counted={k:e['counted'] for k,e in D['entries'].items()}
    by_node={}
    for x in D['ends']: by_node.setdefault(x['node'],[]).append(x)
    for nid,ends in by_node.items():
        x0,y0=ends[0]['E'],ends[0]['N']
        if not inside(x0,y0): continue
        x,y=P(x0,y0); r=6*FS; d.ellipse([x-r,y-r,x+r,y+r],fill=(255,255,255),outline=(0,0,0))
        deg=ends[0]['deg']; codes=D['nodes'].get(nid,{}).get('codes',[])
        lane_change=len({counted.get((z['way'],z['sr'])) for z in ends})>1
        if deg>=3 or (deg==2 and lane_change):
            label(x+9*FS,y-24*FS,'%d %s'%(nid,','.join(codes) or ('J:'+ends[0]['kind'])),(255,255,255),fb)
    # way ids last (on top)
    for wid,w in D['ways'].items():
        dense=[(a[0]+(b[0]-a[0])*k/10,a[1]+(b[1]-a[1])*k/10) for a,b in zip(w['pts'],w['pts'][1:]) for k in range(10)]+w['pts'][-1:]; cand=[p for p in dense if inside(*p,-5)]
        if not cand: continue
        p=min(cand,key=lambda q:(q[0]-ce)**2+(q[1]-cn)**2); x,y=P(*p)
        label(x+4*FS,y+4*FS,str(wid),(0,230,255),fs)
im.resize((OUT_PX,OUT_PX),Image.LANCZOS).save(out)
