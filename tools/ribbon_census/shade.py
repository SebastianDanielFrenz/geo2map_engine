import re,sys,math
import numpy as np
from PIL import Image,ImageDraw,ImageFont
src,out=sys.argv[1],sys.argv[2]; ce,cn,half=map(float,sys.argv[3:6]); LW=float(sys.argv[6])
marks=[(float(a),float(b),c) for a,b,c in (m.split(',') for m in sys.argv[7:])]
W=2000; sc=W/(2*half)
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
try: f=ImageFont.truetype('arial.ttf',24)
except: f=None
g=20 if half<=120 else 50
x=math.ceil((ce-half)/g)*g
while x<ce+half:
    px=(x-ce+half)*sc; d.line([(px,0),(px,12)],fill=(255,255,0),width=2); d.text((px+3,12),str(int(x)),fill=(255,255,0),font=f); x+=g
yy=math.ceil((cn-half)/g)*g
while yy<cn+half:
    py=W-(yy-cn+half)*sc; d.line([(0,py),(12,py)],fill=(255,255,0),width=2); d.text((14,py-12),str(int(yy)),fill=(255,255,0),font=f); yy+=g
for e,n,lab in marks:
    px,py=(e-ce+half)*sc,W-(n-cn+half)*sc
    d.ellipse([px-10,py-10,px+10,py+10],outline=(255,40,40),width=3); d.text((px+13,py-13),lab,fill=(255,70,70),font=f)
im.resize((1300,1300),Image.LANCZOS).save(out)
