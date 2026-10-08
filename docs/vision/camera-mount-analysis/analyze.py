"""Ideal full-tag frustum checks, not a detection or localization simulator.
Run: python3 analyze.py (Python standard library only).
Layout: WPILib 2026.2.1 welded JSON, checked against FIRST manual section 5.11.
Assumptions: 70-degree HFOV, 1280x800 full frame or 1280x720 horizontal-preserving crop, 8.125-inch
printed tag square, 6 m illustrative candidate cutoff. Approximate Hub-box occlusion only; outputs still require other structure/robot
line-of-sight checks. Six meters is a screening cutoff, not detector range.
Front/rear mounts are hypothetical +/-0.30 m longitudinal, 0.40 m high.
"""
import argparse,math,json,csv
from pathlib import Path
OUT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser()
parser.add_argument('--image-height', type=int, choices=(720,800), default=800)
args=parser.parse_args()
layout=json.loads((OUT/'2026-rebuilt-welded.json').read_text())
rad=math.radians
half=8.125*.0254/2
hfov=70
vfov=math.degrees(2*math.atan(math.tan(rad(35))*args.image_height/1280))
tags=[]
for t in layout['tags']:
 p=t['pose']['translation'];q=t['pose']['rotation']['quaternion']; yaw=2*math.atan2(q['Z'],q['W'])
 tags.append((t['ID'],p['x'],p['y'],p['z'],yaw))
# Approximate opaque Hub footprints from tag face planes. All these rays are
# below the Hub body top. Other structures and robot geometry are not modeled.
HUBS=[(4.0218614,5.2291742,3.4312352,4.63804),
      (11.3118646,12.5191774,3.4312352,4.63804)]
def blocked_by_hub(x,y,tx,ty):
 for xmin,xmax,ymin,ymax in HUBS:
  lower,upper=0.0,1.0
  for start,delta,lo,hi in ((x,tx-x,xmin+1e-5,xmax-1e-5),
                           (y,ty-y,ymin+1e-5,ymax-1e-5)):
   if abs(delta)<1e-12:
    if not lo<start<hi: lower,upper=1,0;break
   else:
    a,b=sorted(((lo-start)/delta,(hi-start)/delta))
    lower,upper=max(lower,a),min(upper,b)
  if lower<upper and upper>1e-6 and lower<1-1e-6:return True
 return False
# Full printed square, ideal pinhole, front face visible. No detection model.
def visible(cam,tag):
 x,y,z,yaw,pitch=cam;tid,tx,ty,tz,normal=tag
 dist=math.hypot(tx-x,ty-y)
 if dist>6 or blocked_by_hub(x,y,tx,ty): return False
 # Exclude back faces; no arbitrary incidence-angle threshold.
 if (x-tx)*math.cos(normal)+(y-ty)*math.sin(normal)<=0: return False
 for side in (-half,half):
  for up in (-half,half):
   dx=tx-side*math.sin(normal)-x;dy=ty+side*math.cos(normal)-y;dz=tz+up-z
   forward=dx*math.cos(yaw)+dy*math.sin(yaw)
   left=-dx*math.sin(yaw)+dy*math.cos(yaw)
   depth=forward*math.cos(pitch)+dz*math.sin(pitch)
   vertical=-forward*math.sin(pitch)+dz*math.cos(pitch)
   if depth<=0 or abs(left/depth)>math.tan(rad(35)) or abs(vertical/depth)>math.tan(rad(vfov/2)):return False
 return True
poses=[('A near Tower',1.8,4.0),('B near Hub',3.2,4.0),('C Outpost lane',3.2,.8),('D opposite lane',3.2,7.2),('E neutral Hub side',6.0,4.0)]
configs={'existing level side cameras':[(.0508,.3302,.4699,90,0),(.0508,-.3302,.4318,-90,0)]}
for tilt in (15,20,25):
 configs[f'front/rear {tilt} deg']=[(.30,0,.4,0,tilt),(-.30,0,.4,180,tilt)]
configs['front 40cm/15deg + rear 22in/10deg']=[(.30,0,.4,0,15),(-.30,0,.5588,180,10)]
rows=[]
for name,x,y in poses:
 for heading in (0,90,180,270):
  for config,cams in configs.items():
   seen=[]
   for ox,oy,z,yaw,pitch in cams:
    h=rad(heading)
    cam=(x+ox*math.cos(h)-oy*math.sin(h),y+ox*math.sin(h)+oy*math.cos(h),z,rad(heading+yaw),rad(pitch))
    seen.append([t[0] for t in tags if visible(cam,t)])
   rows.append([name,x,y,heading,config,','.join(map(str,seen[0])),','.join(map(str,seen[1]))])
with (OUT/f'sample-visibility-{args.image_height}.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['pose','x_m','y_m','heading_deg','configuration','camera_1_tag_ids','camera_2_tag_ids']);w.writerows(rows)
for r in rows:
 if r[3]==0:print(r)
print('Full printed-square vertical limits:')
for tilt in (15,20,25):
 lo=rad(tilt-vfov/2);hi=rad(tilt+vfov/2)
 for label,z in [('Tower/Outpost',.55245),('Trench',.889),('Hub',1.12395)]:
  near=(z+half-.4)/math.tan(hi)
  far=(z-half-.4)/math.tan(lo) if lo>0 else float('inf')
  print(tilt,label,round(near,3),round(far,3))
