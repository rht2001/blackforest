"""Derive 1,062 approximate local taxel centres from the embedded Toolkit meshes.

No external files/downloads or hardware access. Component mesh XY bounds with a
12% inset approximate active areas. Ray intersections place centres on surfaces.
The generated frame is always the sensor link, never another model's frame.
"""
from pathlib import Path
import hashlib
import json
import math
import re
import numpy as np

ROOT=Path(__file__).resolve().parent
PROFILE='rh56_tactile_toolkit_left'

def origin(xyz,rpy):
    r,p,y=rpy
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    m=np.eye(4)
    m[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    m[:3,3]=xyz
    return m

def regions():
    for f in ['little','ring','middle','index']:
        for zone,n,rows,cols in [('tip',3,3,3),('nail',2,12,8),('pad',1,10,8)]:
            yield f'{f}_{zone}',f'left_{f}_force_sensor_{n}',rows,cols
    for zone,n,rows,cols in [('tip',4,3,3),('nail',3,12,8),('mid',2,3,3),('pad',1,12,8)]:
        yield f'thumb_{zone}',f'left_thumb_force_sensor_{n}',rows,cols
    yield 'palm','left_palm_force_sensor',8,14

def build(data):
    p=data['reference']['primary_profiles'][PROFILE]
    frames={p['root_link']:np.eye(4)}
    pending=p['joints'].copy()
    while pending:
        j=next(j for j in pending if j['parent'] in frames)
        frames[j['child']]=frames[j['parent']]@origin(j['origin_xyz_m'],j['origin_rpy_rad'])
        pending.remove(j)
    base_inv=np.linalg.inv(frames['left_base_link'])
    yaw,pitch=math.radians(30),math.radians(20)
    reference_u=np.array([math.cos(yaw),-math.sin(yaw),0])
    reference_n=np.array([-math.sin(yaw)*math.cos(pitch),-math.cos(yaw)*math.cos(pitch),math.sin(pitch)])
    reference_v=np.cross(reference_n,reference_u)
    assets={a['link']:a for a in data['assets'][PROFILE]}
    result=[]
    for key,link,rows,cols in regions():
        visual=assets[link]
        vertices=np.asarray(visual['vertices']).reshape(-1,3)*visual['scale']
        triangles=vertices[np.asarray(visual['indices']).reshape(-1,3)]
        visual_tf=origin(visual['origin_xyz_m'],visual['origin_rpy_rad'])
        zero_tf=base_inv@frames[link]@visual_tf
        mirror_x=bool(reference_u@zero_tf[:3,0]<0)
        mirror_y=bool(reference_v@zero_tf[:3,1]>0)
        low,high=vertices.min(axis=0),vertices.max(axis=0)
        lower=low[:2]+.12*(high-low)[:2]
        upper=high[:2]-.12*(high-low)[:2]
        a,b,c=triangles[:,0],triangles[:,1],triangles[:,2]
        den=(b[:,1]-c[:,1])*(a[:,0]-c[:,0])+(c[:,0]-b[:,0])*(a[:,1]-c[:,1])
        safe=np.where(abs(den)>1e-15,den,np.nan)
        cells=[]
        for index in range(rows*cols):
            row,col=(rows-1-index%rows,index//rows)if key=='palm'else divmod(index,cols)
            uv=np.array([(col+.5)/cols,(row+.5)/rows])
            if mirror_x:uv[0]=1-uv[0]
            if mirror_y:uv[1]=1-uv[1]
            x,y=lower+uv*(upper-lower)
            w0=((b[:,1]-c[:,1])*(x-c[:,0])+(c[:,0]-b[:,0])*(y-c[:,1]))/safe
            w1=((c[:,1]-a[:,1])*(x-c[:,0])+(a[:,0]-c[:,0])*(y-c[:,1]))/safe
            depth=w0*a[:,2]+w1*b[:,2]+(1-w0-w1)*c[:,2]
            valid=(w0>=-1e-7)&(w1>=-1e-7)&(w0+w1<=1+1e-7)&np.isfinite(depth)
            if not valid.any():raise ValueError(f'{key}:{index} misses sensor surface')
            candidates=np.where(valid)[0]
            hit=candidates[np.argmax(depth[candidates])]
            normal=np.cross(b[hit]-a[hit],c[hit]-a[hit])
            normal/=np.linalg.norm(normal)
            if normal[2]<0:normal=-normal
            point=(visual_tf@np.array([x,y,depth[hit],1]))[:3]
            normal=visual_tf[:3,:3]@normal
            cells.append({'wire_index':index,'row':row,'col':col,
                'point_m':point.round(10).tolist(),'normal':normal.round(10).tolist()})
        result.append({'key':key,'link':link,'rows':rows,'cols':cols,'mirror_x':mirror_x,'mirror_y':mirror_y,
            'mesh':visual['mesh'],'mesh_geometry_sha256':hashlib.sha256(json.dumps(visual,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
            'cells':cells})
    geometry={'schema_version':1,'profile':PROFILE,'reference_sha256':data['reference_sha256'],
        'units':'metres','point_frame':'region sensor link','total_cells':sum(len(r['cells'])for r in result),
        'geometry_status':'APPROXIMATE_UNCALIBRATED','placement':'Sensor mesh XY bounds inset 12%, positive local-Z surface ray intersection',
        'orientation_status':'Reference-view row/column convention; verify on physical hand with single-cell activation',
        'regions':result}
    assert geometry['total_cells']==1062
    geometry['geometry_sha256']=hashlib.sha256(json.dumps(geometry,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return geometry

def main():
    html=(ROOT/'hand_twin_6dof.html').read_text(encoding='utf-8')
    data=json.loads(re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>',html,re.S).group(1))
    geometry=build(data)
    (ROOT/'tactile_geometry.json').write_text(json.dumps(geometry,indent=2)+'\n',encoding='utf-8')
    print(f"Built {geometry['total_cells']} cells on {len(geometry['regions'])} sensor meshes; zero projection misses.")

if __name__=='__main__':main()
