"""Manual PDF pp.29-30: regions and wire order; metric CAD fitting is inferred."""
REGIONS={}
for finger in ('little','ring','middle','index'):
    for zone,shape in [('tip',(3,3)),('nail',(12,8)),('pad',(10,8))]:
        REGIONS[f'{finger}_{zone}']=shape
for zone,shape in [('tip',(3,3)),('nail',(12,8)),('mid',(3,3)),('pad',(12,8))]:
    REGIONS[f'thumb_{zone}']=shape
REGIONS['palm']=(8,14)

def physical_cell(key,index):
    rows,cols=REGIONS[key]
    return (rows-1-index%rows,index//rows) if key=='palm' else divmod(index,cols)

def project_grid(triangles,origin,u,v,normal,seeds):
    """Intersect rays with CAD triangles; retain regular spacing instead of snapping."""
    import numpy as np
    local=triangles-origin
    xyz=np.stack([local@u,local@v,local@normal],axis=-1)
    a,b,c=xyz[:,0],xyz[:,1],xyz[:,2]
    den=(b[:,1]-c[:,1])*(a[:,0]-c[:,0])+(c[:,0]-b[:,0])*(a[:,1]-c[:,1])
    safe=np.where(abs(den)>1e-10,den,np.nan)
    points=[];misses=0
    # Fit each row inside the CAD silhouette, preserving evenly spaced columns.
    rows={}
    for x,y in seeds:rows.setdefault(y,[]).append(x)
    edges=xyz[:,[0,1,2,0]]
    row_ranges={}
    for y,xs in rows.items():
        start,end=edges[:,:3],edges[:,1:]
        delta=end[:,:,1]-start[:,:,1]
        t=(y-start[:,:,1])/np.where(abs(delta)>1e-10,delta,np.nan)
        valid=(t>=0)&(t<=1)
        intersections=(start[:,:,0]+t*(end[:,:,0]-start[:,:,0]))[valid]
        if len(intersections):
            low=max(min(xs),float(intersections.min())+.2)
            high=min(max(xs),float(intersections.max())-.2)
            if low<high:row_ranges[y]=(low,high,min(xs),max(xs))
    for x,y in seeds:
        if y in row_ranges:
            low,high,old_low,old_high=row_ranges[y]
            if old_high>old_low:x=low+(high-low)*(x-old_low)/(old_high-old_low)
        w0=((b[:,1]-c[:,1])*(x-c[:,0])+(c[:,0]-b[:,0])*(y-c[:,1]))/safe
        w1=((c[:,1]-a[:,1])*(x-c[:,0])+(a[:,0]-c[:,0])*(y-c[:,1]))/safe
        hit=(w0>=-1e-8)&(w1>=-1e-8)&(w0+w1<=1+1e-8)
        depth=w0*a[:,2]+w1*b[:,2]+(1-w0-w1)*c[:,2]
        if hit.any():d=depth[hit].max()
        else:
            misses+=1;center=xyz.mean(axis=1)
            d=center[((center[:,:2]-[x,y])**2).sum(axis=1).argmin(),2]
        points.append((origin+x*u+y*v+(d+.55)*normal).round(4).tolist())
    return points,misses

def layout(vertices,faces,groups,fingers):
    import numpy as np
    masks=dict(groups);regions=[]
    sizes={'little':{'tip':(6.5,4.7),'nail':(12.4,16.5),'pad':(12.3,15.4)},
           'ring':{'tip':(6.3,4.3),'nail':(11.5,15.8),'pad':(11.6,14.8)},
           'middle':{'tip':(6.,3.9),'nail':(10.8,15.4),'pad':(10.9,14.3)},
           'index':{'tip':(6.2,4.1),'nail':(11.2,15.6),'pad':(11.4,14.7)},
           'thumb':{'tip':(6.,3.9),'nail':(10.5,15.2),'mid':(6.3,6.2),'pad':(11.7,15.8)}}
    palm_center=np.array([-4.,8.,96.])
    for key,(rows,cols) in REGIONS.items():
        finger,_,zone=key.partition('_')
        fi=next((i for i,f in enumerate(fingers) if f['key']==finger),-1)
        u=np.array([1.,0,0]);v=np.array([0.,0,-1]);normal=np.array([0.,1,0]);origin=np.zeros(3)
        if fi>=0:
            f=fingers[fi];distal=zone!='pad';part=finger+('_dist' if distal else '_prox')
            width,height=sizes[finger][zone]
            zcenter=(f['base'][2]+f['pip'][2])/2 if zone=='pad' else f['tip'][2]-9-height/2 if zone=='nail' else f['tip'][2]-3.5
            origin=np.array([f['base'][0],0,zcenter])
        elif finger=='thumb':
            distal=zone in ('mid','nail','tip');part='thumb_dist' if distal else 'thumb_prox'
            base=np.array([39.74,52.76,82.63]);axis=np.array([13.,25.,39.]);axis/=np.linalg.norm(axis)
            # The gripping side faces the palm; do NOT select it by the sign of Y.
            toward=palm_center-base;normal=toward-axis*np.dot(toward,axis);normal/=np.linalg.norm(normal)
            v=-axis;u=np.cross(v,normal);u/=np.linalg.norm(u)
            cloud=(vertices[faces[masks[part]]].reshape(-1,3)-base)@axis
            low,high=np.quantile(cloud,[.05,.95])
            width,height=sizes['thumb'][zone]
            along={'pad':low+(high-low)*.38,'mid':low+height/2+1,
                   'nail':high-8-height/2,'tip':high-height/2-1}[zone]
            origin=base+axis*along
        else:distal=False;part='static';origin=palm_center.copy();origin[1]=0;width,height=40.,25.
        seeds=[];cells=[]
        for index in range(rows*cols):
            r,c=physical_cell(key,index);cells.append([r,c])
            seeds.append([width*((c+.5)/cols-.5),height*((r+.5)/rows-.5)])
        points,misses=project_grid(vertices[faces[masks[part]]],origin,u,v,normal,seeds)
        # Keep the editable anchor on the selected outer surface.
        anchor=np.mean(points,axis=0)-normal*.55
        regions.append(dict(key=key,rows=rows,cols=cols,finger=fi,thumb=finger=='thumb',distal=distal,
            points=points,physical_cells=cells,projection_misses=misses,
            parent=part,width=width,height=height,center=anchor.round(5).tolist(),
            basis=[u.round(8).tolist(),v.round(8).tolist(),normal.round(8).tolist()],
            grip_target=palm_center.tolist() if finger=='thumb' else None,
            source='Manual PDF p.29 regions; p.30 wire order',
            size_source='user supplied sensor component XY bounds / palm 40x25 mm; active area approximate',
            placement='regular CAD surface grid; thumb normal toward palm; mounting inferred, uncalibrated'))
    return regions
