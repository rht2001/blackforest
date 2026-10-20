"""Cached triangle BVH for exact, batched first ray hits in millimetres."""
import numpy as np


class TriangleBVH:
    def __init__(self, mesh, leaf_size=64):
        triangles=np.asarray(mesh.triangles)
        self.vertices=triangles[:,0].copy()
        self.edge1=(triangles[:,1]-triangles[:,0]).copy()
        self.edge2=(triangles[:,2]-triangles[:,0]).copy()
        lower=triangles.min(axis=1);upper=triangles.max(axis=1)
        centers=(lower+upper)*.5
        bounds=[];children=[];leaves=[]
        def build(ids):
            node=len(bounds);bounds.append((lower[ids].min(axis=0),upper[ids].max(axis=0)))
            children.append((-1,-1));leaves.append(None)
            if len(ids)<=leaf_size:
                leaves[node]=ids
            else:
                axis=np.argmax(np.ptp(centers[ids],axis=0));middle=len(ids)//2
                order=np.argpartition(centers[ids,axis],middle)
                children[node]=(build(ids[order[:middle]]),build(ids[order[middle:]]))
            return node
        build(np.arange(len(triangles)))
        self.lower=np.array([b[0] for b in bounds])-1e-9
        self.upper=np.array([b[1] for b in bounds])+1e-9
        self.children=children;self.leaves=leaves

    def first(self, origins, directions, *, all_hits=False, max_distance=None):
        if not all_hits:
            ri,fi,distance=self.first(origins,directions,all_hits=True,max_distance=max_distance)
            count=len(origins);best=np.full(count,np.inf);faces=np.full(count,-1,dtype=int)
            if len(ri):
                order=np.lexsort((fi,distance,ri));ri,fi,distance=ri[order],fi[order],distance[order]
                ids,first=np.unique(ri,return_index=True);best[ids]=distance[first];faces[ids]=fi[first]
            return best,faces
        origins=np.asarray(origins,dtype=float).reshape(-1,3)
        directions=np.asarray(directions,dtype=float).reshape(-1,3)
        n=len(origins);best=np.full(n,np.inf);faces=np.full(n,-1,dtype=int)
        limit=np.full(n,np.inf) if max_distance is None else np.broadcast_to(np.asarray(max_distance,dtype=float),(n,))
        if not n:return (np.empty(0,dtype=int),np.empty(0,dtype=int),np.empty(0)) if all_hits else (best,faces)
        parallel=abs(directions)<1e-15
        inverse=np.divide(1.,directions,out=np.zeros_like(directions),where=~parallel)
        ray_batches=[];face_batches=[]
        stack=[(0,np.arange(n))]
        while stack:
            node,rays=stack.pop()
            lo=self.lower[node];hi=self.upper[node]
            a=(lo-origins[rays])*inverse[rays];b=(hi-origins[rays])*inverse[rays]
            entering=np.where(parallel[rays],-np.inf,np.minimum(a,b)).max(axis=1)
            leaving=np.where(parallel[rays],np.inf,np.maximum(a,b)).min(axis=1)
            outside=np.any(parallel[rays]&((origins[rays]<lo)|(origins[rays]>hi)),axis=1)
            rays=rays[(leaving>=np.maximum(entering,0.))&~outside&(entering<=limit[rays])]
            if not len(rays):continue
            leaf=self.leaves[node]
            if leaf is None:
                left,right=self.children[node];stack.append((right,rays));stack.append((left,rays))
            else:
                ray_batches.append(np.repeat(rays,len(leaf)));face_batches.append(np.tile(leaf,len(rays)))
        if not ray_batches:return (np.empty(0,dtype=int),np.empty(0,dtype=int),np.empty(0)) if all_hits else (best,faces)
        ray_ids=np.concatenate(ray_batches);face_ids=np.concatenate(face_batches)
        hits=[]
        for start in range(0,len(ray_ids),32768):
            ri=ray_ids[start:start+32768];fi=face_ids[start:start+32768]
            d=directions[ri];e1=self.edge1[fi];e2=self.edge2[fi]
            p=np.cross(d,e2);det=np.einsum('ij,ij->i',e1,p)
            valid=abs(det)>1e-12
            inv=np.divide(1.,det,out=np.zeros_like(det),where=valid)
            delta=origins[ri]-self.vertices[fi]
            u=np.einsum('ij,ij->i',delta,p)*inv
            q=np.cross(delta,e1);v=np.einsum('ij,ij->i',d,q)*inv
            distance=np.einsum('ij,ij->i',e2,q)*inv
            valid&=(u>=-1e-10)&(v>=-1e-10)&(u+v<=1.+1e-10)&(distance>=0.)&(distance<=limit[ri]+1e-9)
            ri,fi,distance=ri[valid],fi[valid],distance[valid]
            if not len(ri):continue
            if all_hits:
                hits.append((ri,fi,distance));continue
            # Sort by ray then distance; duplicate faces at CAD seams do not
            # multiply evidence. Lowest face index resolves exact ties.
            order=np.lexsort((fi,distance,ri));ri,fi,distance=ri[order],fi[order],distance[order]
            unique,first=np.unique(ri,return_index=True)
            improve=distance[first]<best[unique]
            chosen=first[improve];chosen_rays=ri[chosen]
            best[chosen_rays]=distance[chosen];faces[chosen_rays]=fi[chosen]
        if all_hits:
            return tuple(np.concatenate([part[i] for part in hits]) for i in range(3)) if hits else (np.empty(0,dtype=int),np.empty(0,dtype=int),np.empty(0))
        return best,faces

    def contains(self, points):
        """Odd/even parity, for watertight meshes only; winding independent."""
        points=np.asarray(points,dtype=float).reshape(-1,3)
        inside=np.zeros(len(points),dtype=bool)
        selected=np.flatnonzero(np.all((points>=self.lower[0])&(points<=self.upper[0]),axis=1))
        if not len(selected):return inside
        direction=np.sqrt([3.,5.,7.]);direction/=np.linalg.norm(direction)
        rays,_,distance=self.first(points[selected],np.tile(direction,(len(selected),1)),all_hits=True)
        if not len(rays):return inside
        order=np.lexsort((distance,rays));rays,distance=rays[order],distance[order]
        unique=np.r_[True,(rays[1:]!=rays[:-1])|(abs(distance[1:]-distance[:-1])>1e-7)]
        counts=np.bincount(rays[unique],minlength=len(selected))
        inside[selected]=(counts%2)==1
        return inside


def cached_bvh(model):
    # CandidateModel owns immutable geometry throughout an identification job.
    # Keep this object with that model, not in an unbounded process-global map.
    bvh=getattr(model,'_first_hit_bvh',None)
    if bvh is None:
        bvh=TriangleBVH(model.mesh);model._first_hit_bvh=bvh
    return bvh
