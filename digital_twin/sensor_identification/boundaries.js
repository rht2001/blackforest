/* Palmar mesh samples stay in their source link frame until FK is applied. */
(function(root){
 'use strict';
 function transform(data,solution,K,T){
  if(!solution?.frames?.left_base_link)return [];
  const base=T.inverseRigid(solution.frames.left_base_link);
  return data.points.filter(p=>solution.frames[p.link]).map(p=>{
   const m=K.mul(base,solution.frames[p.link]),[x,y,z]=K.point(m,p.point_m);
   return {id:p.id,link:p.link,x,y,z,normal:T.direction(m,p.normal),kind:'hand_boundary',pressure:null};
  });
 }
 root.HandBoundaries={transform};
 if(typeof module!=='undefined')module.exports=root.HandBoundaries;
})(typeof globalThis==='undefined'?this:globalThis);
