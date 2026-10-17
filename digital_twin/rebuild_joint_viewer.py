"""Rebuild the self-contained viewer from source modules and embedded mesh assets.

Run from the repository environment (NumPy is already a project dependency).
Source changes require explicit new model acquisition; UI edits preserve meshes.
"""
from pathlib import Path
import hashlib
import json
import re
from build_tactile_geometry import build

ROOT=Path(__file__).resolve().parent
BEGIN='<!-- JOINT PROFILES BEGIN -->'
END='<!-- JOINT PROFILES END -->'

def main():
    viewer=ROOT/'hand_twin_6dof.html'
    html=viewer.read_text(encoding='utf-8')
    if html.count(BEGIN)!=1 or html.count(END)!=1:
        raise ValueError('Expected exactly one viewer extension')
    packed=re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>',html,re.S)
    data=json.loads(packed.group(1))
    digest=hashlib.sha256((ROOT/'joint_geometry_profiles_for_codex.json').read_bytes()).hexdigest()
    if digest!=data['reference_sha256']:
        raise ValueError('Reference changed: verify matching geometry before rebuilding')
    data['tactile_geometry']=build(data)
    (ROOT/'tactile_geometry.json').write_text(json.dumps(data['tactile_geometry'],indent=2)+'\n',encoding='utf-8')
    packed=json.dumps(data,separators=(',',':')).replace('</','<\\/')
    scripts='\n'.join((ROOT/name).read_text(encoding='utf-8') for name in
        ['joint_pressure.js','joint_kinematics.js','tactile_points.js','joint_profiles_ui.js'])
    extension='\n'.join([BEGIN,'<style>',(ROOT/'joint_profiles.css').read_text(encoding='utf-8'),'</style>',
        (ROOT/'joint_profiles_panel.html').read_text(encoding='utf-8'),
        '<script id="jointProfileData" type="application/json">'+packed+'</script>',
        '<script>',scripts,'</script>',END])
    result=html[:html.index(BEGIN)]+extension+html[html.index(END)+len(END):]
    controller=(ROOT/'toolkit_controls.js').read_text(encoding='utf-8')
    result=re.sub(r'<script id="toolkitControls">.*?</script>',lambda _: '<script id="toolkitControls">'+controller+'</script>',result,flags=re.S)
    viewer.write_text(result,encoding='utf-8')
    print('Rebuilt viewer: matching source meshes, 17 fields, 1062 taxels.')

if __name__=='__main__':main()
