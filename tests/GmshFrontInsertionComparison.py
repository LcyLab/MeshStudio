"""冻结已分配节点/前沿池/空腔，调用实际native insertVertexB验收/局部连接与活动更新。"""
import argparse
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def normalize_insertion(document):
    document['validation'][:3] = map(float,document['validation'][:3])
    for row in document['created']: row[:] = [float(x) if not isinstance(x,str) else x for x in row]
    for row in document['pool']: row[3] = float(row[3])
    return document


def reference_front_insertion(gmsh,face,nodes,triangles,all_ids,active,cavity,shell,new_node,force,star):
    native = gmsh.lib.meshstudioReferenceFrontInsertionLeaf; p = ctypes.POINTER(ctypes.c_double); q = ctypes.POINTER(ctypes.c_int)
    native.argtypes = [ctypes.c_int,p,ctypes.c_int,p,ctypes.c_int,q,ctypes.c_int,q,ctypes.c_int,q,ctypes.c_int,q,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_char),ctypes.c_int]; native.restype = ctypes.c_int
    def doubles(rows):
        flat = [x for row in rows for x in row]; return (ctypes.c_double*max(1,len(flat)))(*flat)
    def ints(values): return (ctypes.c_int*max(1,len(values)))(*values)
    out = ctypes.create_string_buffer(16*1024*1024)
    code = native(face,doubles(nodes),len(nodes),doubles(triangles),len(triangles),ints(all_ids),len(all_ids),ints(active),len(active),ints(cavity),len(cavity),ints([x for row in shell for x in row]),len(shell),new_node,int(force),int(star),out,len(out))
    if code != 0: raise ValueError('native insertion leaf failed:'+str(code))
    return normalize_insertion(json.loads(out.value.decode()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('api','library','own-executable','frozen-dir','out-dir'): parser.add_argument('--'+name,required=True)
    parser.add_argument('--runtime-dir',action='append',default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]; gmsh = load_reference_api(args.api,args.library)
    report = {'scope':__doc__,'complete':False,'cases':[],'runtime_state_frozen':True,'new_node_geometry_and_sizes_frozen':True,'insert_apoint_lifecycle_validated':False,'front_generation_feedback_validated':False,
              'library_sha256':hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),'own_executable_sha256':hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    cases = []; pool = [[1,2,3,2.,1,-1,-1,1],[2,4,3,2.,1,0,-1,-1]]; shell = [[0,0],[0,1],[1,1],[1,2]]
    for ext,star,force,dims,large,point in itertools.product((0,1),(False,True),(False,True),((1,1,1,1),(2,2,2,2),(0,1,2,2)),(False,True),((.5,.5),(1.,1.),(3.,1.),(0.,0.))):
        nodes = [[i+1,*uv,*uv,0.,dims[i],1. if large else .125,1. if large else .25] for i,uv in enumerate(((0.,0.),(2.,0.),(0.,2.),(2.,2.)))]
        nodes.append([5,*point,*point,0.,2,1. if large else .5,1.])
        cases.append(('case-'+str(len(cases)),nodes,pool,[0,1],[1],[0,1],shell,4,ext,force,star))
    square = [[i+1,*uv,*uv,0.,1,.125,.25] for i,uv in enumerate(((0.,0.),(2.,0.),(0.,2.),(2.,2.)))]+[[5,1.,1.,1.,1.,0.,2,.5,1.]]
    cases.append(('single-cavity',square,pool,[0,1],[1],[0],shell,4,1,False,True))
    cases.append(('bad-euler',square,pool,[0,1],[1],[0,1],shell[:-1],4,1,False,True))
    equality = [list(r) for r in square]
    for row in equality[:4]: row[1] *= 500000.; row[2] *= 500000.
    equality[-1][1:3] = [-1.e-6,500000.]
    cases.append(('strict-equality-minus5',equality,pool,[0,1],[1],[0,1],shell,4,1,False,True))
    negative = [list(r) for r in square]
    for row in negative: row[-2:] = [-.25,-.5]
    for ext in (0,1): cases.append(('negative-dual-size-'+str(ext),negative,pool,[0,1],[1],[0,1],shell,4,ext,False,True))
    external = [list(r) for r in square]; external[-1][0] = 6; external.insert(4,[5,-1.,1.,-1.,1.,0.,2,.125,.25])
    external_pool = [list(r) for r in pool]+[[3,5,1,.1,0,0,-1,-1]]; external_pool[0][5] = 2
    cases.append(('local-other-side',external,external_pool,[0,1,2],[1],[0,1],shell,5,1,False,True))
    repeated_shell = [[0,0],[0,0],[1,1],[1,2]]
    cases.append(('duplicate-local-other-side',external,external_pool,[0,1,2],[1],[0,1],repeated_shell,5,1,True,False))
    close = [list(r) for r in square]
    close[-1][1:6] = [.01,.01,.01,.01,0.]
    for row in close: row[-2:] = [1.,1.]
    cases.append(('close-minus4',close,pool,[0,1],[1],[0,1],shell,4,1,False,True))
    obtuse = [list(r) for r in square]; obtuse[-1][3:6] = [1.,1.e-5,0.]
    for row in obtuse[:4]: row[6] = 2
    cases.append(('obtuse-face-endpoints-minus4',obtuse,pool,[0,1],[1],[0,1],shell,4,1,False,True))
    for dim in (1,2):
        height = [list(r) for r in square]; height[-1][3:6] = [.01,.3,0.]
        for row in height[:4]: row[6] = dim
        cases.append(('boundary-height-dim'+str(dim),height,pool,[0,1],[1],[0,1],shell,4,1,False,True))
    curved = [list(r) for r in square]
    for row,z in zip(curved,(0.,.2,-.1,.4,.3)): row[5] = z
    cases.append(('nonplanar-physical-radius',curved,pool,[0,1],[1],[0,1],shell,4,1,False,True))
    gmsh.initialize([],readConfigFiles=False)
    try:
        gmsh.option.setNumber('General.Terminal',0.); gmsh.model.add('insertion-leaf'); gmsh.model.occ.importShapes(str(Path(args.frozen_dir)/'rectangle.brep'),highestDimOnly=False); gmsh.model.occ.synchronize(); face = gmsh.model.getEntities(2)[0][1]
        for name,nodes,triangles,all_ids,active,cavity,shell,new_node,ext,force,star in cases:
            gmsh.option.setNumber('Mesh.MeshSizeExtendFromBoundary',ext)
            expected = reference_front_insertion(gmsh,face,nodes,triangles,all_ids,active,cavity,shell,new_node,force,star)
            frozen = directory/(name+'-input.tsv'); rows = [[len(nodes),len(triangles),len(all_ids),len(active),len(cavity),len(shell),new_node,ext,int(force),int(star)],*nodes,*triangles,*[[i] for i in all_ids],*[[i] for i in active],*[[i] for i in cavity],*shell]
            frozen.write_text(''.join(' '.join(format(x,'.17g') for x in row)+'\n' for row in rows),encoding='utf-8')
            (directory/(name+'-reference.json')).write_text(json.dumps(expected,allow_nan=False),encoding='utf-8'); own = directory/(name+'-own.json')
            subprocess.run([args.own_executable,str(frozen),str(own)],check=True); actual = normalize_insertion(json.loads(own.read_text(encoding='utf-8')))
            difference = first_trace_difference([{'stage':'front_insertion',**expected}],[{'stage':'front_insertion',**actual}],0.)
            report['cases'].append({'name':name,'code':expected['code'],'created':len(expected['created']),'retained_new_triangles':len(expected['pool'])-len(triangles),'neighborhood':len(expected['neighborhood']),'first_difference':difference})
            if difference: raise ValueError('native insertion divergence:'+name+':'+json.dumps(difference))
        if len(report['cases']) != 204 or set(c['code'] for c in report['cases']) != {1,-1,-2,-3,-4,-5}: raise ValueError('fixed insertion matrix count/codes differ')
        report['complete'] = True
    finally:
        gmsh.finalize(); (directory/'front-insertion-report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        for handle in handles: handle.close()
    print(json.dumps({'cases':len(report['cases']),'native_insert_vertex_b_equivalent':report['complete']}))


if __name__ == '__main__': main()
