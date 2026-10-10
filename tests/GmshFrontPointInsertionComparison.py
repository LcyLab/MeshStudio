"""同BREP上的实际native insertAPoint叶对照；冻结原运行池，不冻结新CAD点/BGM/空腔/提交。"""
import argparse
import ctypes
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference
from GmshFrontInsertionComparison import normalize_insertion


def normalize_point_insertion(document):
    normalize_insertion(document)
    document['model_length'] = float(document['model_length'])
    document['coordinates'] = list(map(float, document['coordinates']))
    for row in document['slots']: row[1:] = map(float, row[1:])
    if document['new_vertex']: document['new_vertex'][1:4] = map(float, document['new_vertex'][1:4])
    if document['new_slot']: document['new_slot'][1:5] = map(float, document['new_slot'][1:5])
    return document


def reference_point_insertion(gmsh, face, nodes, triangles, all_ids, active, internal, worst, query, maximum, star, failed):
    native = gmsh.lib.meshstudioReferenceFrontPointInsertionLeaf
    p, q = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
    native.argtypes = [ctypes.c_int,p,ctypes.c_int,p,ctypes.c_int,q,ctypes.c_int,q,ctypes.c_int,q,ctypes.c_int,ctypes.c_int,p,ctypes.c_size_t,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_char),ctypes.c_int]
    native.restype = ctypes.c_int
    def doubles(rows):
        flat = [x for row in rows for x in row]; return (ctypes.c_double*max(1,len(flat)))(*flat)
    def ints(values): return (ctypes.c_int*max(1,len(values)))(*values)
    output = ctypes.create_string_buffer(4*1024*1024)
    code = native(face,doubles(nodes),len(nodes),doubles(triangles),len(triangles),ints(all_ids),len(all_ids),ints(active),len(active),ints([x for row in internal for x in row]),len(internal),worst,(ctypes.c_double*5)(*query),maximum,int(star),int(failed),output,len(output))
    if code != 0: raise ValueError('native insertAPoint leaf failed:'+str(code))
    return normalize_point_insertion(json.loads(output.value.decode()))


def reference_initial_point_insertion(gmsh,face,node_state,face_mesh,triangle_state,queue_state,index,candidate,consumed):
    physical = {row[0]:row for row in face_mesh['vertices']}
    nodes = [[r[0],*r[1:3],*physical[r[0]][5:8],physical[r[0]][3],*r[3:5],1] for r in node_state['nodes']]
    triangles = [[*r[:3],r[6],int(r[10]),*r[7:10]] for r in triangle_state['triangles']]
    active = [i for i in queue_state['initial_active'] if i not in consumed]
    return reference_point_insertion(gmsh,face,nodes,triangles,queue_state['initial_all'],active,node_state['internal_edges'],index,
        [*candidate['values'][34:36],*candidate['values'][10:13]],face_mesh['maximum_id'],True,False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('api','library','own-executable','frozen-dir','out-dir'): parser.add_argument('--'+name,required=True)
    parser.add_argument('--runtime-dir',action='append',default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    # Child CAD replay must receive DLL paths too.
    os.environ['PATH'] = os.pathsep.join([*args.runtime_dir,os.environ.get('PATH','')])
    os.environ.pop('MESHSTUDIO_GMSH_TRACE',None); gmsh = load_reference_api(args.api,args.library)
    report = {'scope':__doc__,'complete':False,'cases':[], 'runtime_state_frozen':True,
              'new_node_geometry_and_background_frozen':False,'insert_apoint_lifecycle_validated':False,
              'front_generation_feedback_validated':False,'true_boundary_outer_gate_validated':False,
              'identity_normalization':'Only actual newly allocated model-global MVertex identity is mapped to frozen maximum+1; creation/consumption and relative order are compared.',
              'failed_gpoint_scope':'Explicit test proxy queries actual OCC Value then marks GPoint failed; not an observed natural OCC failure.',
              'library_sha256':hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              'own_executable_sha256':hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    totals = {}
    try:
        for fixture in ('rectangle','hole','cylinder','sphere'):
            gmsh.initialize([],readConfigFiles=False)
            try:
                if not gmsh.option.getString('General.Version').startswith('4.15.2'): raise ValueError('wrong fixed Gmsh version')
                gmsh.option.setNumber('General.Terminal',0); gmsh.option.setNumber('General.NumThreads',1)
                gmsh.option.setNumber('Mesh.Algorithm',6); gmsh.option.setNumber('Mesh.MeshSizeFromPoints',0)
                gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',0); gmsh.option.setNumber('Mesh.MeshSizeMin',0)
                gmsh.model.add(fixture); cad = Path(args.frozen_dir)/(fixture+'.brep')
                gmsh.model.occ.importShapes(str(cad),highestDimOnly=False); gmsh.model.occ.synchronize()
                gmsh.option.setNumber('Mesh.MeshSizeMax',.5); gmsh.model.mesh.generate(1)
                inputs,expected,descriptors = [],[],[]
                for _,face in gmsh.model.getEntities(2):
                    low,high = gmsh.model.getParametrizationBounds(2,face)
                    def uv(x,y): return [float(low[0]+(high[0]-low[0])*(.23+.34*x)),float(low[1]+(high[1]-low[1])*(.23+.34*y))]
                    coordinates = [uv(x,y) for x,y in ((0.,0.),(1.,0.),(0.,1.),(1.,1.))]
                    positions = [list(map(float,gmsh.model.getValue(2,face,p))) for p in coordinates]
                    length = max(math.dist(positions[0],p) for p in positions[1:])
                    if length <= 0.: raise ValueError('invalid frozen surface patch')
                    pool = [[1,2,3,2.,0,-1,-1,1],[2,4,3,2.,0,0,-1,-1]]
                    def collect(name,nodes,triangles,internal,point,extension,star,mode,failed=False,maximum=4):
                        max_size = length*.7; gmsh.option.setNumber('Mesh.MeshSizeMax',max_size)
                        gmsh.option.setNumber('Mesh.MeshSizeExtendFromBoundary',extension)
                        gmsh.model.mesh.setSizeCallback(lambda d,t,x,y,z,lc: max_size*(.25+.6/(1.+x*x+y*y+z*z)) if mode else max_size)
                        query = [*point,1.3,.2,.9]
                        reference = reference_point_insertion(gmsh,face,nodes,triangles,list(range(len(triangles))),[1] if len(triangles)>1 else [],internal,0,query,maximum,star,failed)
                        expected.append(reference); descriptors.append([face,name,extension,star,mode,failed])
                        header = [face,len(nodes),len(triangles),len(triangles),1 if len(triangles)>1 else 0,len(internal),0,maximum,extension,int(star),int(failed),mode,max_size,reference['model_length'],*query]
                        rows = [header,*nodes,*triangles,list(range(len(triangles))),[1] if len(triangles)>1 else [],*[list(edge) for edge in internal]]
                        inputs.append('\n'.join(' '.join(format(x,'.17g') for x in row) for row in rows)+'\n')
                    for extension,star,mode,dims,large,point in itertools.product((0,1),(False,True),(0,1),((1,1,1,1),(2,2,2,2),(0,1,2,2)),(False,True),((.25,.25),(.5,.5),(.01,.01),(.9,.9),(1.8,1.8))):
                        nodes = [[i+1,*coordinates[i],*positions[i],dims[i],length*(1. if large else .06*(i+1)),length*(1. if large else .1*(i+1)),1] for i in range(4)]
                        collect('matrix-'+str(len(inputs)),nodes,pool,[],uv(*point),extension,star,mode)
                    nodes = [[i+1,*coordinates[i],*positions[i],1,length*.06,length*.1,1] for i in range(4)]
                    for extension,mode in itertools.product((0,1),(0,1)):
                        collect('single-cavity',nodes,[pool[0][:5]+[-1,-1,-1]],[],uv(.25,.25),extension,True,mode)
                        collect('internal-diagonal',nodes,pool,[[2,3]],uv(.25,.25),extension,True,mode)
                        collect('injected-failed-gpoint',nodes,pool,[],uv(.25,.25),extension,True,mode,True)
                        ghost = [*nodes,[5,*uv(.01,.01),*positions[0],2,length,length,0],[6,*uv(.02,.02),*positions[0],2,length,length,0]]
                        collect('retained-ghost-slots',ghost,pool,[],uv(.25,.25),extension,True,mode,maximum=6)
                        predeleted = [pool[0],pool[1][:4]+[1]+pool[1][5:]]
                        collect('searched-predeleted-seed',nodes,predeleted,[],uv(1.,1.),extension,True,mode)
                frozen = directory/(fixture+'-input.tsv'); frozen.write_text(str(len(inputs))+'\n'+''.join(inputs),encoding='utf-8')
                own = directory/(fixture+'-own.json'); subprocess.run([args.own_executable,str(cad),str(frozen),str(own)],check=True)
                actual = json.loads(own.read_text(encoding='utf-8'))['cases']; summary = {'fixture':fixture,'queries':len(inputs),'codes':{},'inserted':0,'slots_appended':0,'injected_failures':0,'first_difference':None,
                    'cad_sha256':hashlib.sha256(cad.read_bytes()).hexdigest(),'input_sha256':hashlib.sha256(frozen.read_bytes()).hexdigest()}
                (directory/(fixture+'-reference.json')).write_text(json.dumps(expected,allow_nan=False),encoding='utf-8')
                if len(actual) != len(expected): raise ValueError('point insertion case count differs')
                for index,(reference,result) in enumerate(zip(expected,actual)):
                    result = normalize_point_insertion(result)
                    difference = first_trace_difference([{'stage':'insert_apoint',**reference}],[{'stage':'insert_apoint',**result}],0.)
                    code = str(reference['code']); summary['codes'][code] = summary['codes'].get(code,0)+1; totals[code] = totals.get(code,0)+1
                    summary['inserted'] += int(reference['inserted']); summary['slots_appended'] += int(reference['new_slot'] is not None); summary['injected_failures'] += int(descriptors[index][-1])
                    if difference: summary['first_difference'] = {'index':index,'case':descriptors[index],'difference':difference}; break
                report['cases'].append(summary)
                if summary['first_difference']: raise ValueError('native insertAPoint divergence:'+json.dumps(summary['first_difference']))
            finally: gmsh.finalize()
        if not all(totals.get(str(code),0) for code in (0,1,-1,-4,-9)): raise ValueError('missing lifecycle branch coverage:'+str(totals))
        report['codes'] = totals; report['insert_apoint_lifecycle_validated'] = True; report['complete'] = True
    except Exception as error: report['error'] = str(error); raise
    finally:
        (directory/'comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'complete':True,'queries':sum(c['queries'] for c in report['cases']),'codes':totals}))


if __name__ == '__main__': main()
