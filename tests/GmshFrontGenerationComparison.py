"""同BREP固定初始池，直接观察actual bowyerWatsonFrontal动态循环；不冻结新点或反馈决策。"""
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


def reference_generation(gmsh, face, nodes, triangles, all_ids, internal, gate, boundary, maximum, maximum_pops=20000):
    native = gmsh.lib.meshstudioReferenceFrontGeneration
    p, q = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
    native.argtypes = [ctypes.c_int,p,ctypes.c_int,p,ctypes.c_int,q,ctypes.c_int,q,ctypes.c_int,ctypes.c_int,p,ctypes.c_int,ctypes.c_size_t,ctypes.c_size_t,ctypes.POINTER(ctypes.c_char),ctypes.c_int]
    native.restype = ctypes.c_int
    def doubles(rows):
        flat = [x for row in rows for x in row]; return (ctypes.c_double*max(1,len(flat)))(*flat)
    def ints(values): return (ctypes.c_int*max(1,len(values)))(*values)
    output = ctypes.create_string_buffer(64*1024*1024)
    code = native(face,doubles(nodes),len(nodes),doubles(triangles),len(triangles),ints(all_ids),len(all_ids),ints([x for row in internal for x in row]),len(internal),gate,doubles(boundary),len(boundary),maximum,maximum_pops,output,len(output))
    if code: raise ValueError('native generation failed:'+str(code))
    return json.loads(output.value.decode())


def normalize_generation(document):
    for event in document['events']:
        insertion = event['insertion']
        if insertion:
            for key in ('model_length','slots','live_indices','maximum_vertex_id','interior_appended','pool','all','active'):
                insertion.pop(key,None)
    final = document['final']
    for key in ('interiors','vertices'):
        if key in document: final[key] = document.pop(key)
    document['final'] = {key:final[key] for key in ('pool','slots','live_indices','maximum_vertex_id','interiors','vertices','all','active')}
    validate_generation_sets(document)
    return document


def validate_generation_sets(document):
    all_ids,active = set(document['initial_all']),set(document['initial_active'])
    if len(all_ids) != len(document['initial_all']) or len(active) != len(document['initial_active']): raise ValueError('duplicate initial queue identity')
    for event in document['events']:
        if event['selection'][0] not in active: raise ValueError('popped triangle absent from preceding activity set')
        for values,key in ((all_ids,'all_delta'),(active,'active_delta')):
            removed,added = event[key]
            if len(set(removed)) != len(removed) or len(set(added)) != len(added) or not set(removed).issubset(values): raise ValueError('invalid generation set removal')
            values.difference_update(removed)
            if values.intersection(added): raise ValueError('generation set addition already present')
            values.update(added)
    if all_ids != set(document['final']['all']) or active != set(document['final']['active']) or active: raise ValueError('generation delta reconstruction differs from final runtime sets')
    if len(document['events']) != document['stats'][0]: raise ValueError('generation event count differs from completed pop count')


def reference_initial_generation(gmsh,face,node_state,face_mesh,triangle_state,queue_state,boundary=None):
    physical = {r[0]:r for r in face_mesh['vertices']}
    nodes = [[r[0],*r[1:3],*physical[r[0]][5:8],physical[r[0]][3],*r[3:5],1] for r in node_state['nodes']]
    triangles = [[*r[:3],r[6],int(r[10]),*r[7:10]] for r in triangle_state['triangles']]
    return normalize_generation(reference_generation(gmsh,face,nodes,triangles,queue_state['initial_all'],node_state['internal_edges'],
        int(boundary is not None),boundary or [],face_mesh['maximum_id']))


def dense_patch(gmsh, face, low, high):
    def uv(x,y): return [float(low[0]+(high[0]-low[0])*(.23+.34*x)),float(low[1]+(high[1]-low[1])*(.23+.34*y))]
    coordinates = [uv(x/3.,y/3.) for y in range(4) for x in range(4)]
    positions = [list(map(float,gmsh.model.getValue(2,face,p))) for p in coordinates]
    length = max(math.dist(positions[0],p) for p in positions[1:])
    if length <= 0: raise ValueError('invalid surface patch length')
    nodes = [[i+1,*coordinates[i],*positions[i],1 if i//4 in (0,3) or i%4 in (0,3) else 2,length*.08,length*.13,1] for i in range(16)]
    triangles = []
    for y in range(3):
        for x in range(3):
            a = y*4+x+1; b,c,d = a+1,a+4,a+5
            triangles.extend([[a,b,c,2.,0,-1,-1,-1],[b,d,c,2.,0,-1,-1,-1]])
    edges = {}
    for i,t in enumerate(triangles):
        for j in range(3):
            key = tuple(sorted((t[(j-1)%3],t[j])))
            if key in edges:
                other,k = edges[key]; triangles[i][5+j] = other; triangles[other][5+k] = i
            else: edges[key] = (i,j)
    def square(x0,y0,x1,y1):
        points = [uv(x0,y0),uv(x1,y0),uv(x1,y1),uv(x0,y1)]
        return [[*points[i],*points[(i+1)%4]] for i in range(4)]
    return nodes,triangles,length,square


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('api','library','own-executable','domain-executable','frozen-dir','out-dir'): parser.add_argument('--'+name,required=True)
    parser.add_argument('--runtime-dir',action='append',default=[])
    parser.add_argument('--fixtures',nargs='+',default=['rectangle','hole','cylinder','sphere','torus'])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    os.environ['PATH'] = os.pathsep.join([*args.runtime_dir,os.environ.get('PATH','')]); os.environ.pop('MESHSTUDIO_GMSH_TRACE',None)
    gmsh = load_reference_api(args.api,args.library)
    sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
    report = {'complete':False,'scope':__doc__,'initial_runtime_frozen':True,'new_geometry_candidates_cavity_insertion_feedback_frozen':False,
              'transfer_smoothing_assembly_validated':False,'full_surface_pipeline_validated':False,
              'identity_normalization':'Each actual allocated global MVertex number maps in creation order to the logical counter, including rejected ghost slots.',
              'library_sha256':sha(args.library),'own_executable_sha256':sha(args.own_executable),'domain_executable_sha256':sha(args.domain_executable),'cases':[]}
    try:
        # Strict numeric products, parity, endpoint/collinearity, duplicated segments and extreme finite scales.
        gmsh.initialize([],readConfigFiles=False)
        try:
            native = gmsh.lib.meshstudioReferenceParametricDomain
            native.argtypes = [ctypes.POINTER(ctypes.c_double),ctypes.c_int,ctypes.POINTER(ctypes.c_double),ctypes.POINTER(ctypes.c_int)]; native.restype = ctypes.c_int
            inputs,expected = [],[]
            square = [[0.,0.,2.,0.],[2.,0.,2.,2.],[2.,2.,0.,2.],[0.,2.,0.,0.]]
            hole = [[.75,.75,1.25,.75],[1.25,.75,1.25,1.25],[1.25,1.25,.75,1.25],[.75,1.25,.75,.75]]
            for segments,point,end,scale in itertools.product(([],square,square+hole,square+[square[2]],square+square,list(reversed(square))),
                ((1.,1.),(.5,.5),(3.,1.),(0.,1.),(0.,0.),(2.,2.),(1.25,1.)),((3.,4.),(3.,3.),(1.,1.),(-2.,4.)),(1.,1.e-90,1.e90)):
                rows = [[v*scale for v in row] for row in segments]; query = [v*scale for v in (*point,*end)]
                flat = [v for row in rows for v in row]; count = ctypes.c_int(-1)
                code = native((ctypes.c_double*max(1,len(flat)))(*flat),len(rows),(ctypes.c_double*4)(*query),ctypes.byref(count))
                if code not in (0,1) or count.value < 0: raise ValueError('native parametric domain failed')
                expected.append([bool(code),count.value]); inputs.append('\n'.join(' '.join(format(x,'.17g') for x in row) for row in [[len(rows),*query],*rows])+'\n')
            frozen = directory/'domain-input.tsv'; frozen.write_text(str(len(inputs))+'\n'+''.join(inputs),encoding='utf-8')
            own = directory/'domain-own.json'; subprocess.run([args.domain_executable,str(frozen),str(own)],check=True)
            actual = json.loads(own.read_text()); difference = first_trace_difference([{'stage':'domain','cases':expected}],[{'stage':'domain','cases':actual}],0.)
            report['domain_queries'] = len(expected); report['domain_difference'] = difference
            if difference: raise ValueError('domain differs:'+json.dumps(difference))
        finally: gmsh.finalize()
        totals = [0]*6; codes = {}; feedback_cases = 0
        for fixture in args.fixtures:
            gmsh.initialize([],readConfigFiles=False)
            try:
                if not gmsh.option.getString('General.Version').startswith('4.15.2'): raise ValueError('wrong fixed version')
                gmsh.option.setNumber('General.Terminal',0); gmsh.option.setNumber('General.NumThreads',1)
                gmsh.option.setNumber('Mesh.Algorithm',6); gmsh.option.setNumber('Mesh.MeshSizeFromPoints',0)
                gmsh.option.setNumber('Mesh.MeshSizeFromCurvature',0); gmsh.option.setNumber('Mesh.MeshSizeMin',0)
                gmsh.model.add(fixture); cad = Path(args.frozen_dir)/(fixture+'.brep')
                gmsh.model.occ.importShapes(str(cad),highestDimOnly=False); gmsh.model.occ.synchronize()
                gmsh.option.setNumber('Mesh.MeshSizeMax',.5); gmsh.model.mesh.generate(1)
                inputs,expected,descriptors = [],[],[]
                for _,face in gmsh.model.getEntities(2):
                    low,high = gmsh.model.getParametrizationBounds(2,face); base_nodes,base_triangles,length,square = dense_patch(gmsh,face,low,high)
                    for extension,mode,name in itertools.product((0,1),(0,1),('no-gate','empty','full','half','hole','duplicate','internal','ghost','deleted','empty-all')):
                        nodes = [row[:] for row in base_nodes]; triangles = [row[:] for row in base_triangles]; internal = []
                        gate = int(name not in ('no-gate','internal','ghost','deleted','empty-all'))
                        boundary = [] if name == 'empty' else square(0.,0.,1.,1.)
                        if name == 'half': boundary = square(0.,0.,.5,1.)
                        if name == 'hole': boundary += square(.33,.33,.66,.66)
                        if name == 'duplicate': boundary += [boundary[2]]
                        if name == 'internal': internal = [[6,7],[7,11]]
                        if name == 'ghost': nodes += [[17,*nodes[0][1:9],0],[18,*nodes[0][1:9],0]]
                        if name == 'deleted': triangles[0][4] = 1
                        all_ids = [] if name == 'empty-all' else list(range(len(triangles)))
                        maximum = len(nodes); maximum_size = length*.13
                        gmsh.option.setNumber('Mesh.MeshSizeMax',maximum_size); gmsh.option.setNumber('Mesh.MeshSizeExtendFromBoundary',extension)
                        gmsh.model.mesh.setSizeCallback(lambda d,t,x,y,z,lc: maximum_size*(.25+.6/(1.+x*x+y*y+z*z)) if mode else maximum_size)
                        reference = reference_generation(gmsh,face,nodes,triangles,all_ids,internal,gate,boundary,maximum)
                        expected.append(normalize_generation(reference)); descriptors.append([face,extension,mode,name])
                        # CTX.lc comes from the actual point leaf without allocating a point (outside patch).
                        from GmshFrontPointInsertionComparison import reference_point_insertion
                        context = reference_point_insertion(gmsh,face,nodes,triangles,list(range(len(triangles))),[],internal,0,[float(high[0]+1.),float(high[1]+1.),1.,0.,1.],maximum,True,False)
                        header = [face,len(nodes),len(triangles),len(all_ids),len(internal),len(boundary),gate,maximum,20000,extension,mode,maximum_size,context['model_length']]
                        rows = [header,*nodes,*triangles,all_ids,*internal,*boundary]
                        inputs.append('\n'.join(' '.join(format(x,'.17g') for x in row) for row in rows)+'\n')
                frozen = directory/(fixture+'-input.tsv'); frozen.write_text(str(len(inputs))+'\n'+''.join(inputs),encoding='utf-8')
                own = directory/(fixture+'-own.json'); subprocess.run([args.own_executable,str(cad),str(frozen),str(own)],check=True)
                actual = json.loads(own.read_text())['cases']; summary = {'fixture':fixture,'queries':len(expected),'cad_sha256':sha(cad),'input_sha256':sha(frozen),'first_difference':None,'stats':[0]*6}
                (directory/(fixture+'-reference.json')).write_text(json.dumps(expected,allow_nan=False),encoding='utf-8')
                if len(actual) != len(expected): raise ValueError('generation case count differs')
                for i,(reference,result) in enumerate(zip(expected,actual)):
                    difference = first_trace_difference([{'stage':'dynamic_front',**reference}],[{'stage':'dynamic_front',**normalize_generation(result)}],0.)
                    if difference: summary['first_difference'] = {'index':i,'case':descriptors[i],'difference':difference}; break
                    for j,value in enumerate(reference['stats']): totals[j] += value; summary['stats'][j] += value
                    feedback_cases += any(e['selection'][0] >= 18 for e in reference['events'])
                    for event in reference['events']:
                        if event['insertion']:
                            code = str(event['insertion']['code']); codes[code] = codes.get(code,0)+1
                report['cases'].append(summary)
                if summary['first_difference']: raise ValueError('generation diverges:'+json.dumps(summary['first_difference']))
            finally: gmsh.finalize()
        report.update(stats=totals,codes=codes,new_triangle_feedback_cases=feedback_cases)
        if not feedback_cases or not totals[3] or not totals[4]: raise ValueError('missing dynamic/gate coverage')
        report['complete'] = True
    except Exception as error: report['error'] = str(error); raise
    finally: (directory/'comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({'complete':True,'domain_queries':report['domain_queries'],'cases':sum(c['queries'] for c in report['cases']),'stats':totals,'codes':codes}))


if __name__ == '__main__': main()
