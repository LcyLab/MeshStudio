"""直接调用固定transferDataStructure，与自研前沿结束池转移逐值比较。"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def reference_transfer(gmsh, face, nodes, triangles, all_ids, equivalence, has_equivalence, physical):
    native = gmsh.lib.meshstudioReferenceFrontMeshTransfer
    p = ctypes.POINTER(ctypes.c_double)
    native.argtypes = [ctypes.c_int,p,ctypes.c_int,p,ctypes.c_int,ctypes.POINTER(ctypes.c_int),ctypes.c_int,
                       p,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_char),ctypes.c_int]
    native.restype = ctypes.c_int
    def doubles(rows):
        flat = [x for row in rows for x in row]
        return (ctypes.c_double*max(1,len(flat)))(*flat)
    ids = (ctypes.c_int*max(1,len(all_ids)))(*all_ids)
    output = ctypes.create_string_buffer(4*1024*1024)
    code = native(face,doubles(nodes),len(nodes),doubles(triangles),len(triangles),ids,len(all_ids),
                  doubles(equivalence),len(equivalence),int(has_equivalence),int(physical),output,len(output))
    if code: raise ValueError('native transfer failed:'+str(code))
    return json.loads(output.value.decode())


def write_transfer_input(path, nodes, triangles, all_ids, equivalence, has_equivalence, physical):
    rows = [[len(nodes),len(triangles),len(all_ids),len(equivalence),int(has_equivalence),int(physical)],
            *nodes,*triangles,all_ids,*equivalence]
    path.write_text(''.join(' '.join(format(x,'.17g') for x in row)+'\n' for row in rows),encoding='utf-8')


def generation_transfer_input(generation, equivalence, extra_vertices=()):
    final = generation['final']
    vertices = {row[0]:row[1:4] for row in generation.get('vertices',final.get('vertices',[]))}
    for row in extra_vertices: vertices.setdefault(row[0],row[5:8])
    slots = {row[0]:final['slots'][row[1]][1:3] for row in final['live_indices']}
    nodes = [[id,*slots.get(id,[0.,0.]),*point] for id,point in sorted(vertices.items())]
    triangles = [row[:5] for row in final['pool']]
    return nodes,triangles,final['all'],equivalence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('api','library','own-executable','out-dir'): parser.add_argument('--'+name,required=True)
    parser.add_argument('--prefix-dir',action='append',default=[])
    parser.add_argument('--runtime-dir',action='append',default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=True)
    handles = [os.add_dll_directory(x) for x in args.runtime_dir] if hasattr(os,'add_dll_directory') else []
    gmsh = load_reference_api(args.api,args.library)
    report = {'complete':False,'cases':[],'prefix_cases':[],
              'library_sha256':hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              'own_executable_sha256':hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest(),
              'scope':'Actual native transferDataStructure. Independent native/own prefix end pools replayed separately; raw 1D/chart metadata remains frozen in prefix. No smoothing, recovery retry, final assembly or general shape acceptance.'}
    gmsh.initialize(['MeshStudioTransferReference']); gmsh.option.setNumber('General.Terminal',0)
    try:
        gmsh.model.add('transfer-leaf'); face = gmsh.model.occ.addRectangle(0.,0.,0.,1.,1.); gmsh.model.occ.synchronize()
        def compare(name, native_input, own_input, has_equivalence, physical, prefix=False, provenance=None):
            expected = reference_transfer(gmsh,face,*native_input,has_equivalence,physical)
            source = directory/(name+'-input.tsv'); target = directory/(name+'-own.json')
            write_transfer_input(source,*own_input,has_equivalence,physical)
            subprocess.run([args.own_executable,str(source),str(target)],check=True,timeout=30)
            actual = json.loads(target.read_text(encoding='utf-8'))
            difference = first_trace_difference([{'stage':'transfer',**expected}],[{'stage':'transfer',**actual}],0.)
            (directory/(name+'-reference.json')).write_text(json.dumps(expected,allow_nan=False),encoding='utf-8')
            row = {'name':name,'triangles':len(expected['triangles']),'deleted':len(expected['deleted']),
                   'reversed':len(expected['reversed']),'collapsed':len(expected['collapsed']),
                   'equivalent_pairs':len(expected['equivalent']),'equivalence':has_equivalence,'physical':physical,
                   'input_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'first_difference':difference}
            if provenance: row['provenance'] = provenance
            report['prefix_cases' if prefix else 'cases'].append(row)
            if difference: raise ValueError('transfer divergence:'+name+':'+json.dumps(difference))
        base = [[1,0.,0.,0.,0.,0.],[2,1.,0.,1.,0.,0.],[3,0.,1.,0.,1.,0.],
                [4,-1.,0.,-1.,0.,0.],[5,2.,0.,2.,0.,0.],[6,2.,1.,2.,1.,0.]]
        for scale in (1.,1.e-100,1.e50):
            for kind in ('normal','first-degenerate','deleted','unregistered','duplicate-set','nonplanar'):
                nodes = [[row[0],*(x*scale for x in row[1:])] for row in base]
                triangles = [[1,2,3,4.,0],[2,1,3,3.,0],[4,2,3,2.,0],[1,4,5,1.,0]]
                all_ids = list(range(4))
                if kind == 'first-degenerate': triangles[0][:3] = [1,2,5]
                if kind == 'deleted': triangles[0][4] = triangles[2][4] = 1
                if kind == 'unregistered': all_ids = [1,3]
                if kind == 'duplicate-set': triangles += [triangles[0][:]]; all_ids += [4]
                if kind == 'nonplanar': nodes[3][3:] = [-1.*scale,0.,.2*scale]
                for mapping in (None,[],[[4,1]],[[4,1],[1,6]]):
                    for physical in (False,True):
                        name = 'leaf-'+str(len(report['cases']))
                        frozen = (nodes,triangles,all_ids,mapping or [])
                        compare(name,frozen,frozen,mapping is not None,physical)
        for eq in (False,True):
            frozen = ([],[],[],[]); compare('empty-'+str(eq),frozen,frozen,eq,False)
        for source_dir in map(Path,args.prefix_dir):
            periodic = any(source_dir.glob('*front_queue-own.json'))
            paths = sorted(source_dir.glob('*front_queue-own.json' if periodic else '*face*-own.json'))
            for path in paths:
                own_doc = json.loads(path.read_text(encoding='utf-8')); ref_path = path.with_name(path.name.replace('-own.json','-reference.json'))
                ref_doc = json.loads(ref_path.read_text(encoding='utf-8'))
                own_gen = own_doc['periodic_front_generation' if periodic else 'front_generation']; ref_gen = ref_doc['front_generation']
                own_face = own_doc['periodic_face_mesh'] if periodic else {'equivalence':[],'vertices':[]}
                if periodic:
                    face_path = path.with_name(path.name.replace('-front_queue-own.json','-face_mesh-reference.json'))
                    native_face = json.loads(face_path.read_text(encoding='utf-8'))
                else: native_face = own_face
                native_input = generation_transfer_input(ref_gen,native_face['equivalence'],native_face['vertices'])
                own_input = generation_transfer_input(own_gen,own_face['equivalence'],own_face['vertices'])
                name = 'prefix-'+str(len(report['prefix_cases']))
                compare(name,native_input,own_input,periodic,False,True,
                        {'own_pool':str(path),'native_pool':str(ref_path),'own_pool_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                         'native_pool_sha256':hashlib.sha256(ref_path.read_bytes()).hexdigest()})
        if len(report['cases']) != 146: raise ValueError('transfer leaf matrix count differs')
        if not any(c['equivalent_pairs'] and c['collapsed'] for c in report['cases']): raise ValueError('equivalence coverage absent')
        if args.prefix_dir and len(report['prefix_cases']) != 168: raise ValueError('expected all 168 completed prefix pools')
        report['complete'] = True
    finally:
        gmsh.finalize(); (directory/'front-transfer-report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        for handle in handles: handle.close()
    print(json.dumps({'complete':report['complete'],'leaf_cases':len(report['cases']),'prefix_cases':len(report['prefix_cases'])}))


if __name__ == '__main__': main()
