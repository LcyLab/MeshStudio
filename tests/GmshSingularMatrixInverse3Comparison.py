"""Fixed actual invert_singular_matrix3x3 and SVD factors; independent own 3x3 kernel."""
import argparse
import ctypes
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import random
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def numerical_cases():
    cases=[]
    def append(name,matrix): cases.append({'name':name,'matrix':list(matrix)})
    append('zero',[0.]*9)
    base=((1.,0.,0.,0.,1.,0.,0.,0.,1.),(1.,2.,3.,0.,1.,4.,5.,6.,0.),
          (3.,0.,0.,4.,0.,0.,0.,0.,0.),(1.,0.,0.,1.,1.,0.,0.,0.,0.),
          (1.,2.,3.,2.,4.,6.,0.,0.,0.))
    for scale in (1.e-100,1.e-16,1.,1.e100):
        for n,matrix in enumerate(base):
            for permutation in itertools.permutations(range(3)):
                append(f'base{n}-scale{scale}-rows{permutation}',[scale*matrix[3*p+j] for p in permutation for j in range(3)])
    for value in (0.,1.e-18,math.nextafter(1.e-16,0.),1.e-16,math.nextafter(1.e-16,1.),1.e-14,1.):
        append('cutoff-'+str(value),[1.,0.,0.,0.,value,0.,0.,0.,-value])
    rng=random.Random(20261008)
    for n in range(180):
        dense=[rng.uniform(-4.,4.) for _ in range(9)]
        append('dense-'+str(n),dense)
        append('tangent-'+str(n),dense[:6]+[0.,0.,0.])
    return cases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('api','library','own-executable','out-dir'): parser.add_argument('--'+name,required=True)
    parser.add_argument('--runtime-dir',action='append',default=[])
    args=parser.parse_args(); directory=Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles=[os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh=load_reference_api(args.api,args.library)
    native=gmsh.lib.meshstudioReferenceSingularMatrixInverse3
    native.argtypes=[ctypes.POINTER(ctypes.c_double)]*2; native.restype=ctypes.c_int
    report={'scope':'actual native three-dimensional pseudoinverse and all SVD factors; frozen numeric matrix including CAD D1 samples; no surface/curve Newton or OCC fallback integration',
            'complete':False,'library_sha256':hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
            'own_executable_sha256':hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest(),'cases':[],
            'surface_inverse_validated':False,'occ_fallback_validated':False,'absolute_tolerance':0.}
    gmsh.initialize([],readConfigFiles=False)
    try:
        gmsh.option.setNumber('General.Terminal',0.)
        cases=numerical_cases()
        # Use actual CAD derivatives as matrix inputs, not a replacement SVD implementation.
        for fixture in ('plane','cylinder','sphere','warped-bspline','torus'):
            gmsh.clear();gmsh.model.add(fixture)
            if fixture=='plane': gmsh.model.occ.addRectangle(0.,0.,0.,2.,3.)
            elif fixture=='cylinder': gmsh.model.occ.addCylinder(0.,0.,0.,0.,0.,3.,1.)
            elif fixture=='sphere': gmsh.model.occ.addSphere(0.,0.,0.,1.)
            elif fixture=='torus': gmsh.model.occ.addTorus(0.,0.,0.,3.,1.)
            else:
                points=[]
                for j in range(4):
                    for i in range(4):
                        u,v=i/3.,j/3.
                        points.append(gmsh.model.occ.addPoint(4.*u+1.5*v,3.*v,1.2*u*v+.4*u*u))
                gmsh.model.occ.addBSplineSurface(points,4,degreeU=3,degreeV=3)
            gmsh.model.occ.synchronize();gmsh.write(str(directory/(fixture+'.brep')))
            for _,face in gmsh.model.getEntities(2):
                low,high=gmsh.model.getParametrizationBounds(2,face)
                for u,v in ((0.,0.),(.2,.3),(.5,.5),(.8,.7),(1.,1.)):
                    uv=[float(low[0]+(high[0]-low[0])*u),float(low[1]+(high[1]-low[1])*v)]
                    matrix=list(map(float,gmsh.model.getDerivative(2,face,uv)))+[0.,0.,0.]
                    if len(matrix)!=9 or not all(map(math.isfinite,matrix)): raise ValueError('invalid CAD derivative input')
                    cases.append({'name':f'{fixture}-face{face}-uv{u},{v}','matrix':matrix})
        expected=[]
        for case in cases:
            output=(ctypes.c_double*31)();code=native((ctypes.c_double*9)(*case['matrix']),output)
            if code!=31 or not all(map(math.isfinite,output)): raise ValueError('native incomplete inverse:'+case['name'])
            expected.append(list(output))
        source=directory/'matrices-input.tsv';target=directory/'matrices-own.json'
        source.write_text(''.join(' '.join(format(x,'.17g') for x in c['matrix'])+'\n' for c in cases),encoding='utf-8')
        (directory/'matrices-reference.json').write_text(json.dumps(expected,allow_nan=False),encoding='utf-8')
        (directory/'matrices-cases.json').write_text(json.dumps(cases,indent=2,allow_nan=False),encoding='utf-8')
        subprocess.run([args.own_executable,str(source),str(target)],check=True,timeout=60)
        actual=json.loads(target.read_text(encoding='utf-8'))['cases']
        if len(actual)!=len(expected): raise ValueError('numerical case count differs')
        for case,reference,own in zip(cases,expected,actual):
            difference=first_trace_difference([reference],[own['values']],0.)
            report['cases'].append({'name':case['name'],'retained':int(reference[30]),'sweeps':own['sweeps'],'first_difference':difference})
            if difference: raise ValueError('native inverse differs:'+case['name']+' '+json.dumps(difference))
        report.update(complete=True,count=len(cases),cad_derivative_cases=sum('-face' in c['name'] for c in cases),input_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    finally:
        (directory/'singular-inverse-report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        gmsh.finalize()
    print(json.dumps({'complete':report['complete'],'count':report['count'],'cad_derivative_cases':report['cad_derivative_cases']}))


if __name__=='__main__': main()
