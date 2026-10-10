"""Actual GFace::XYZtoUV on fixed BREP versus independent OCC source and own Newton/SVD."""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def create_cad(gmsh,name,path):
    gmsh.clear();gmsh.model.add(name)
    if name=='plane':gmsh.model.occ.addRectangle(0.,0.,0.,2.,3.)
    elif name=='cylinder':gmsh.model.occ.addCylinder(0.,0.,0.,0.,0.,3.,1.)
    elif name=='sphere':gmsh.model.occ.addSphere(0.,0.,0.,1.)
    elif name=='torus':gmsh.model.occ.addTorus(0.,0.,0.,3.,1.)
    elif name=='trimmed-plane':
        outer=gmsh.model.occ.addRectangle(0.,0.,0.,2.,3.)
        inner=gmsh.model.occ.addDisk(1.,1.5,0.,.3,.2)
        gmsh.model.occ.cut([(2,outer)],[(2,inner)])
    else:
        points=[]
        for j in range(4):
            for i in range(4):
                u,v=i/3.,j/3.
                points.append(gmsh.model.occ.addPoint(4.*u+1.5*v,3.*v,1.2*u*v+.4*u*u))
        gmsh.model.occ.addBSplineSurface(points,4,degreeU=3,degreeV=3)
    gmsh.model.occ.synchronize();gmsh.write(str(path));gmsh.clear();gmsh.open(str(path))


def reference_inverse(native,face,target,options):
    buffer=ctypes.create_string_buffer(16*1024*1024)
    code=native(face,(ctypes.c_double*3)(*target),*options,buffer,len(buffer))
    if code:raise ValueError('actual GFace inverse failed:'+str(code))
    return json.loads(buffer.value.decode())


def validate_result(result):
    if result['stop'] not in range(4):raise ValueError('unexpected inverse stop')
    if result['seeds']<1 or result['passes']<1 or result['seeds']>81*result['passes']:raise ValueError('inverse seed/pass counts differ')
    if sum(q[0]==1 for q in result['queries'])!=result['steps']:raise ValueError('inverse derivative/step count differs')
    for q in result['queries']:
        if q[0] not in (0,1) or len(q)!=(6 if q[0]==0 else 9) or not all(map(math.isfinite,q)):
            raise ValueError('invalid finite inverse query')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('api','library','own-executable','out-dir'):parser.add_argument('--'+name,required=True)
    parser.add_argument('--runtime-dir',action='append',default=[])
    args=parser.parse_args();directory=Path(args.out_dir);directory.mkdir(parents=True,exist_ok=False)
    handles=[os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    os.environ['PATH']=os.pathsep.join(args.runtime_dir)+os.pathsep+os.environ.get('PATH','')
    gmsh=load_reference_api(args.api,args.library)
    native=gmsh.lib.meshstudioReferenceSurfaceParameterInverse
    native.argtypes=[ctypes.c_int,ctypes.POINTER(ctypes.c_double),ctypes.c_double,ctypes.c_double,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_char),ctypes.c_int]
    native.restype=ctypes.c_int
    report={'scope':'actual GFace base inverse explicitly called on OCC faces; independent BREP trim bounds/Value/D1 and own SVD/Newton; no natural direct projector failure/closestPoint/curve inverse or full CAD generation',
            'complete':False,'cases':[],'library_sha256':hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
            'own_executable_sha256':hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest(),
            'natural_occ_failure_validated':False,'full_generation_validated':False,'absolute_tolerance':0.}
    gmsh.initialize([],readConfigFiles=False)
    try:
        gmsh.option.setNumber('General.Terminal',0.)
        for fixture in ('plane','cylinder','sphere','warped-bspline','torus','trimmed-plane'):
            cad=directory/(fixture+'.brep');create_cad(gmsh,fixture,cad)
            rows=[];expected=[];names=[]
            for _,face in gmsh.model.getEntities(2):
                low,high=gmsh.model.getParametrizationBounds(2,face)
                def point(u,v):
                    uv=[float(low[0]+(high[0]-low[0])*u),float(low[1]+(high[1]-low[1])*v)]
                    return list(map(float,gmsh.model.getValue(2,face,uv)))
                selected=[]
                for index,(u,v) in enumerate(((.5,.5),(.2,.3),(.8,.7),(0.,0.),(1.,1.))):
                    target=point(u,v)
                    selected.append((f'on-{index}',target,(1.,1.,1,0,0)))
                    if index==1:
                        selected.append(('on-relax',target,(1.,.5,1,1,0)))
                        selected.append(('on-closest',target,(1.,1.,0,0,0)))
                center=point(.5,.5)
                shifted=[center[0]+.031,center[1]-.043,center[2]+.057]
                selected.extend([('off-warning',shifted,(1.,1.,1,0,0)),('off-strict',shifted,(1.,1.,1,1,0)),
                                 ('off-option-strict',shifted,(1.,1.,1,0,1)),('off-closest',shifted,(1.,1.,0,0,0)),
                                 ('outside-closest',[center[0]+30.,center[1]+30.,center[2]+30.],(1.,1.,0,0,0))])
                for name,target,options in selected:
                    names.append(f'{fixture}-face{face}-{name}');rows.append([face,*target,*options])
                    result=reference_inverse(native,face,target,options);validate_result(result);expected.append(result)
            source=directory/(fixture+'-input.tsv');target=directory/(fixture+'-own.json')
            source.write_text(str(len(rows))+'\n'+''.join(' '.join(str(v) if isinstance(v,int) else format(v,'.17g') for v in row)+'\n' for row in rows),encoding='utf-8')
            (directory/(fixture+'-reference.json')).write_text(json.dumps(expected,allow_nan=False),encoding='utf-8')
            subprocess.run([args.own_executable,str(cad),str(source),str(target)],check=True,timeout=120)
            actual=json.loads(target.read_text(encoding='utf-8'))['cases']
            if len(actual)!=len(expected):raise ValueError('inverse case count differs')
            for name,reference,own in zip(names,expected,actual):
                validate_result(own)
                difference=first_trace_difference([reference],[own],0.)
                report['cases'].append({'name':name,'stop':reference['stop'],'passes':reference['passes'],'seeds':reference['seeds'],'steps':reference['steps'],
                                        'queries':len(reference['queries']),'first_difference':difference})
                if difference:raise ValueError('surface inverse diverged:'+name+' '+json.dumps(difference))
        report['complete']=True
    finally:
        (directory/'surface-inverse-report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        gmsh.finalize()
    print(json.dumps({'complete':True,'count':len(report['cases']),'queries':sum(c['queries'] for c in report['cases'])}))


if __name__=='__main__':main()
