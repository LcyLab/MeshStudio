"""固定4.15.2初始BDS面积、数组度量圆与Quality(false)真实叶函数对照。"""
import argparse
import copy
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess
from GmshBdsMeshConversionComparison import normalize, write_input
from GmshBdsConstraintRecoveryComparison import numeric
from GmshReproductionComparison import load_reference_api, first_trace_difference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "out-dir"): parser.add_argument("--"+name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library); dp = ctypes.POINTER(ctypes.c_double); ip = ctypes.POINTER(ctypes.c_int)
    area = gmsh.lib.meshstudioReferenceBdsParameterArea; area.argtypes = [dp]; area.restype = ctypes.c_double
    circle = gmsh.lib.meshstudioReferenceInitialMetricCircle; circle.argtypes = [dp, dp, dp]; circle.restype = ctypes.c_int
    swap = gmsh.lib.meshstudioReferenceInitialBdsSwap
    swap.argtypes = [ctypes.c_int, dp, ctypes.c_int, ip, ctypes.c_int, ip, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_char), ctypes.c_int]; swap.restype = ctypes.c_int
    report = {"scope": "native surface_triangle_param, array inCircumCircleAniso and actual BDS swap_edge Quality(false,true) leaves",
              "complete": False, "area_cases": 0, "circle_cases": 0, "swap_cases": [],
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest(),
              "full_periodic_driver_validated": False, "full_cad_pipeline_validated": False}
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)

    def check(name, lines, expected):
        source = directory/(name+"-input.tsv"); write_input(source, lines)
        target = directory/(name+"-own.json")
        (directory/(name+"-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
        subprocess.run([args.own_executable, str(source), str(target)], check=True, timeout=30)
        actual = json.loads(target.read_text(encoding="utf-8"))
        if isinstance(expected, dict): normalize(actual["mesh"])
        elif name == "area": actual = [x if isinstance(x, str) else float(x) for x in actual]
        else: actual = [[float(x) for x in row] for row in actual]
        difference = first_trace_difference([{"stage": name, "values": expected}], [{"stage": name, "values": actual}], 0.)
        if difference: raise ValueError(name+" diverged: "+json.dumps(difference))

    gmsh.initialize([], readConfigFiles=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0.)
        rows = []
        for scale in (1.e-160, 1.e-8, 1., 1.e8, 1.e160):
            for order in itertools.permutations(range(3)):
                for flags in ((0,0,0), (1,0,0), (0,1,0), (0,0,1), (2,0,0), (0,2,0), (0,0,2), (1,2,0), (3,0,0)):
                    base = [[0.,0.,flags[0]], [2.*scale,scale,flags[1]], [4.*scale,3.*scale,flags[2]]]
                    rows.append([x for i in order for x in base[i]])
        expected = [numeric(area((ctypes.c_double*9)(*r))) for r in rows]
        check("area", [["area",len(rows)], *rows], expected); report["area_cases"] = len(rows)
        rows = []
        base = [[0.,0.],[2.,0.],[0.,2.]]
        for order in itertools.permutations(range(3)):
            for scale in (1.e-4, 1., 100., 1000.):
                for offset in (0., 1.e4):
                    for metric in ((1.,0.,1.), (16.,6.,11.25), (1.,.99,1.), (0.,0.,0.), (1.,0.,0.)):
                        for query in ([1.,1.], [2.,2.], [3.,3.]):
                            points = [base[i] for i in order]+[query]
                            rows.append([coordinate*scale+offset for p in points for coordinate in p]+list(metric))
        for m in (500., 501., 50000., 50001.): rows.append([0.,0.,2.,0.,0.,2.,1.,1.,m,0.,m])
        rows.append([5.,5.,6.,5.,7.,5.,0.,0.,1.,0.,1.])
        expected = []
        for r in rows:
            result = (ctypes.c_double*6)()
            if circle((ctypes.c_double*8)(*r[:8]), (ctypes.c_double*3)(*r[8:]), result) != 6: raise ValueError("native circle failure")
            expected.append([float(x) for x in result])
        check("circle", [["circle",len(rows)], *rows], expected); report["circle_cases"] = len(rows)

        def run_swap(name, points, triangles, labels=(), first=1, second=3):
            p = (ctypes.c_double*(14*len(points)))(*(x for row in points for x in row))
            t = (ctypes.c_int*(3*len(triangles)))(*(x for row in triangles for x in row))
            l = (ctypes.c_int*max(1,5*len(labels)))(*(x for row in labels for x in row))
            result = ctypes.create_string_buffer(1024*1024)
            if swap(len(points),p,len(triangles),t,len(labels),l,first,second,result,len(result)) < 0: raise ValueError(name+" native swap failure")
            expected = json.loads(result.value.decode("utf-8")); normalize(expected["mesh"])
            check(name, [["swap",len(points)], *points, [len(triangles)], *triangles, [len(labels)], *labels, [first,second]], expected)
            report["swap_cases"].append({"name": name,"swapped": expected["swapped"],"first_difference": None})

        base = [[i,0,u,v,u,v,0.,-1,0,.3,.3,-1,1,0] for i,(u,v) in enumerate(((0.,0.),(2.,0.),(2.,1.),(0.,2.)))]
        for order in itertools.permutations(range(3)):
            triangles = [[(0,1,3)[i] for i in order], [1,2,3]]
            run_swap("order-"+"".join(map(str,order)),base,triangles)
            run_swap("curve-"+"".join(map(str,order)),base,triangles,[(0,1,3,11,1)])
            run_swap("face-label-"+"".join(map(str,order)),base,triangles,[(1,0,0,7,2),(1,1,0,8,2),(0,1,3,9,2)])
        triangles = [[0,1,3],[1,2,3]]
        for dimension in (0,1,2,3):
            for endpoint in (1,3):
                points = copy.deepcopy(base); points[endpoint][7:9] = [dimension,11]
                run_swap(f"small-{dimension}-{endpoint}",points,triangles)
        points = copy.deepcopy(base)
        for p in points: p[12] = 0
        run_swap("all-unmodified",points,triangles)
        points[0][12] = 1; run_swap("opposite-only-modified",points,triangles)
        for first_flag, second_flag in ((1,0),(0,1),(1,1),(1,2),(2,2),(3,0)):
            points = copy.deepcopy(base); points[0][13] = first_flag; points[2][13] = second_flag
            run_swap(f"pole-{first_flag}-{second_flag}",points,triangles)
        for first_ptr,second_ptr in ((2,-1),(-1,0),(2,0),(1,3)):
            points = copy.deepcopy(base); points[0][11] = first_ptr; points[2][11] = second_ptr
            run_swap(f"counterpart-{first_ptr}-{second_ptr}",points,triangles)
        points = copy.deepcopy(base); points[2][2:7] = [-1.,-1.,-1.,-1.,0.]
        run_swap("nonconvex-area",points,triangles)
        run_swap("one-face",base,triangles,first=0,second=1)
        run_swap("duplicate-two-faces",base,[[0,1,3],[0,1,3]])
        # 让两旧端点超过4条邻边，验证小连接度保护解除，且交换只影响原双面。
        points = copy.deepcopy(base); expanded = copy.deepcopy(triangles)
        for endpoint,ids in ((1,(10,11,12)),(3,(20,21,22))):
            points[endpoint][7:9] = [2,7]
            for k,id in enumerate(ids): points.append([id,0,10.+k,endpoint,10.+k,endpoint,0.,-1,0,.3,.3,-1,1,0])
            expanded.extend([[endpoint,ids[0],ids[1]],[endpoint,ids[1],ids[2]]])
        run_swap("high-valence",points,expanded)
        if report["area_cases"] != 270 or report["circle_cases"] != 725 or len(report["swap_cases"]) != 42: raise ValueError("fixed leaf count changed")
        report["complete"] = True
    finally:
        gmsh.finalize()
        (directory/"initial-metric-leaf-report.json").write_text(json.dumps(report,indent=2)+"\n", encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"area_cases": report["area_cases"], "circle_cases": report["circle_cases"], "swap_cases": len(report["swap_cases"]), "complete": report["complete"]}))


if __name__ == "__main__": main()
