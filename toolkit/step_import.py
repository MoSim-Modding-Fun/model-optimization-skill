# STEP -> mesh dump for step_build.py. Runs in a Python that has OpenCASCADE bindings (cadquery-ocp), NOT in
# Blender:
#   occ_venv/bin/python step_import.py export robot.step out_dir [lin_mm ang_rad] [--only REGEX]
# The source STEP is only read. export writes out_dir/tree.json (assembly nodes: names, local transforms, mesh refs) and
# out_dir/meshes.npz (per unique solid: vertices in metres, triangles, per-triangle material index).
# One mesh per (part label, inherited style) and solid, shared by every instance, like the glTF imports the pipeline was
# tuned on (instancing keeps Phase 1 per unique mesh). Vertices are NOT welded here (OCC meshes face by face):
# step_build.py welds at 1 um.
import json, os, re, sys, time
import numpy as np


def _name(label):
    from OCP.TDataStd import TDataStd_Name
    a = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), a):
        return a.Get().ToExtString()
    return ""


def _entry(label):
    from OCP.TDF import TDF_Tool
    from OCP.TCollection import TCollection_AsciiString
    s = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, s)
    return s.ToCString()


def _app():
    from OCP.XCAFApp import XCAFApp_Application
    app = XCAFApp_Application.GetApplication_s()
    return app


def read(step):
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.IFSelect import IFSelect_RetDone
    app = _app()
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    app.InitDocument(doc)
    r = STEPCAFControl_Reader()
    r.SetColorMode(True); r.SetNameMode(True); r.SetLayerMode(False); r.SetPropsMode(False)
    t = time.time()
    st = r.ReadFile(step)
    print(f"ReadFile {st} {time.time() - t:.0f}s", flush=True)
    if st != IFSelect_RetDone:
        raise SystemExit("read failed")
    t = time.time()
    ok = r.Transfer(doc)
    print(f"Transfer {ok} {time.time() - t:.0f}s", flush=True)
    return app, doc


def _mat(trsf, scale=1e-3):
    if hasattr(trsf, "Transformation"):
        trsf = trsf.Transformation()  # TopLoc_Location -> gp_Trsf
    m = [[trsf.Value(r, c) for c in range(1, 5)] for r in range(1, 4)]
    for r in range(3):
        m[r][3] *= scale  # mm -> m
    return m + [[0.0, 0.0, 0.0, 1.0]]


def _rgba(c):
    """Quantity_ColorRGBA -> sRGB 0..1 (the STEP file's own values) + alpha, rounded."""
    from OCP.Quantity import Quantity_TypeOfColor
    rgb = c.GetRGB()
    v = rgb.Values(Quantity_TypeOfColor.Quantity_TOC_sRGB)
    return tuple(round(float(x), 6) for x in v) + (round(float(c.Alpha()), 6),)


def export(step, out, lin_mm=0.05, ang_rad=0.25, only=None):
    from OCP.XCAFPrs import XCAFPrs_DocumentExplorer, XCAFPrs_DocumentExplorerFlags_None
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
    from OCP.RWMesh import RWMesh_FaceIterator
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_SHELL
    os.makedirs(out, exist_ok=True)
    app, doc = read(step)  # (an XBF save/reopen came back empty in OCP 8: re-read the STEP, ~1 min)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    rx = re.compile(only) if only else None
    nodes, meshes, mats = [], {}, {}
    arrays = {}
    parts = {}  # ref entry -> part index (deterministic: first-visit order)
    meshed = set()
    stack = []  # (depth, node index)
    exp = XCAFPrs_DocumentExplorer(doc, XCAFPrs_DocumentExplorerFlags_None)
    t0 = time.time()
    ntri = 0
    while exp.More():
        n = exp.Current()
        depth = exp.CurrentDepth()
        while stack and stack[-1][0] >= depth:
            stack.pop()
        parent = stack[-1][1] if stack else -1
        ref = n.RefLabel
        rent = _entry(ref)
        style = n.Style
        scol = _rgba(style.GetColorSurfRGBA()) if style.IsSetColorSurf() else None
        node = {"id": n.Id.ToCString(), "parent": parent, "inst": _name(n.Label), "prod": _name(ref),
                "asm": bool(n.IsAssembly), "local": _mat(n.LocalTrsf), "ref": rent, "style": scol}
        idx = len(nodes)
        nodes.append(node)
        stack.append((depth, idx))
        if not n.IsAssembly:
            pi = parts.setdefault(rent, len(parts))
            key = (rent, scol)
            if key not in meshes:
                if rx and not rx.search(node["prod"]):
                    meshes[key] = None
                else:
                    shape = XCAFDoc_ShapeTool.GetShape_s(ref)
                    if rent not in meshed:
                        BRepMesh_IncrementalMesh(shape, lin_mm, False, ang_rad, True)
                        meshed.add(rent)
                    # face -> solid index (faces outside any solid: shells / free faces -> one extra group)
                    solid_of = {}
                    solids = []
                    e = TopExp_Explorer(shape, TopAbs_SOLID)
                    while e.More():
                        s = e.Current()
                        si = len(solids)
                        solids.append(s)
                        f = TopExp_Explorer(s, TopAbs_FACE)
                        while f.More():
                            solid_of.setdefault(hash(f.Current()), si)
                            f.Next()
                        e.Next()
                    nsol = max(1, len(solids))
                    V = [[] for _ in range(nsol + 1)]
                    T = [[] for _ in range(nsol + 1)]
                    M = [[] for _ in range(nsol + 1)]
                    cnt = [0] * (nsol + 1)
                    it = RWMesh_FaceIterator(ref, TopLoc_Location(), True, style)
                    while it.More():
                        if not it.IsEmptyMesh():
                            si = solid_of.get(hash(it.Face()), nsol)
                            col = _rgba(it.FaceColor()) if it.HasFaceColor() else (0.8, 0.8, 0.8, 1.0)
                            mi = mats.setdefault(col, len(mats))
                            lo, hi = it.NodeLower(), it.NodeUpper()
                            base = cnt[si] - lo
                            for i in range(lo, hi + 1):
                                p = it.NodeTransformed(i)
                                V[si].append((p.X(), p.Y(), p.Z()))
                            cnt[si] += hi - lo + 1
                            for i in range(it.ElemLower(), it.ElemUpper() + 1):
                                a, b, c = it.TriangleOriented(i).Get()
                                T[si].append((a + base, b + base, c + base))
                                M[si].append(mi)
                        it.Next()
                    sol = []
                    for si in range(nsol + 1):
                        if not T[si]:
                            continue
                        name = f"m{pi:04d}s{si:02d}" + ("" if scol is None else f"c{sum(1 for k in meshes if k[0] == rent)}")
                        arrays[name + "_v"] = (np.asarray(V[si], np.float64) * 1e-3).astype(np.float32)
                        arrays[name + "_t"] = np.asarray(T[si], np.int32)
                        arrays[name + "_m"] = np.asarray(M[si], np.int16)
                        ntri += len(T[si])
                        sol.append({"mesh": name, "solid": si if si < nsol else -1, "tris": len(T[si])})
                    meshes[key] = sol
            node["solids"] = meshes[key]
            node["part"] = pi
        exp.Next()
        if len(nodes) % 200 == 0:
            print(f"  {len(nodes)} nodes, {len(meshed)} parts meshed, {ntri:,} tris, {time.time() - t0:.0f}s", flush=True)
    tree = {"nodes": nodes, "materials": [list(k) for k, _ in sorted(mats.items(), key=lambda kv: kv[1])],
            "lin_mm": lin_mm, "ang_rad": ang_rad, "tris": ntri}
    with open(os.path.join(out, "tree.json"), "w") as f:
        json.dump(tree, f)
    np.savez(os.path.join(out, "meshes.npz"), **arrays)
    print(f"export: {len(nodes)} nodes, {len(parts)} parts, {sum(1 for v in meshes.values() if v)} mesh variants, "
          f"{len(arrays) // 3} meshes, {ntri:,} unique tris, {len(mats)} colours, {time.time() - t0:.0f}s")


if __name__ == "__main__":
    a = sys.argv[1:]
    only = None
    if "--only" in a:
        i = a.index("--only"); only = a[i + 1]; del a[i:i + 2]
    if a[0] == "export":
        export(a[1], a[2], *(float(x) for x in a[3:5]), only=only)
