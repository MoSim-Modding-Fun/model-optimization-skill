# Prepare a cleaned CAD file for the per-part pipeline:
#  1. objects whose meshes are identical (same local geometry + materials) share ONE mesh datablock, so the pipeline
#     plans each geometry once (the glTF export gave every instance its own copy);
#  2. merged sub-assemblies (cfg.EXPLODE part names) are split into their loose pieces: "<object>~NNN" children
#     with identity local transforms (copies of an assembly share the piece meshes). The assembly object keeps its
#     name/parent/transform but gets an empty mesh; implode.py joins the processed pieces back into it.
#   blender -b cleaned.blend --python explode.py -- exploded.blend explode.json
import bpy, bmesh, sys, os, json, time, collections
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import robots
cfg = robots.load()


def arr(coll, attr, n, dt, k=1):
    a = np.empty(n * k, dt)
    coll.foreach_get(attr, a)
    return a.reshape(-1, k) if k > 1 else a


def geo_hash(me):
    import hashlib
    nv, nl, npo = len(me.vertices), len(me.loops), len(me.polygons)
    h = hashlib.sha1()
    for x in (np.round(arr(me.vertices, "co", nv, np.float32, 3).astype(np.float64), 6), arr(me.loops, "vertex_index", nl, np.int32),
              arr(me.polygons, "loop_total", npo, np.int32), arr(me.polygons, "material_index", npo, np.int32)):
        h.update(np.ascontiguousarray(x).tobytes())
    h.update("|".join(m.name if m else "-" for m in me.materials).encode())
    return h.hexdigest()[:16]


def islands(me):
    """Face-index lists of the connected components (shared vertices) of a mesh."""
    nv = len(me.vertices)
    ev = arr(me.edges, "vertices", len(me.edges), np.int32, 2)
    lab = np.arange(nv)
    while True:
        old = lab.copy()
        m = np.minimum(lab[ev[:, 0]], lab[ev[:, 1]])
        np.minimum.at(lab, ev[:, 0], m)
        np.minimum.at(lab, ev[:, 1], m)
        while True:
            nl = lab[lab]
            if np.array_equal(nl, lab):
                break
            lab = nl
        if np.array_equal(lab, old):
            break
    ls = arr(me.polygons, "loop_start", len(me.polygons), np.int32)
    lv = arr(me.loops, "vertex_index", len(me.loops), np.int32)
    flab = lab[lv[ls]]
    order = np.argsort(flab, kind="stable")
    _, starts = np.unique(flab[order], return_index=True)
    return np.split(order, starts[1:])


FIELD = {'FLOAT': "value", 'INT': "value", 'BOOLEAN': "value", 'INT8': "value", 'FLOAT_VECTOR': "vector",
         'FLOAT2': "vector", 'FLOAT_COLOR': "color", 'BYTE_COLOR': "color", 'INT32_2D': "value", 'QUATERNION': "value"}
WIDTH = {'FLOAT_VECTOR': 3, 'FLOAT2': 2, 'FLOAT_COLOR': 4, 'BYTE_COLOR': 4, 'INT32_2D': 2, 'QUATERNION': 4}
DTYPE = {'INT': np.int32, 'INT8': np.int32, 'BOOLEAN': bool, 'INT32_2D': np.int32}


class MeshArrays:
    """Everything needed to cut sub-meshes out of a big mesh without re-reading it per piece."""
    def __init__(self, me):
        self.me = me
        self.nv = len(me.vertices)
        self.co = arr(me.vertices, "co", self.nv, np.float32, 3)
        self.ls = arr(me.polygons, "loop_start", len(me.polygons), np.int32)
        self.lt = arr(me.polygons, "loop_total", len(me.polygons), np.int32)
        self.lv = arr(me.loops, "vertex_index", len(me.loops), np.int32)
        self.attrs = {}
        for a in me.attributes:
            if a.name.startswith(".") or a.name == "position" or a.data_type not in FIELD:
                continue
            k = WIDTH.get(a.data_type, 1)
            n = {'POINT': self.nv, 'EDGE': len(me.edges), 'FACE': len(me.polygons), 'CORNER': len(me.loops)}[a.domain]
            vals = np.empty(n * k, DTYPE.get(a.data_type, np.float32))
            a.data.foreach_get(FIELD[a.data_type], vals)
            self.attrs[a.name] = (a.domain, a.data_type, vals.reshape(-1, k) if k > 1 else vals)
        ev = arr(me.edges, "vertices", len(me.edges), np.int32, 2)
        self.ekey = np.sort(ev, axis=1).astype(np.int64)
        self.ekey = self.ekey[:, 0] * self.nv + self.ekey[:, 1]


def piece_mesh(A, faces, name):
    """New mesh holding only `faces` (all generic attributes kept, unused material slots dropped)."""
    faces = np.sort(faces)
    lt = A.lt[faces]
    loops = np.repeat(A.ls[faces] - np.concatenate(([0], np.cumsum(lt)[:-1])), lt) + np.arange(lt.sum())
    verts, lv_new = np.unique(A.lv[loops], return_inverse=True)
    new = bpy.data.meshes.new(name)
    new.vertices.add(len(verts))
    new.vertices.foreach_set("co", A.co[verts].ravel())
    new.loops.add(len(loops))
    new.loops.foreach_set("vertex_index", lv_new.astype(np.int32))
    new.polygons.add(len(faces))
    new.polygons.foreach_set("loop_start", np.concatenate(([0], np.cumsum(lt)[:-1])).astype(np.int32))
    new.update(calc_edges=True)
    nev = arr(new.edges, "vertices", len(new.edges), np.int32, 2)
    nkey = np.sort(verts[nev], axis=1).astype(np.int64)
    nkey = nkey[:, 0] * A.nv + nkey[:, 1]
    order = np.argsort(A.ekey)
    emap = order[np.searchsorted(A.ekey, nkey, sorter=order)]
    used = []
    for aname, (dom, dt, vals) in A.attrs.items():
        sub = {'POINT': verts, 'EDGE': emap, 'FACE': faces, 'CORNER': loops}[dom]
        v = vals[sub]
        if aname == "material_index":
            used = sorted(set(v.tolist()))
            v = np.searchsorted(np.array(used), v).astype(np.int32)
        a = new.attributes.get(aname) or new.attributes.new(aname, dt, dom)
        a.data.foreach_set(FIELD[dt], np.ascontiguousarray(v).ravel())
    for i in (used or [0]):
        if i < len(A.me.materials):
            new.materials.append(A.me.materials[i])
    new.update()
    return new



def main():
    OUT, INFO = sys.argv[sys.argv.index("--") + 1:][:2]
    T0 = time.time()
    sc = bpy.context.scene
    bpy.context.preferences.filepaths.save_version = 0
    objs = [o for o in sc.objects if o.type == 'MESH']
    info = {"shared": {}, "exploded": {}}

    # ---------------------------------------------------------------- 1. share identical meshes
    by_hash = collections.defaultdict(list)
    for o in objs:
        by_hash[geo_hash(o.data)].append(o)
    n_before = len({o.data.name for o in objs})
    for h, os_ in by_hash.items():
        lead = sorted(os_, key=lambda o: o.data.name)[0].data
        for o in os_:
            o["orig_mesh"] = o.data.name
            if o.data is not lead:
                old = o.data
                o.data = lead
                if old.users == 0:
                    bpy.data.meshes.remove(old)
        if len(os_) > 1:
            info["shared"][lead.name] = [o.name for o in os_]
    print(f"EXPLODE shared meshes: {n_before} -> {len(by_hash)} datablocks ({time.time() - T0:.1f}s)", flush=True)

    # ---------------------------------------------------------------- 2. split merged assemblies into pieces
    done = {}
    for o in objs:
        if cfg.part(o.name) not in getattr(cfg, "EXPLODE", ()):
            continue
        me = o.data
        if me.name not in done:
            t = time.time()
            isl = sorted(islands(me), key=lambda f: -len(f))
            A = MeshArrays(me)
            pieces = [piece_mesh(A, f, f"{me.name}~{k:03d}") for k, f in enumerate(isl)]
            assert sum(len(p.polygons) for p in pieces) == len(me.polygons)
            empty = bpy.data.meshes.new(me.name + "~exploded")
            done[me.name] = (pieces, empty)
            print(f"EXPLODE {o.name}: {len(pieces)} pieces ({time.time() - t:.1f}s)", flush=True)
        pieces, empty = done[me.name]
        o["exploded_from"] = me.name
        names = []
        for k, pm in enumerate(pieces):
            po = bpy.data.objects.new(f"{o.name}~{k:03d}", pm)
            for c in o.users_collection:
                c.objects.link(po)
            po.parent = o
            po.matrix_parent_inverse.identity()
            po.matrix_basis.identity()
            po["piece_of"] = o.name
            names.append(po.name)
        o.data = empty
        info["exploded"][o.name] = {"mesh": me.name, "pieces": names}
    for mname in done:
        m = bpy.data.meshes[mname]
        if m.users == 0:
            bpy.data.meshes.remove(m)
    json.dump(info, open(INFO, "w"), indent=0)
    bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=False)
    print(f"EXPLODE done: {len([o for o in sc.objects if o.type == 'MESH'])} mesh objects, {len(bpy.data.meshes)} meshes, {time.time() - T0:.1f}s", flush=True)


if __name__ == "__main__":
    main()
