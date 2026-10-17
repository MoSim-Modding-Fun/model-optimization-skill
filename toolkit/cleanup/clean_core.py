# Phase 1 core (cleanup/run_phase1.py): Merge by Distance -> Limited Dissolve per mesh datablock, with topology stats and renders.
import bpy, bmesh, math, time, hashlib, os, numpy as np
from mathutils import Vector

MERGE_DIST = 1e-5          # 0.01 mm in metres (scene unit scale 1.0, METERS)
DISSOLVE_DEG = 5.0         # 64-sided cylinders are 5.6 deg per face
SHARP_DEG = 30.0           # edges marked sharp after custom normals are cleared (data-level Auto Smooth)


def _arr(coll, attr, n, dt, k=1):
    a = np.empty(n * k, dt)
    coll.foreach_get(attr, a)
    return a.reshape(-1, k) if k > 1 else a


def tris(me):
    n = len(me.polygons)
    return int((_arr(me.polygons, "loop_total", n, np.int32) - 2).sum()) if n else 0


def topo(me):
    """boundary / non-manifold (>2 faces) / wire edges, loose verts, from the Mesh (fast, numpy)."""
    ne, nv = len(me.edges), len(me.vertices)
    le = _arr(me.loops, "edge_index", len(me.loops), np.int32)
    fc = np.bincount(le, minlength=ne)
    ev = _arr(me.edges, "vertices", ne, np.int32, 2)
    used = np.zeros(nv, bool); used[ev.ravel()] = True
    return {"verts": nv, "tris": tris(me), "boundary": int((fc == 1).sum()), "nonmanifold": int((fc > 2).sum()),
            "wire": int((fc == 0).sum()), "loose_verts": int((~used).sum())}


def geo_hash(me):
    nv, nl, npo = len(me.vertices), len(me.loops), len(me.polygons)
    h = hashlib.sha1()
    for x in (np.round(_arr(me.vertices, "co", nv, np.float32, 3).astype(np.float64), 6),
              _arr(me.loops, "vertex_index", nl, np.int32), _arr(me.polygons, "loop_total", npo, np.int32),
              _arr(me.polygons, "material_index", npo, np.int32)):
        h.update(np.ascontiguousarray(x).tobytes())
    h.update("|".join(m.name if m else "-" for m in me.materials).encode())
    return h.hexdigest()[:16]


def _drop_custom_normals(me):
    if "custom_normal" in me.attributes:
        me.attributes.remove(me.attributes["custom_normal"])


def _finish_shading(me):
    me.shade_smooth()
    me.set_sharp_from_angle(angle=math.radians(SHARP_DEG))
    me.update()


def clean_mesh(me, delimit=frozenset({'MATERIAL'})):
    _drop_custom_normals(me)              # garbage after welding; also saves bmesh memory
    bm = bmesh.new()
    bm.from_mesh(me)
    t = time.time()
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=MERGE_DIST)
    t1 = time.time()
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(DISSOLVE_DEG), use_dissolve_boundaries=False,
                             verts=bm.verts, edges=bm.edges, delimit=set(delimit))
    t2 = time.time()
    bm.to_mesh(me)
    bm.free()
    _drop_custom_normals(me)
    _finish_shading(me)
    return {"merge_s": round(t1 - t, 2), "dissolve_s": round(t2 - t1, 2)}


def copy_geometry(src, dst):
    """Give dst the exact geometry of src (identical-geometry duplicate), keeping dst's datablock and name."""
    bm = bmesh.new()
    bm.from_mesh(src)
    bm.to_mesh(dst)
    bm.free()
    _drop_custom_normals(dst)
    _finish_shading(dst)


def world_bbox(o):
    mw = o.matrix_world
    bb = [mw @ Vector(c) for c in o.bound_box]
    return [[min(v[i] for v in bb) for i in range(3)], [max(v[i] for v in bb) for i in range(3)]]


def peak_rss_mb():
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import sysmem
    return int(sysmem.peak_rss_mb())


# ---------------------------------------------------------------- renders (Workbench, cavity off)
_cam = None


def setup_render(res=512):
    global _cam
    sc = bpy.context.scene
    sc.render.engine = 'BLENDER_WORKBENCH'
    sc.render.resolution_x = sc.render.resolution_y = res
    sh = sc.display.shading
    sh.light = 'STUDIO'; sh.color_type = 'MATERIAL'; sh.show_cavity = False; sh.show_object_outline = False
    sh.show_specular_highlight = True
    sc.display.render_aa = '8'; sc.view_settings.view_transform = 'Standard'
    for m in bpy.data.materials:
        if m.use_nodes and m.node_tree:
            for n in m.node_tree.nodes:
                if n.bl_idname == 'ShaderNodeBsdfPrincipled' and not n.inputs['Base Color'].is_linked:
                    m.diffuse_color = (*n.inputs['Base Color'].default_value[:3], 1)
    cam = bpy.data.cameras.new("p1cam"); co = bpy.data.objects.new("p1cam", cam)
    sc.collection.objects.link(co); sc.camera = co; cam.type = 'ORTHO'
    _cam = co


def render_obj(o, path, d=(1.0, -1.3, 0.9)):
    sc = bpy.context.scene
    keep = {}
    for x in sc.objects:
        if x.type == 'MESH':
            keep[x.name] = x.hide_render
            x.hide_render = x is not o
    bb = [Vector(c) for c in o.bound_box]
    ctr = o.matrix_world @ (sum(bb, Vector()) / 8)
    R = max((o.matrix_world @ b - ctr).length for b in bb)
    d = Vector(d).normalized()
    _cam.data.ortho_scale = 2.1 * R; _cam.location = ctr + d * 4 * R
    _cam.data.clip_start = 0.0002; _cam.data.clip_end = 10 * R
    _cam.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
    for n, h in keep.items():
        sc.objects[n].hide_render = h
