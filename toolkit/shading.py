# Owner's final shading pass: Limited Dissolve (<=5 deg) as the last geometry step, then
# Smooth by Angle (30 deg) -> WeightedNormal (Face Area, weight 50, Keep Sharp) as live modifiers.
import bpy, bmesh, math, os

ESSENTIALS = "geometry_nodes_essentials.blend"
SBA = "Smooth by Angle"
WN = "WeightedNormal"
DISSOLVE_DEG = 5.0  # 64-sided cylinders have 5.6 deg between faces; stay below so they keep their sides


def sba_group():
    ng = bpy.data.node_groups.get(SBA)
    if ng is None:
        path = os.path.join(bpy.utils.system_resource('DATAFILES'), "assets", "nodes", ESSENTIALS)
        with bpy.data.libraries.load(path, link=False) as (src, dst):
            dst.node_groups = [SBA]
        ng = dst.node_groups[0]
    return ng


def set_input(mod, identifier, value):
    """Geometry Nodes modifier input; Blender 5.x moved these from ID properties to mod.properties.inputs."""
    props = getattr(mod, "properties", None)
    if props is not None and hasattr(props.inputs, identifier):
        getattr(props.inputs, identifier).value = value
    else:
        mod[identifier] = value


def has_shading(o):
    return any(m.type == 'WEIGHTED_NORMAL' for m in o.modifiers)


def add_shading(o, angle_deg=30.0):
    """Smooth by Angle -> WeightedNormal at the end of the stack (no-op if already there)."""
    if has_shading(o):
        return False
    # faces smooth first, as Blender's "Shade Auto Smooth" does: with Ignore Sharpness off, Smooth by Angle keeps
    # every edge of a flat-shaded face sharp, and rebuilt / Collapse-decimated meshes come out flat-shaded, so the stack
    # did nothing (a half-cylinder plate rebuild: 218 of 218 edges sharp). Explicitly marked sharp edges stay sharp.
    if o.type == 'MESH' and o.data.library is None and any(not p.use_smooth for p in o.data.polygons):
        import numpy as np
        o.data.polygons.foreach_set("use_smooth", np.ones(len(o.data.polygons), bool))
        o.data.update()
    s = o.modifiers.new(SBA, 'NODES')
    s.node_group = sba_group()
    for item in s.node_group.interface.items_tree:
        if getattr(item, "in_out", None) == 'INPUT' and item.name == "Angle":
            set_input(s, item.identifier, math.radians(angle_deg))
    w = o.modifiers.new(WN, 'WEIGHTED_NORMAL')
    w.mode = 'FACE_AREA'
    w.weight = 50
    w.keep_sharp = True
    return True


def remove_shading(o):
    for m in list(o.modifiers):
        if m.type == 'WEIGHTED_NORMAL' or (m.type == 'NODES' and m.node_group and m.node_group.name.startswith(SBA)):
            o.modifiers.remove(m)


def add_planar_dissolve(o, angle_deg=DISSOLVE_DEG):
    """Limited Dissolve as a live Decimate (Planar) modifier, for objects whose geometry is still a live
    Collapse; inserted right after the last Decimate so shading modifiers stay at the end."""
    if any(m.type == 'DECIMATE' and m.decimate_type == 'DISSOLVE' for m in o.modifiers):
        return
    m = o.modifiers.new("Limited Dissolve", 'DECIMATE')
    m.decimate_type = 'DISSOLVE'
    m.angle_limit = math.radians(angle_deg)
    m.delimit = {'NORMAL', 'MATERIAL'}
    dec = [i for i, x in enumerate(o.modifiers) if x.type == 'DECIMATE' and x != m]
    target = (max(dec) if dec else -1) + 1
    if len(o.modifiers) - 1 != target:
        o.modifiers.move(len(o.modifiers) - 1, target)
    # n-gons >4 sides from the planar dissolve: triangulate them here (same triangle count) so neither
    # Blender's display nor Unity's importer has to guess at concave n-gons
    t = o.modifiers.new("Triangulate n-gons", 'TRIANGULATE')
    t.min_vertices = 5
    t.ngon_method = 'BEAUTY'
    o.modifiers.move(len(o.modifiers) - 1, target + 1)


def _concave(f):
    n = f.normal
    vs = f.verts
    k = len(vs)
    for i in range(k):
        a, b, c = vs[i - 1].co, vs[i].co, vs[(i + 1) % k].co
        if (b - a).cross(c - b).dot(n) < -1e-12:
            return True
    return False


def limited_dissolve(me, angle_deg=DISSOLVE_DEG, weld=None):
    """Mesh-data Limited Dissolve (keeps material borders and flipped-normal borders). Returns tris after.
    Concave n-gons it creates are triangulated again: a 105-vertex n-gon wrapped round a wheel's windows was
    tessellated across the windows by Blender's display (and Unity triangulates n-gons itself on import). When
    the dissolve saves under 1% of the triangles, the original triangulation is kept.
    weld: the owner's Merge by Distance (cfg.MERGE_DIST, never above ~0.05 mm) as its own pass first, kept even
    when the dissolve itself is skipped."""
    if weld:
        bm = bmesh.new()
        bm.from_mesh(me)
        n0, f0 = len(bm.verts), len(bm.faces)
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=weld)
        bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=weld)
        if len(bm.verts) != n0 or len(bm.faces) != f0:
            bm.to_mesh(me)
            me.update()
        bm.free()
    before = sum(len(p.vertices) - 2 for p in me.polygons)
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-7)
    bmesh.ops.dissolve_limit(bm, angle_limit=math.radians(angle_deg), use_dissolve_boundaries=False,
                             verts=bm.verts, edges=bm.edges, delimit={'NORMAL', 'MATERIAL'})
    after = sum(len(f.verts) - 2 for f in bm.faces)
    if after > 0.99 * before:
        bm.free()
        if "custom_normal" in me.attributes:  # the CAD's own normals are often broken; shading is redone anyway
            me.attributes.remove(me.attributes["custom_normal"])
            me.update()
        return before
    concave = [f for f in bm.faces if len(f.verts) > 4 and _concave(f)]
    if concave:
        bmesh.ops.triangulate(bm, faces=concave, quad_method='BEAUTY', ngon_method='BEAUTY')
    bm.to_mesh(me)
    bm.free()
    if "custom_normal" in me.attributes:
        me.attributes.remove(me.attributes["custom_normal"])
    me.update()
    return sum(len(p.vertices) - 2 for p in me.polygons)
