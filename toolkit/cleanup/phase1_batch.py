# One Phase 1 batch in its own Blender process: open latest checkpoint -> clean the batch's meshes -> save.
#   blender -b <in.blend> --python phase1_batch.py -- plan.json <batch> <out.blend> <result.json> <render_dir>
import bpy, sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clean_core as cc
PLAN, BI, OUT, RES, RDIR = sys.argv[sys.argv.index("--") + 1:][:5]
BI = int(BI); os.makedirs(RDIR, exist_ok=True)
T0 = time.time()
bpy.context.preferences.filepaths.save_version = 0
batch = json.load(open(PLAN))[BI]
sc = bpy.context.scene
users = {}
for o in sc.objects:
    if o.type == 'MESH':
        users.setdefault(o.data.name, []).append(o)
units = batch["units"]
for u in units:  # identical-geometry groups must still be identical in this checkpoint
    hs = {cc.geo_hash(bpy.data.meshes[m]) for m in [u["lead"]] + u["copies"]}
    assert len(hs) == 1, f"copies diverged for {u['lead']}"

# ---- remember render/viewport state so the checkpoint only differs in geometry
s = sc.display.shading
st = {"engine": sc.render.engine, "rx": sc.render.resolution_x, "ry": sc.render.resolution_y, "fp": sc.render.filepath,
      "aa": sc.display.render_aa, "vt": sc.view_settings.view_transform, "cam": sc.camera,
      "sh": {a: getattr(s, a) for a in ("light", "color_type", "show_cavity", "show_object_outline", "show_specular_highlight")}}
diffuse = {m.name: tuple(m.diffuse_color) for m in bpy.data.materials}
cc.setup_render(512)
top = sorted(units, key=lambda u: -u["tris"])[:5]
for k, u in enumerate(top):
    cc.render_obj(users[u["lead"]][0], f"{RDIR}/b{BI:02d}_{k}_before.png")
t_render0 = time.time() - T0

out = {"batch": BI, "units": []}
tb = ta = 0
for u in units:
    lead = bpy.data.meshes[u["lead"]]
    objs = [o for m in [u["lead"]] + u["copies"] for o in users[m]]
    bb0 = {o.name: cc.world_bbox(o) for o in objs}
    s0 = cc.topo(lead)
    t = time.time()
    tm = cc.clean_mesh(lead)
    for m in u["copies"]:
        cc.copy_geometry(lead, bpy.data.meshes[m])
    el = time.time() - t
    s1 = cc.topo(lead)
    dbb = max(abs(bb0[o.name][a][i] - b[a][i]) for o in objs for b in [cc.world_bbox(o)] for a in range(2) for i in range(3)) * 1000
    n = 1 + len(u["copies"])
    tb += s0["tris"] * n; ta += s1["tris"] * n
    out["units"].append({"lead": u["lead"], "object": users[u["lead"]][0].name, "copies": n, "before": s0, "after": s1,
                         "secs": round(el, 2), **tm, "bbox_dmm": round(dbb, 4), "meshes": [u["lead"]] + u["copies"]})
    print(f"P1 {users[u['lead']][0].name[:50]:50s} x{n:<3d} {s0['tris']:>9,} -> {s1['tris']:>9,}  {el:7.1f}s  dbb {dbb:.4f}mm", flush=True)
t_proc = time.time() - T0 - t_render0
for k, u in enumerate(top):
    cc.render_obj(users[u["lead"]][0], f"{RDIR}/b{BI:02d}_{k}_after.png")
out["render_labels"] = [users[u["lead"]][0].name for u in top]

# ---- restore render/viewport state, drop the render camera
cam = sc.objects.get("p1cam")
if cam:
    cd = cam.data; bpy.data.objects.remove(cam); bpy.data.cameras.remove(cd)
sc.render.engine = st["engine"]; sc.render.resolution_x = st["rx"]; sc.render.resolution_y = st["ry"]; sc.render.filepath = st["fp"]
sc.display.render_aa = st["aa"]; sc.view_settings.view_transform = st["vt"]; sc.camera = st["cam"]
for a, v in st["sh"].items(): setattr(s, a, v)
for m in bpy.data.materials:
    if m.name in diffuse: m.diffuse_color = diffuse[m.name]

t = time.time()
bpy.ops.wm.save_as_mainfile(filepath=OUT, compress=False)
out.update({"tris_before": tb, "tris_after": ta, "t_render": round(t_render0 + (time.time() - T0 - t_render0 - t_proc), 1),
            "t_process": round(t_proc, 1), "t_save": round(time.time() - t, 1), "t_total": round(time.time() - T0, 1),
            "peak_rss_mb": cc.peak_rss_mb()})
json.dump(out, open(RES, "w"))
print(f"P1 BATCH {BI} done: {tb:,} -> {ta:,} tris, process {t_proc:.1f}s, save {out['t_save']}s, total {out['t_total']}s, peak {out['peak_rss_mb']} MB", flush=True)
