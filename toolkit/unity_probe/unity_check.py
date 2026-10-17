# Drop-in check in Unity terms: does the prefab still resolve every FBX mesh and material it references, with the same
# submesh count, after the new FBX is imported with the game's .meta? (dumps from unity_probe.sh)
#   python3 unity_check.py <orig dump.json> <new dump.json> <prefab file> <fbx guid> [--all]
# Prints every problem with --all (default: the first 50). "mesh missing" is only acceptable for meshes whose every
# user the owner deleted on purpose.
import json, re, sys, collections

A, B = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
pref, guid = open(sys.argv[3]).read(), sys.argv[4]
ma = {x["fileID"]: x for x in A if x["type"] == "Mesh"}
mb = {x["fileID"]: x for x in B if x["type"] == "Mesh"}
matb = {x["fileID"] for x in B if x["type"] == "Material"}
ref = collections.Counter(int(x) for x in re.findall(r"m_Mesh: \{fileID: (-?\d+), guid: " + guid, pref))
mref = set(int(x) for x in re.findall(r"fileID: (-?\d+), guid: " + guid, pref)) - set(ref)
bad = []
for k, n in ref.items():
    if k not in mb:
        bad.append(("mesh missing", ma.get(k, {}).get("name"), n))
    elif mb[k]["submeshes"] != ma[k]["submeshes"]:
        bad.append(("submesh count", mb[k]["name"], ma[k]["submeshes"], mb[k]["submeshes"]))
for k in mref:
    if k not in matb:
        bad.append(("material missing", k))
va = sum(ma[k]["verts"] * n for k, n in ref.items() if k in ma)
vb = sum(mb[k]["verts"] * n for k, n in ref.items() if k in mb)
print(f"prefab mesh refs {len(ref)} ({sum(ref.values())} MeshFilters), material refs {len(mref)}; problems: {len(bad)}")
print(f"in-game vertices (Unity's count, after its weld): {va} -> {vb}")
for b in bad if "--all" in sys.argv else bad[:50]:
    print("  ", b)
sys.exit(1 if bad else 0)
