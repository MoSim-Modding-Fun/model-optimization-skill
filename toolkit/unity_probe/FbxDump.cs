// Drop-in check: dump every sub-asset of the probe project's FBX (type, name, fileID, submeshes, vertices) to JSON.
// Unity -batchmode -nographics -projectPath <probe> -executeMethod FbxDump.Run
using UnityEditor;
using UnityEngine;
using System.IO;
using System.Text;

public static class FbxDump
{
    public static void Run()
    {
        AssetDatabase.Refresh();
        var sb = new StringBuilder("[\n");
        bool first = true;
        foreach (var path in Directory.GetFiles("Assets", "*.fbx"))
        {
            foreach (var o in AssetDatabase.LoadAllAssetsAtPath(path))
            {
                if (o == null) continue;
                AssetDatabase.TryGetGUIDAndLocalFileIdentifier(o, out string guid, out long fid);
                int sm = -1, vc = -1;
                var m = o as Mesh;
                if (m != null) { sm = m.subMeshCount; vc = m.vertexCount; }
                if (!first) sb.Append(",\n");
                first = false;
                sb.Append("{\"file\":\"" + path + "\",\"type\":\"" + o.GetType().Name + "\",\"name\":\"" +
                          o.name.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\",\"guid\":\"" + guid + "\",\"fileID\":" + fid +
                          ",\"submeshes\":" + sm + ",\"verts\":" + vc + "}");
            }
        }
        sb.Append("\n]\n");
        File.WriteAllText("fbx_dump.json", sb.ToString());
        Debug.Log("FBXDUMP done");
        EditorApplication.Exit(0);
    }
}
