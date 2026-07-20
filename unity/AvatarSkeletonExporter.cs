using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEngine;

// TrackingCorrector avatar-skeleton exporter.
//
// Copies a Unity humanoid skeleton into TrackingCorrector's skeleton JSON format
// ({ "bones": [{name, parent, offset}] }). This is a pure representation
// transform: it does NOT interpret the skeleton (no per-bone logic, no chains).
//
// Joint names are the HumanBodyBones enum members verbatim; the humanoid mapping
// only selects which transforms are bones. The one representation difference the
// plugin bridges: in Unity a bone's transform sits at the START of the bone, in
// our format a joint sits at the END. So each joint is placed at the bone's end,
// computed generically:
//   - a bone with mapped children -> the average of the children's positions;
//   - a leaf bone -> a child transform named "<name>_end" (Blender FBX leaf-bone
//     convention) if present, else the average of all its child transforms'
//     positions, else a default length along the bone's direction;
//   - the root (Hips) -> its own transform position (no bone ends at the root; it
//     seeds rootPosition on load).
//
// The root is a special case: Unity's single Hips bone has no counterpart in our
// format (the root carries no bone), so it "explodes" into one joint per child at
// each child's own start position - named Waist / LeftHip / RightHip (chosen to
// avoid HumanBodyBones members) - and the root's children reparent onto those so
// their bones pivot at the right place instead of the pelvis centre.
//
// Coordinates: Unity is left-handed, Y-up, +Z-forward, left side at -X;
// TrackingCorrector is right-handed, Y-up, -Z-forward, left side at +X. They
// differ by (x, y, z) -> (-x, y, -z). World positions are read (so any in-scene
// scale is baked into the offsets), expressed relative to the Animator root's
// frame (translation + rotation removed, scale kept), then mapped to our axes.
// Offsets are child-minus-parent; the root's offset is its own position.
public class AvatarSkeletonExporter : EditorWindow
{
    // Fraction of the Animator's humanScale used as the length of a leaf bone
    // when no matching "<name>_end" transform exists in the hierarchy.
    private const float kDefaultLeafLength = 0.1f;

    private const string kEndSuffix = "_end";

    private Animator avatar;

    [MenuItem("Window/TrackingCorrector/Export Avatar Skeleton")]
    public static void OpenWindow()
    {
        var window = GetWindow<AvatarSkeletonExporter>();
        window.titleContent = new GUIContent("Export Avatar Skeleton");
    }

    private void OnGUI()
    {
        avatar = (Animator)EditorGUILayout.ObjectField("Avatar (Animator)", avatar, typeof(Animator), true);

        if (avatar == null)
        {
            EditorGUILayout.HelpBox("Assign a humanoid Animator.", MessageType.Info);
            return;
        }
        if (!avatar.isHuman)
        {
            EditorGUILayout.HelpBox("The Animator's avatar is not humanoid.", MessageType.Error);
            return;
        }

        if (GUILayout.Button("Export..."))
        {
            Export();
        }
    }

    private void Export()
    {
        try
        {
            List<Bone> bones = BuildBones(avatar);
            string json = ToJson(bones);

            string path = EditorUtility.SaveFilePanel(
                "Export Avatar Skeleton", "", "user-avatar-skeleton.json", "json");
            if (string.IsNullOrEmpty(path))
                return;

            File.WriteAllText(path, json);
            Debug.Log($"Exported {bones.Count} bones to {path}");
        }
        catch (Exception e)
        {
            Debug.LogError($"Avatar skeleton export failed: {e}");
            EditorUtility.DisplayDialog("Export failed", e.Message, "OK");
        }
    }

    private struct Bone
    {
        public string name;
        public string parent;   // null for the root
        public Vector3 offset;  // in TrackingCorrector axes, relative to the parent joint
    }

    // A mapped humanoid bone: its enum id, transform, and mapped parent (or null
    // for the root).
    private struct MappedBone
    {
        public HumanBodyBones id;
        public Transform transform;
        public HumanBodyBones? parent;
    }

    // Maps a Unity world position into the Animator-root frame (translation and
    // rotation removed, scale kept) and then into TrackingCorrector axes.
    private static Vector3 ToLocalFrame(Transform root, Vector3 worldPos)
    {
        Vector3 p = Quaternion.Inverse(root.rotation) * (worldPos - root.position);
        return new Vector3(-p.x, p.y, -p.z);
    }

    private static List<Bone> BuildBones(Animator animator)
    {
        Transform root = animator.transform;

        // Collect every humanoid-mapped transform - that is the skeleton.
        var idOf = new Dictionary<Transform, HumanBodyBones>();
        var order = new List<HumanBodyBones>();
        var transformOf = new Dictionary<HumanBodyBones, Transform>();
        foreach (HumanBodyBones id in Enum.GetValues(typeof(HumanBodyBones)))
        {
            if (id == HumanBodyBones.LastBone)
                continue;
            Transform t = animator.GetBoneTransform(id);
            if (t != null && !idOf.ContainsKey(t))
            {
                idOf[t] = id;
                transformOf[id] = t;
                order.Add(id);
            }
        }
        if (order.Count == 0)
            throw new Exception("No humanoid bones found on the Animator.");

        // Nearest mapped ancestor of each bone (walking up real parents, so any
        // unmapped intermediate transforms are skipped).
        HumanBodyBones? MappedParent(Transform t)
        {
            for (Transform p = t.parent; p != null; p = p.parent)
                if (idOf.TryGetValue(p, out HumanBodyBones id))
                    return id;
            return null;
        }

        var childrenOf = new Dictionary<HumanBodyBones, List<Transform>>();
        var mapped = new List<MappedBone>();
        foreach (HumanBodyBones id in order)
        {
            HumanBodyBones? parent = MappedParent(transformOf[id]);
            mapped.Add(new MappedBone { id = id, transform = transformOf[id], parent = parent });
            if (parent.HasValue)
            {
                if (!childrenOf.TryGetValue(parent.Value, out List<Transform> list))
                    childrenOf[parent.Value] = list = new List<Transform>();
                list.Add(transformOf[id]);
            }
        }

        float leafLength = kDefaultLeafLength * animator.humanScale;

        // Joint position = the END of the bone, in our frame.
        var positions = new Dictionary<HumanBodyBones, Vector3>();
        foreach (MappedBone b in mapped)
            positions[b.id] = BoneEnd(root, b, childrenOf, leafLength);

        // The root (Hips) carries no bone in our format, so its Unity bone
        // "explodes" into one joint per child: each root child bone actually
        // starts at its own transform (a real point), not at the pelvis centre.
        // Emit those start joints under the root and reparent the root's children
        // onto them, so their bones pivot at the correct place. Names are the
        // ones our skeleton uses for these joints (not reserved by Unity's
        // HumanBodyBones), keyed by the child bone.
        MappedBone rootBone = mapped.Find(b => !b.parent.HasValue);
        var explodedName = new Dictionary<HumanBodyBones, string>();  // root child -> its start joint name
        var explodedPos = new Dictionary<string, Vector3>();
        var explodedJoints = new List<Bone>();
        foreach (MappedBone b in mapped)
        {
            if (b.parent != rootBone.id || !kRootChildJointNames.TryGetValue(b.id, out string jointName))
                continue;
            explodedName[b.id] = jointName;
            Vector3 startPos = ToLocalFrame(root, b.transform.position);
            explodedPos[jointName] = startPos;
            explodedJoints.Add(new Bone
            {
                name = jointName,
                parent = rootBone.id.ToString(),
                offset = startPos - positions[rootBone.id],
            });
        }

        var bones = new List<Bone>();
        foreach (MappedBone b in mapped)
        {
            Vector3 pos = positions[b.id];
            if (!b.parent.HasValue)
            {
                bones.Add(new Bone { name = b.id.ToString(), parent = null, offset = pos });
                bones.AddRange(explodedJoints);
                continue;
            }

            string parentName;
            Vector3 parentPos;
            if (b.parent == rootBone.id && explodedName.TryGetValue(b.id, out string startJoint))
            {
                parentName = startJoint;
                parentPos = explodedPos[startJoint];
            }
            else
            {
                parentName = b.parent.Value.ToString();
                parentPos = positions[b.parent.Value];
            }

            bones.Add(new Bone
            {
                name = b.id.ToString(),
                parent = parentName,
                offset = pos - parentPos,
            });
        }
        return bones;
    }

    // Root-only special case: our skeleton splits Unity's single Hips bone into a
    // waist and two hips. Keyed by the root's child bone; names are chosen to
    // avoid HumanBodyBones members.
    private static readonly Dictionary<HumanBodyBones, string> kRootChildJointNames =
        new Dictionary<HumanBodyBones, string>
        {
            { HumanBodyBones.Spine, "Waist" },
            { HumanBodyBones.LeftUpperLeg, "LeftHip" },
            { HumanBodyBones.RightUpperLeg, "RightHip" },
        };

    private static Vector3 BoneEnd(Transform root, MappedBone bone,
        Dictionary<HumanBodyBones, List<Transform>> childrenOf, float defaultLength)
    {
        // The root carries no bone: it sits at its own transform.
        if (!bone.parent.HasValue)
            return ToLocalFrame(root, bone.transform.position);

        // A bone with children ends at the average of the children's starts
        // (their transform positions). Averaged in world space, mapped once.
        if (childrenOf.TryGetValue(bone.id, out List<Transform> children))
        {
            Vector3 sum = Vector3.zero;
            foreach (Transform child in children)
                sum += child.position;
            return ToLocalFrame(root, sum / children.Count);
        }

        // Leaf: prefer an explicit "<name>_end" transform, else the average of
        // all child transforms' positions (world-averaged, mapped once), else
        // extend a default length along the bone's own direction (parent -> bone).
        Transform end = FindEndTransform(bone.transform);
        if (end != null)
            return ToLocalFrame(root, end.position);

        if (bone.transform.childCount > 0)
        {
            Vector3 sum = Vector3.zero;
            foreach (Transform child in bone.transform)
                sum += child.position;
            return ToLocalFrame(root, sum / bone.transform.childCount);
        }

        Vector3 bonePos = ToLocalFrame(root, bone.transform.position);
        Vector3 dir = Vector3.zero;
        if (bone.transform.parent != null)
            dir = bonePos - ToLocalFrame(root, bone.transform.parent.position);
        if (dir.sqrMagnitude < 1e-8f)
            dir = Vector3.up;
        return bonePos + dir.normalized * defaultLength;
    }

    private static Transform FindEndTransform(Transform bone)
    {
        string target = bone.name + kEndSuffix;
        foreach (Transform child in bone)
            if (child.name == target)
                return child;
        return null;
    }

    private static string ToJson(List<Bone> bones)
    {
        var ci = CultureInfo.InvariantCulture;
        var sb = new StringBuilder();
        sb.Append("{\n  \"bones\": [\n");
        for (int i = 0; i < bones.Count; ++i)
        {
            Bone b = bones[i];
            sb.Append("    { \"name\": ");
            sb.Append(Quote(b.name));
            sb.Append(", \"parent\": ");
            sb.Append(b.parent != null ? Quote(b.parent) : "null");
            sb.Append(", \"offset\": [");
            sb.Append(b.offset.x.ToString("R", ci));
            sb.Append(", ");
            sb.Append(b.offset.y.ToString("R", ci));
            sb.Append(", ");
            sb.Append(b.offset.z.ToString("R", ci));
            sb.Append("] }");
            sb.Append(i + 1 < bones.Count ? ",\n" : "\n");
        }
        sb.Append("  ]\n}\n");
        return sb.ToString();
    }

    private static string Quote(string s)
    {
        var sb = new StringBuilder();
        sb.Append('"');
        foreach (char c in s)
        {
            switch (c)
            {
                case '"': sb.Append("\\\""); break;
                case '\\': sb.Append("\\\\"); break;
                case '\n': sb.Append("\\n"); break;
                case '\r': sb.Append("\\r"); break;
                case '\t': sb.Append("\\t"); break;
                default: sb.Append(c); break;
            }
        }
        sb.Append('"');
        return sb.ToString();
    }
}
