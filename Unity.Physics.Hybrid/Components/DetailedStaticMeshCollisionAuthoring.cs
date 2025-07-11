using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// An authoring component that enables detailed static mesh collision computation for static meshes,
    /// (i.e. environmental meshes), based on their polygon data.
    /// Any UnityEngine.MeshCollider component attached at same hierarchy level as this component
    /// will be flagged as a detailed static mesh during the baking process.
    /// </summary>
    [Icon(k_IconPath)]
    [AddComponentMenu("Entities/Physics/Detailed Static Mesh Collision")]
    [HelpURL(HelpURLs.DetailedStaticMeshCollisionAuthoring)]
    [DisallowMultipleComponent]
    public class DetailedStaticMeshCollisionAuthoring : MonoBehaviour
    {
        const string k_IconPath = "Packages/com.unity.physics/Unity.Physics.Editor/Editor Default Resources/Icons/d_BoxCollider@64.png";
        private void Start()
        {
        }
    }
}
