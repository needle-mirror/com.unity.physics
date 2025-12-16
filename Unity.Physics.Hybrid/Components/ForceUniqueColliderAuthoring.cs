using System;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// An authoring component to tag the physics colliders used by this entity as unique. It is intended to be used with
    /// Built-In Physics Collider Components. Any collider that is present on a GameObject with this component will be
    /// flagged as unique during the baking process. A unique collider will not share a BlobAssetReference&lt;Collider&gt; with
    /// any other collider.
    /// </summary>
    [Icon(k_IconPath)]
    [AddComponentMenu("Entities/Physics/Force Unique Collider")]
    [HelpURL(HelpURLs.ForceUniqueColliderAuthoring)]
    [DisallowMultipleComponent]
    public class ForceUniqueColliderAuthoring : MonoBehaviour
    {
        const string k_IconPath = "Packages/com.unity.physics/Unity.Physics.Editor/Editor Default Resources/Icons/d_BoxCollider@64.png";

        internal uint ForceUniqueID => m_ForceUniqueID ^ m_PrefabInstanceID; // Note: incorporate prefab instance ID for uniqueness in prefab instances
        [SerializeField, HideInInspector]
        uint m_ForceUniqueID = 0;

        [SerializeField, HideInInspector]
        uint m_PrefabInstanceID = 0;

        const int k_VersionAddedForceUniqueID = 1; // added ForceUniqueID for stable artifact IDs
        const int k_VersionAddedPrefabInstanceID = 2; // added m_PrefabInstanceID for unique ForceUniqueID across prefab instances
        const int k_LatestVersion = k_VersionAddedPrefabInstanceID;

        internal bool NeedsVersionUpgrade => m_NeedsVersionUpgrade;
        bool m_NeedsVersionUpgrade = false;

        [SerializeField, HideInInspector]
        int m_SerializedVersion = k_LatestVersion;

#if UNITY_EDITOR
        static string s_LastWarnedPath;
        static double s_NextWarningTime;
#endif

        void OnEnable()
        {
            // included so tick box appears in Editor
        }

        void OnValidate()
        {
            if (m_ForceUniqueID == 0)
            {
                m_ForceUniqueID = (uint)UnityEngine.Random.Range(1, Int32.MaxValue);
            }

#if UNITY_EDITOR
            // Case: Unique collider in a prefab instance:
            // The force unique IDs of multiple collider instances stemming from the same prefab will all be identical.
            // So, in order to assign their resultant baked colliders a unique "force unique id", thereby ensuring
            // that the resultant baked colliders are indeed unique, we need to incorporate an extra unique identifier
            // representing the prefab instance into the force unique ID (see ForceUniqueID property above).
            // This can be achieved using the targetPrefabId from the GlobalObjectId of the collider's game object, which is
            // guaranteed to be unique for each prefab instance in the scene and stable across reloads and sessions.

            var objectId = GlobalObjectId.GetGlobalObjectIdSlow(gameObject);
            var prefabInstanceID = (uint)objectId.targetPrefabId;
            if (prefabInstanceID != m_PrefabInstanceID)
            {
                // Note: we get here if this component is part of a prefab instance, either newly created (in which
                // case m_PrefabInstanceID is zero initially) or duplicated (in which case the current m_PrefabInstanceID
                // matches that of the original prefab instance and needs to be updated to the id of the new instance
                // in order to be unique).

                // Explicitly mark prefab instance as dirty so that the change gets saved alongside the prefab instance.
                // Otherwise, the m_PrefabInstanceID value in the prefab would be used, which is not unique
                // and would thus not allow creating unique colliders.
                Undo.RecordObject(this, "Acquire Prefab Instance ID");
                PrefabUtility.RecordPrefabInstancePropertyModifications(this);

                m_PrefabInstanceID = prefabInstanceID;
            }
#endif

            if (m_SerializedVersion < k_LatestVersion)
            {
                m_SerializedVersion = k_LatestVersion;

                m_NeedsVersionUpgrade = true;

#if UNITY_EDITOR
                if (PrefabUtility.IsPartOfAnyPrefab(this) || gameObject.scene.IsValid())
                {
                    // Inform user that scene needs to be saved:

                    var scenePath = gameObject.scene.path;

                    if (string.IsNullOrEmpty(scenePath))
                    {
                        var gameObjectPath = UnityEditor.Search.SearchUtils.GetHierarchyPath(gameObject);
                        Debug.LogWarning("A Force Unique Collider component in game object '" + gameObjectPath + "' needs to be upgraded. "
                            + "To apply the upgrade, open and re-save the containing asset, " +
                            "or use the automatic version upgrade tool under 'Tools -> Unity Physics -> Upgrade Force Unique Collider Versions'.", gameObject);
                    }
                    else if (scenePath != s_LastWarnedPath || EditorApplication.timeSinceStartup > s_NextWarningTime)
                    {
                        Debug.LogWarning("A Force Unique Collider component in a scene needs to be upgraded. To apply the upgrade, open and re-save the scene '" + scenePath + "', " +
                            "or use the automatic version upgrade tool under 'Tools -> Unity Physics -> Upgrade Force Unique Collider Versions'");

                        s_LastWarnedPath = scenePath;
                        s_NextWarningTime = EditorApplication.timeSinceStartup + 5f;
                    }
                }
#endif
            }
        }
    }
}
