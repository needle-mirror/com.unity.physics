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
    public class ForceUniqueColliderAuthoring : MonoBehaviour, ISerializationCallbackReceiver
    {
        const string k_IconPath = "Packages/com.unity.physics/Unity.Physics.Editor/Editor Default Resources/Icons/d_BoxCollider@64.png";

        internal uint ForceUniqueID => m_ForceUniqueID;
        [SerializeField, HideInInspector]
        uint m_ForceUniqueID = 0;

        const int k_VersionAddedForceUniqueID = 1; // added ForceUniqueID for stable artifact IDs
        const int k_LatestVersion = k_VersionAddedForceUniqueID;
#if UNITY_EDITOR
        bool m_UpgradeRequired = false;
#endif

        [SerializeField, HideInInspector]
        int m_SerializedVersion = 0;

#if UNITY_EDITOR
        static string s_LastWarnedPath;
        static double s_NextWarningTime;
#endif

        void OnEnable()
        {
            // included so tick box appears in Editor
        }

        void ISerializationCallbackReceiver.OnBeforeSerialize()
        {
#if UNITY_EDITOR
            m_UpgradeRequired = false;
#endif
        }

        void ISerializationCallbackReceiver.OnAfterDeserialize()
        {
            if (m_SerializedVersion < k_LatestVersion)
            {
                m_SerializedVersion = k_LatestVersion;
#if UNITY_EDITOR
                m_UpgradeRequired = true;
#endif
            }
        }

        void OnValidate()
        {
            if (m_ForceUniqueID == 0)
            {
                m_ForceUniqueID = (uint)UnityEngine.Random.Range(1, Int32.MaxValue);
            }

            if (m_SerializedVersion < k_LatestVersion)
            {
                m_SerializedVersion = k_LatestVersion;
            }

#if UNITY_EDITOR
            if (m_UpgradeRequired)
            {
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
            }
#endif
        }
    }
}
