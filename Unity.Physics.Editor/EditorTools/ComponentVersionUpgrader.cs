#if UNITY_EDITOR

using System;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

namespace Unity.Physics.Editor
{
    /// <summary>
    /// Upgrades specific components to their latest version by resaving all prefabs or scenes
    /// in the project that contain them.
    /// </summary>
    static class ComponentVersionUpgrader
    {
        [MenuItem("Tools/Unity Physics/Upgrade Force Unique Collider Versions")]
        static void UpgradeAssetsWithForceUniqueCollider()
        {
            UpgradeAssets<Authoring.ForceUniqueColliderAuthoring>("Force Unique Collider");
        }

        internal static void UpgradeAssets<T>(in string componentName) where T : Component
        {
            int prefabCount = 0;
            int upgradedPrefabCount = 0;
            int sceneCount = 0;
            int upgradedSceneCount = 0;

            Debug.Log($"Starting upgrade of assets containing {componentName} components." +
                "This may take a while depending on the number of assets in your project.");

            try
            {
                var cancelled = UpgradePrefabs<T>(componentName, out prefabCount, out upgradedPrefabCount);
                if (!cancelled)
                {
                    UpgradeScenes<T>(componentName, out sceneCount, out upgradedSceneCount);
                }
            }
            finally
            {
                EditorUtility.FocusProjectWindow();

                EditorUtility.DisplayDialog($"Completed {componentName} Version Upgrade",
                    $"Checked {prefabCount} prefab(s) and upgraded {upgradedPrefabCount} prefab(s).\n" +
                    $"Checked {sceneCount} scene(s) and upgraded {upgradedSceneCount} scene(s).",
                    "OK");

                Debug.Log("Asset upgrade complete.");
            }
        }

        /// <summary>
        /// Finds and processes all prefabs and prefab variants in the project.
        /// <returns> False if cancelled. True otherwise. </returns>
        /// </summary>
        static bool UpgradePrefabs<T>(in string componentName, out int prefabCount, out int upgradedPrefabCount) where T : Component
        {
            string[] prefabGuids = AssetDatabase.FindAssets("t:Prefab");
            int processedCount = 0;
            int modifiedCount = 0;

            Debug.Log($"Found {prefabGuids.Length} prefabs to check...");

            var cancelled = false;
            try
            {
                foreach (string guid in prefabGuids)
                {
                    string path = AssetDatabase.GUIDToAssetPath(guid);
                    processedCount++;

                    cancelled = EditorUtility.DisplayCancelableProgressBar(
                        "Upgrading Prefabs",
                        $"Checking: {path}",
                        (float)processedCount / prefabGuids.Length);

                    if (cancelled)
                    {
                        Debug.Log("Upgrade cancelled.");
                        break;
                    }

                    // else:

                    // Load the prefab's contents into a temporary scene to inspect it.
                    GameObject root = PrefabUtility.LoadPrefabContents(path);
                    if (root == null)
                    {
                        Debug.LogWarning($"Could not load prefab at path '{path}'. Skipping.");
                        continue;
                    }

                    try
                    {
                        // Check if any GameObject in the prefab has the target component.
                        if (root.GetComponentInChildren<T>(true) != null)
                        {
                            Debug.Log($"Found {componentName} components in prefab '{path}'. Resaving.");
                            modifiedCount++;

                            // Resave the prefab asset.
                            PrefabUtility.SaveAsPrefabAsset(root, path);
                        }
                    }
                    finally
                    {
                        // Unload the temporary scene and its contents.
                        PrefabUtility.UnloadPrefabContents(root);
                    }
                }
            }
            finally
            {
                // Ensure the progress bar is cleared even if an error occurs.
                EditorUtility.ClearProgressBar();

                Debug.Log($"Finished checking {processedCount} prefab(s) and upgraded {modifiedCount} prefab(s).");
            }

            prefabCount = processedCount;
            upgradedPrefabCount = modifiedCount;

            return cancelled;
        }

        /// <summary>
        /// Finds and processes all scenes in the project.
        /// <returns> False if cancelled. True otherwise. </returns>
        /// </summary>
        static bool UpgradeScenes<T>(in string componentName, out int sceneCount, out int upgradedSceneCount) where T : Component
        {
            // Save the current scene setup to restore it later.
            SceneSetup[] originalSceneSetup = EditorSceneManager.GetSceneManagerSetup();

            string[] sceneGuids = AssetDatabase.FindAssets("t:Scene");
            int processedCount = 0;
            int modifiedCount = 0;

            Debug.Log($"Found {sceneGuids.Length} scenes to check...");

            var cancelled = false;
            try
            {
                foreach (string guid in sceneGuids)
                {
                    string path = AssetDatabase.GUIDToAssetPath(guid);
                    processedCount++;

                    cancelled = EditorUtility.DisplayCancelableProgressBar(
                        "Upgrading Scenes",
                        $"Checking: {path}",
                        (float)processedCount / sceneGuids.Length);

                    if (cancelled)
                    {
                        Debug.Log("Upgrade cancelled.");
                        break;
                    }
                    // else:

                    // Open the scene to inspect its contents.
                    try
                    {
                        var scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
                        bool sceneNeedsSaving = false;
                        foreach (GameObject rootObject in scene.GetRootGameObjects())
                        {
                            if (rootObject.GetComponentInChildren<T>(true) != null)
                            {
                                sceneNeedsSaving = true;
                                break; // Found one, no need to check other objects in this scene.
                            }
                        }

                        if (sceneNeedsSaving)
                        {
                            Debug.Log($"Found {componentName} components in scene '{path}'. Saving.");
                            modifiedCount++;
                            EditorSceneManager.SaveScene(scene);
                        }
                    }
                    catch (Exception)
                    {
                        Debug.LogWarning($"Could not load scene at path '{path}'. Skipping.");
                    }
                }
            }
            finally
            {
                // Ensure the progress bar is cleared even if an error occurs.
                EditorUtility.ClearProgressBar();

                Debug.Log($"Finished checking {processedCount} scene(s) and upgraded {modifiedCount} scene(s).");

                // Restore the original scene setup that was open before the script ran.
                if (originalSceneSetup.Length > 0)
                {
                    EditorSceneManager.RestoreSceneManagerSetup(originalSceneSetup);
                }
            }

            sceneCount = processedCount;
            upgradedSceneCount = modifiedCount;

            return cancelled;
        }
    }
}
#endif
