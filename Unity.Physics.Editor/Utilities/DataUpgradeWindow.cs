using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEditor.PackageManager;
using UnityEditor.SceneManagement;
using UnityEditor.VersionControl;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Unity.Physics.Authoring
{
    class DataUpgradeWindow : EditorWindow
    {
        static class Content
        {
            public static readonly string Title = L10n.Tr("Upgrade Physics Data");
            public static readonly string Description = L10n.Tr(
                "Use this utility to upgrade authoring data when prompted to do so after updating Unity Physics. " +
                "It will be removed in a future version of Unity Physics before the package exits preview."
            );
            public static readonly string DoItButton = L10n.Tr("Upgrade");

            public static readonly string DialogTitle = L10n.Tr("Are you sure?");
            public static readonly string DialogMessage = L10n.Tr(
                "This will scan all assets in your project and can take a long time to complete. " +
                "Make sure you have saved a copy of your project before you proceed."
            );
            public static readonly string DialogOK = L10n.Tr("Upgrade");
            public static readonly string DialogCancel = L10n.Tr("Cancel");

            public static readonly string FindAsset = L10n.Tr("Find");

            public static readonly GUIContent Failures =
                EditorGUIUtility.TrTextContent("Failures", "View assets that failed to upgrade.");
            public static readonly string NoFailures = L10n.Tr("No failed upgrades.");
            public static readonly string FailuresFormat = L10n.Tr("{0} assets failed to upgrade.");

            public static readonly GUIContent Successes =
                EditorGUIUtility.TrTextContent("Upgraded", "View assets that successfully upgraded.");
            public static readonly string NoUpgradedAssets = L10n.Tr("No assets have been upgraded. Select data layouts to upgrade and press Upgrade");
            public static readonly string UpgradedAssetsFormat = L10n.Tr("{0} assets have been upgraded successfully.");
        }

        static class Styles
        {
            public const float FindAssetButtonWidth = 100f;
        }

        [SerializeField]
        [Tooltip("Enable this option to update physics material templates and shape prefabs authored in versions before 0.4.0-preview.")]
        bool m_UpgradeMaterials = true;

        Vector2 m_ScrollPosition;

        [MenuItem("Window/DOTS/Physics/Upgrade Data")]
        static void Create() => GetWindow<DataUpgradeWindow>();

        [Serializable]
        class AssetGroupViewData
        {
            public bool Expanded;

            public IReadOnlyDictionary<string, string> AssetPathsAndStatuses => m_AssetPathsAndStatus;
            Dictionary<string, string> m_AssetPathsAndStatus = new Dictionary<string, string>();

            public void AddEntry(string path, string message) => m_AssetPathsAndStatus[path] = message;

            public void Clear() => m_AssetPathsAndStatus.Clear();

            public int Count => m_AssetPathsAndStatus.Count;
        }

        SerializedObject m_SerializedObject;

        [SerializeField] AssetGroupViewData m_Failures = new AssetGroupViewData { Expanded = true };
        [SerializeField] AssetGroupViewData m_Successes = new AssetGroupViewData { Expanded = false };

        void OnEnable()
        {
            titleContent.text = Content.Title;

            m_SerializedObject = new SerializedObject(this);
        }

        [Serializable]
        struct ReportFailure
        {
            public string AssetPath;
            public string Message;
        }

        [Serializable]
        struct ReportOutput
        {
            public string[] Successes;
            public ReportFailure[] Failures;
        }

        void OnGUI()
        {
            EditorGUILayout.HelpBox(Content.Description, MessageType.Warning);
            EditorGUILayout.PropertyField(m_SerializedObject.FindProperty(nameof(m_UpgradeMaterials)));

            if (
                GUILayout.Button(Content.DoItButton)
                && EditorUtility.DisplayDialog(Content.DialogTitle, Content.DialogMessage, Content.DialogOK, Content.DialogCancel))
            {
                m_Failures.Clear();
                m_Successes.Clear();

                try
                {
                    AssetDatabase.StartAssetEditing();

                    if (m_UpgradeMaterials)
                    {
                        PhysicsMaterialProperties.s_SuppressUpgradeWarnings = true;
                        UpgradeMaterials();
                    }
                }
                finally
                {
                    AssetDatabase.StopAssetEditing();
                    PhysicsMaterialProperties.s_SuppressUpgradeWarnings = false;
                }
                
                var upgradeReport = new ReportOutput
                {
                    Successes = m_Successes.AssetPathsAndStatuses.Keys.ToArray(),
                    Failures = m_Failures.AssetPathsAndStatuses.Select(kv => new ReportFailure { AssetPath = kv.Key, Message = kv.Value }).ToArray()
                };
                using (var sw = new StreamWriter($"Assets/{DateTime.Now:yyyy-dd-M--HH-mm-ss}-PhysicsMaterialUpgradeReport.json"))
                    sw.Write(EditorJsonUtility.ToJson(upgradeReport));

                AssetDatabase.SaveAssets();

                GUIUtility.ExitGUI();
            }

            m_ScrollPosition = GUILayout.BeginScrollView(m_ScrollPosition);
            DisplayAssetGroup(
                m_Failures,
                Content.Failures,
                m_Failures.Count == 0 ? Content.NoFailures : string.Format(Content.FailuresFormat, m_Failures.Count),
                m_Failures.Count == 0 ? MessageType.None : MessageType.Warning
            );
            DisplayAssetGroup(
                m_Successes,
                Content.Successes,
                m_Successes.Count == 0 ? Content.NoUpgradedAssets : string.Format(Content.UpgradedAssetsFormat, m_Successes.Count),
                MessageType.None
            );
            GUILayout.EndScrollView();

            GUILayout.FlexibleSpace();
        }

        static readonly GUIContent s_TempContent = new GUIContent();

        void DisplayAssetGroup(AssetGroupViewData group, GUIContent label, string statusMessage, MessageType status)
        {
            if (group.Expanded = EditorGUILayout.BeginFoldoutHeaderGroup(group.Expanded, label))
            {
                ++EditorGUI.indentLevel;
                if (!string.IsNullOrEmpty(statusMessage))
                {
                    EditorGUILayout.HelpBox(statusMessage, status);
                }
                foreach (var path in group.AssetPathsAndStatuses)
                {
                    var rowRect = EditorGUI.IndentedRect(
                        EditorGUILayout.GetControlRect(false, EditorGUIUtility.singleLineHeight)
                    );

                    s_TempContent.text = path.Key;
                    s_TempContent.tooltip = path.Value;
                    GUI.Label(new Rect(rowRect) { xMax = rowRect.xMax - Styles.FindAssetButtonWidth }, s_TempContent);

                    if (
                        GUI.Button(new Rect(rowRect) { xMin = rowRect.xMax - Styles.FindAssetButtonWidth }, Content.FindAsset)
                    )
                        EditorGUIUtility.PingObject(AssetDatabase.LoadMainAssetAtPath(path.Key));
                }
                --EditorGUI.indentLevel;
            }
            EditorGUILayout.EndFoldoutHeaderGroup();
        }

        static bool IsEditable(AssetList assetList, AssetGroupViewData failures)
        {
            if (!Provider.enabled || !Provider.isActive || !Provider.CheckoutIsValid(assetList, CheckoutMode.Both))
                return true;

            var task = Provider.Checkout(assetList, CheckoutMode.Both);
            task.Wait();
            if (!task.success)
            {
                foreach (var asset in assetList)
                    failures.AddEntry(asset.assetPath, "Checkout from version control failed.");
                return false;
            }

            return true;
        }

        static bool IsEditable(Asset asset, AssetGroupViewData failures)
        {
            if (asset.readOnly || asset.locked)
            {
                failures.AddEntry(asset.path, $"readOnly = {asset.readOnly}, locked = {asset.locked}");
                return false;
            }
            // do not append to failures if it is a readonly package
            var packageInfo = UnityEditor.PackageManager.PackageInfo.FindForAssetPath(asset.assetPath);
            return packageInfo == null
                   || packageInfo.source == PackageSource.Embedded || packageInfo.source == PackageSource.Local;
        }

        void UpgradeMaterials()
        {
            // upgrade material template assets
            var assetList = new AssetList();
            assetList.AddRange(
                AssetDatabase.FindAssets($"t:{nameof(PhysicsMaterialTemplate)}")
                    .Select(guid => AssetDatabase.GUIDToAssetPath(guid))
                    .Select(path => new Asset(path))
            );
            if (assetList.Count > 0)
            {
                if (IsEditable(assetList, m_Failures))
                {
                    foreach (var asset in assetList)
                    {
                        if (!IsEditable(asset, m_Failures))
                            continue;

                        // material templates are upgraded in OnEnable(), so it should be sufficient to merely load them
                        var materialTemplate = AssetDatabase.LoadAssetAtPath<PhysicsMaterialTemplate>(asset.path);
                        EditorUtility.SetDirty(materialTemplate);
                        m_Successes.AddEntry(asset.path, string.Empty);
                    }
                }
            }

            // upgrade prefabs
            assetList.Clear();
            foreach (var guid in AssetDatabase.FindAssets("t:Prefab"))
            {
                var prefabAssetPath = AssetDatabase.GUIDToAssetPath(guid);

                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabAssetPath);

                var shapeComponents = prefab.GetComponentsInChildren<PhysicsShapeAuthoring>(true);
                if (shapeComponents.Length == 0)
                    continue;

                var asset = new Asset(prefabAssetPath);
                if (!IsEditable(asset, m_Failures))
                    continue;

                assetList.Add(asset);
            }

            if (assetList.Count > 0)
            {
                if (IsEditable(assetList, m_Failures))
                {
                    var upgradedAssets = new HashSet<Asset>();
                    foreach (var asset in assetList)
                        UpgradePrefabAsset(asset, assetList, upgradedAssets, m_Successes, m_Failures);
                }
            }

            // update scene objects
            EditorSceneManager.SetActiveScene(EditorSceneManager.NewScene(NewSceneSetup.EmptyScene));
            foreach (var guid in AssetDatabase.FindAssets("t:Scene"))
            {
                var scenePath = AssetDatabase.GUIDToAssetPath(guid);

                var asset = new Asset(scenePath);
                if (!IsEditable(asset, m_Failures))
                    continue;

                Scene scene;
                try { scene = EditorSceneManager.OpenScene(scenePath); }
                catch (Exception e)
                {
                    m_Failures.AddEntry(asset.path, $"{e.Message}\n\n{e.StackTrace}");
                    continue;
                }
                EditorSceneManager.SetActiveScene(scene);

                var succcess = true;
                if (IsEditable(new AssetList { new Asset(scenePath) }, m_Failures))
                {
                    var upgradedAny = false;
                    foreach (
                        var sceneObject in scene.GetRootGameObjects()
                            .Where(go => go.GetComponentsInChildren<PhysicsShapeAuthoring>(true).Any())
                    )
                    {
                        try
                        {
                            UpgradePrefabInstance(sceneObject);
                            upgradedAny = true;
                        }
                        catch (Exception e)
                        {
                            succcess = false;
                            m_Failures.AddEntry(
                                $"{scenePath}::{AnimationUtility.CalculateTransformPath(sceneObject.transform, sceneObject.transform.root)}",
                                $"{e.Message}\n\n{e.StackTrace}"
                            );
                        }
                    }

                    if (upgradedAny)
                        EditorSceneManager.SaveScene(scene);
                }

                if (succcess)
                    m_Successes.AddEntry(scenePath, string.Empty);

                EditorSceneManager.SetActiveScene(EditorSceneManager.NewScene(NewSceneSetup.EmptyScene));
                EditorSceneManager.CloseScene(scene, true);
            }
        }

        static void UpgradePrefabAsset(Asset asset,
            AssetList prefabsWithShapes,
            ICollection<Asset> upgradedAssets,
            AssetGroupViewData successes,
            AssetGroupViewData failures
        )
        {
            // exit if we have already upgraded the asset (e.g. it is a parent and we already upgraded it via a variant)
            if (upgradedAssets.Contains(asset))
                return;

            upgradedAssets.Add(asset);

            // exit if it is a prefab parent that doesn't actually have any shapes
            if (!prefabsWithShapes.Contains(asset))
                return;

            GameObject prefab = null;
            try { prefab = PrefabUtility.LoadPrefabContents(asset.path); }
            catch (Exception e)
            {
                failures.AddEntry(asset.path, $"{e.Message}\n\n{e.StackTrace}");
                return;
            }

            // if the prefab is a variant, ensure the prefab it inherits is upgraded first
            if (PrefabUtility.GetPrefabAssetType(prefab) == PrefabAssetType.Variant)
            {
                var sourcePrefab = PrefabUtility.GetCorrespondingObjectFromSource(prefab);
                var sourceAsset = new Asset(AssetDatabase.GetAssetPath(sourcePrefab));
                UpgradePrefabAsset(sourceAsset, prefabsWithShapes, upgradedAssets, successes, failures);
            }

            // next upgrade any nested prefabs
            foreach (
                var rootAsset in prefab.GetComponentsInChildren<Transform>(true)
                    .Select(PrefabUtility.GetNearestPrefabInstanceRoot)
                    .Where(r => r != null && r != prefab) // skip if the child is not a nested prefab
                    .Select(r => new Asset(AssetDatabase.GetAssetPath(r)))
                    .Where(r => prefabsWithShapes.Any(a => a.assetPath == r.assetPath)) // skip if the nested prefab isn't one with a shape
            )
                UpgradePrefabAsset(rootAsset, prefabsWithShapes, upgradedAssets, successes, failures);

            bool success = true;
            try
            {
                UpgradePrefabInstance(prefab);
                PrefabUtility.SaveAsPrefabAsset(prefab, asset.path);
            }
            catch (Exception e)
            {
                failures.AddEntry(asset.path, $"{e.Message}\n\n{e.StackTrace}");
            }
            finally
            {
                PrefabUtility.UnloadPrefabContents(prefab);
            }
            if (success)
                successes.AddEntry(asset.path, string.Empty);
        }

        static readonly Regex k_MatchMaterial = new Regex(@"m_Material\.");

        struct Sub
        {
            public string NewPath;
            public Func<int, int> RemapValue;

            public static Sub Default(string newPath) => new Sub { NewPath = newPath, RemapValue = null };
            public static Sub IsTrigger() => new Sub { NewPath = "m_Material.m_CollisionResponse.Value", RemapValue = v => (int)(v == 0 ? CollisionResponsePolicy.Collide : CollisionResponsePolicy.RaiseTriggerEvents) };
            public static Sub RaiseCollisionEvents() => new Sub { NewPath = "m_Material.m_CollisionResponse.Value", RemapValue = v => (int)(v == 0 ? CollisionResponsePolicy.Collide : CollisionResponsePolicy.CollideRaiseCollisionEvents) };
        }

        // substitutions to be applied in order of precedence
        static List<KeyValuePair<Regex, Sub>> k_Substitutions = new List<KeyValuePair<Regex, Sub>>
        {
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_BelongsTo\.m_Value\.Array\.data\[(\d+)\]"),    Sub.Default("m_Material.m_BelongsToCategories.m_Value.Category{0:00}")),
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_BelongsTo\.m_Override"),                       Sub.Default("m_Material.m_BelongsToCategories.m_Override")),
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_CollidesWith\.m_Value\.Array\.data\[(\d+)\]"), Sub.Default("m_Material.m_CollidesWithCategories.m_Value.Category{0:00}")),
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_CollidesWith\.m_Override"),                    Sub.Default("m_Material.m_CollidesWithCategories.m_Override")),
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_CustomTags\.m_Value\.Array\.data\[(\d+)\]"),   Sub.Default("m_Material.m_CustomMaterialTags.m_Value.Tag{0:00}")),
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_CustomTags\.m_Override"),                      Sub.Default("m_Material.m_CustomMaterialTags.m_Override")),

            // trigger has a higher priority than raise collision events when mapping to collision response
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_IsTrigger\.Value"),               Sub.IsTrigger()),
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_IsTrigger\.Override"),            Sub.Default("m_Material.m_CollisionResponse.Override")),
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_RaiseCollisionEvents\.Value"),    Sub.RaiseCollisionEvents()),
            new KeyValuePair<Regex, Sub>(new Regex(@"m_Material\.m_RaiseCollisionEvents\.Override"), Sub.Default("m_Material.m_CollisionResponse.Override"))
        };

        static void UpgradePrefabInstance(GameObject prefab)
        {
            var modifications = (PrefabUtility.GetPropertyModifications(prefab)??Array.Empty<PropertyModification>()).ToList();
            foreach (
                var modification in modifications
                .Where(m => m.target is PhysicsShapeAuthoring && k_MatchMaterial.IsMatch(m.propertyPath))
                .ToArray()
            )
            {
                foreach (var substitution in k_Substitutions)
                {
                    var match = substitution.Key.Match(modification.propertyPath);
                    if (!match.Success)
                        continue;

                    var value = int.Parse(modification.value);
                    var newPath = string.Format(
                        substitution.Value.NewPath,
                        int.TryParse(match.Groups[1].Value, out var propertyIndex) ? propertyIndex : default
                    );
                    modifications.Remove(modification);
                    modifications.Add(new PropertyModification
                    {
                        objectReference = modification.objectReference,
                        propertyPath = newPath,
                        target = modification.target,
                        value = (substitution.Value.RemapValue?.Invoke(value) ?? value).ToString()
                    });
                    break;
                }
            }

            PrefabUtility.SetPropertyModifications(prefab, modifications.ToArray());
        }
    }
}
