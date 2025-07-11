using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEditor;
using UnityEditor.Build;
using UnityEngine;

/// <summary>
/// Preferences for Unity Physics package
/// </summary>
public static class Preferences
{
    const string k_DisableIntegrityDefine = "UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS";
    const string k_EnableDebugDisplayAtRuntime = "ENABLE_UNITY_PHYSICS_RUNTIME_DEBUG_DISPLAY";
    const char k_defineSeparator = ';';

    /// <summary>
    /// IntegrityChecksDisabled
    /// </summary>
    public static bool IntegrityChecksDisabled
    {
        get => DefineExists(k_DisableIntegrityDefine);
        set => UpdateDefine(k_DisableIntegrityDefine, value);
    }

    /// <summary>
    /// DebugDisplayRuntimeEnabled
    /// </summary>
    public static bool DebugDisplayRuntimeEnabled
    {
        get => DefineExists(k_EnableDebugDisplayAtRuntime, BuildPipeline.GetBuildTargetGroup(EditorUserBuildSettings.activeBuildTarget));
        set => UpdateDefine(k_EnableDebugDisplayAtRuntime, value);
    }

    [SettingsProvider]
    private static SettingsProvider PhysicsChecksMenuItem()
    {
        var provider = new SettingsProvider("Project/Physics/Unity Physics", SettingsScope.Project)
        {
            label = "Unity Physics",
            keywords = new[] { "Unity Physics", "Physics", "Enable Integrity Checks", "Disable Integrity Checks" },
            guiHandler = (searchContext) =>
            {
                EditorGUIUtility.labelWidth = 180;

                bool oldEnableIntegrityChecks = !IntegrityChecksDisabled;
                bool newEnableIntegrityChecks = EditorGUILayout.Toggle(new GUIContent("Enable Integrity Checks",
                    "Integrity checks should be disabled when measuring performance. Integrity checks should be enabled when checking simulation quality and behaviour."),
                    oldEnableIntegrityChecks);
                if (newEnableIntegrityChecks != oldEnableIntegrityChecks)
                {
                    IntegrityChecksDisabled = !newEnableIntegrityChecks;
                }

                bool oldEnableDebugDisplay = DebugDisplayRuntimeEnabled;
                bool newEnableDebugDisplay = EditorGUILayout.Toggle(new GUIContent("Enable Player Debug Display",
                    "Allows debugging physics directly in the Player build. Enable to inspect behavior in-game. Disable for better performance."),
                    oldEnableDebugDisplay);
                if (newEnableDebugDisplay != oldEnableDebugDisplay)
                {
                    DebugDisplayRuntimeEnabled = newEnableDebugDisplay;
                }
            }
        };

        return provider;
    }

    private static void UpdateDefine(string define, bool add)
    {
        //collect all relevant build targets
        var buildTargetGroups = new List<BuildTargetGroup>();

        var activeBuildTarget = EditorUserBuildSettings.activeBuildTarget;
        var activeBuildTargetGroup = BuildPipeline.GetBuildTargetGroup(activeBuildTarget);

        // Provide the define for activeBuildTargetGroup(e.g. Android, PS4)
        buildTargetGroups.Add(activeBuildTargetGroup);

        // Windows, Mac, Linux - always include these, as they are the only ones where the development happens
        // and could possibly want/not want integrity checks in the editor, as opposed to only the connected device.
        if (activeBuildTargetGroup != BuildTargetGroup.Standalone)
        {
            buildTargetGroups.Add(BuildTargetGroup.Standalone);
        }

        foreach (var buildTargetGroup in buildTargetGroups)
        {
            var fromBuildTargetGroup = NamedBuildTarget.FromBuildTargetGroup(buildTargetGroup);
            var defines = PlayerSettings.GetScriptingDefineSymbols(fromBuildTargetGroup);

            // We add the separator at the end so we can add a new one if needed
            // Unity will automatically remove any unneeded separators at the end
            if (defines.Length > 0 && !defines.EndsWith("" + k_defineSeparator))
                defines += k_defineSeparator;

            var definesSb = new StringBuilder(defines);

            if (add)
            {
                // add at the end if it isn't already defined
                if (!defines.Contains(define))
                {
                    definesSb.Append(define);
                    definesSb.Append(k_defineSeparator);
                }
            }
            else
            {
                // find it and just replace that spot with and empty string
                var replaceToken = define + k_defineSeparator;
                definesSb.Replace(replaceToken, "");
            }

            PlayerSettings.SetScriptingDefineSymbols(fromBuildTargetGroup, definesSb.ToString());
        }
    }

    private static bool DefineExists(string define, BuildTargetGroup buildTargetGroup = BuildTargetGroup.Standalone)
    {
        var fromBuildTargetGroup = NamedBuildTarget.FromBuildTargetGroup(buildTargetGroup);
        var defines = PlayerSettings.GetScriptingDefineSymbols(fromBuildTargetGroup);
        return defines.Contains(define);
    }
}
