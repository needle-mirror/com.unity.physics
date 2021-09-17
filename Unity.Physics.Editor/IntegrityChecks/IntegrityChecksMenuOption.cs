using System;
using System.Collections.Generic;
using System.Text;
using UnityEditor;

namespace Unity.Physics.Editor
{
    static class IntegrityChecksMenuOption
    {
        const string k_IntegrityChecksMenuItem = "DOTS/Physics/Enable Integrity Checks";
        const string k_DisableIntegrityDefine = "UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS";
        const char k_defineSeparator = ';';

        [MenuItem(k_IntegrityChecksMenuItem, false)]
        static void SwitchIntegrityCheckStatus()
        {
            List<BuildTargetGroup> buildTargetGroups = new List<BuildTargetGroup>();

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

            UpdateDefines(k_DisableIntegrityDefine, buildTargetGroups);
        }

        [MenuItem(k_IntegrityChecksMenuItem, true)]
        static bool SwitchIntegrityCheckStatusValidate()
        {
            Menu.SetChecked(k_IntegrityChecksMenuItem, !DefineExists(k_DisableIntegrityDefine, BuildTargetGroup.Standalone));
            return true;
        }

        public static void UpdateDefines(string _define, List<BuildTargetGroup> _buildTargetGroups)
        {
            for (int i = 0; i < _buildTargetGroups.Count; i++)
            {
                var buildTargetGroup = _buildTargetGroups[i];
                var defines = PlayerSettings.GetScriptingDefineSymbolsForGroup(buildTargetGroup);

                // We add the separator at the end so we can add a new one if needed
                // Unity will automatically remove any unneeded separators at the end
                if (defines.Length > 0 && !defines.EndsWith("" + k_defineSeparator))
                    defines = defines + k_defineSeparator;

                var definesSb = new StringBuilder(defines);

                // add at the end if it isn't already defined
                if (!defines.Contains(_define))
                {
                    definesSb.Append(_define);
                    definesSb.Append(k_defineSeparator);
                }
                else // find it and just replace that spot with and empty string
                {
                    var replaceToken = _define + k_defineSeparator;
                    definesSb.Replace(replaceToken, "");
                }

                PlayerSettings.SetScriptingDefineSymbolsForGroup(buildTargetGroup, definesSb.ToString());
            }
        }

        public static bool DefineExists(string _define, BuildTargetGroup _buildTargetGroup)
        {
            var defines = PlayerSettings.GetScriptingDefineSymbolsForGroup(_buildTargetGroup);
            return defines != null && defines.Length > 0 && defines.IndexOf(_define, 0) >= 0;
        }
    }
}
