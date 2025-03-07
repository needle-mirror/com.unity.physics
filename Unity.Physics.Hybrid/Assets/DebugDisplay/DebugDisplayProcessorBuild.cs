#if UNITY_EDITOR && ENABLE_UNITY_PHYSICS_RUNTIME_DEBUG_DISPLAY
using UnityEditor;
using UnityEngine;
using UnityEditor.Build.Reporting;
using UnityEditor.Build;
using UnityEditor.Callbacks;
using System.IO;
using System.Linq;
using System;

namespace Unity.DebugDisplay
{
    class DebugDisplayProcessorBuild : IPreprocessBuildWithReport
    {
        public int callbackOrder { get { return 0; } }
        internal static string ResourcesPath => Path.Combine(Managed.debugDirName, "Resources");

        public void OnPreprocessBuild(BuildReport report)
        {
            try
            {
                if (!Directory.Exists(ResourcesPath))
                {
                    Directory.CreateDirectory(ResourcesPath);
                }

                // Get all .mat and .shader files from debugDirName
                string[] materialFiles = Directory.GetFiles(Managed.debugDirName, "*.mat", SearchOption.TopDirectoryOnly);
                string[] shaderFiles = Directory.GetFiles(Managed.debugDirName, "*.shader", SearchOption.TopDirectoryOnly);

                foreach (string file in materialFiles.Concat(shaderFiles))
                {
                    string destinationFile = Path.Combine(ResourcesPath, Path.GetFileName(file));
                    File.Copy(file, destinationFile, true);
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"<color=red>Build Preprocess Failed</color>: {ex.Message}");
            }
        }

        [PostProcessBuild(1)]
        private static void OnPostprocessBuild(BuildTarget target, string pathToBuiltProject)
        {
            var metaFile = Path.Combine(Managed.debugDirName, "Resources.meta");
            if (File.Exists(metaFile))
            {
                File.Delete(metaFile);
            }

            if (Directory.Exists(ResourcesPath))
            {
                Directory.Delete(ResourcesPath, true);
            }
        }
    }
}
#endif
