using System;
using UnityEditor;

#if UNITY_EDITOR

namespace Unity.Physics.Editor
{
    static class CustomComponentVersionUpgrader
    {
        [MenuItem("Tools/Unity Physics/Upgrade Physics Shape Versions")]
        static void UpgradeAssetsWithPhysicsShape()
        {
            var componentNeedsUpgradeFunction = new Func<Authoring.PhysicsShapeAuthoring, bool>(
                component => component.NeedsVersionUpgrade);
            ComponentVersionUpgrader.UpgradeAssets("Physics Shape", componentNeedsUpgradeFunction);
        }
    }
}

#endif
