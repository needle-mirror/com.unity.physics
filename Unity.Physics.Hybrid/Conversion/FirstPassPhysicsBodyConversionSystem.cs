using System;
using Unity.Entities;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [UpdateBefore(typeof(PhysicsShapeConversionSystem))]
    [UpdateBefore(typeof(LegacyBoxColliderConversionSystem))]
    [UpdateBefore(typeof(LegacyCapsuleColliderConversionSystem))]
    [UpdateBefore(typeof(LegacySphereColliderConversionSystem))]
    [UpdateBefore(typeof(LegacyMeshColliderConversionSystem))]
    [DisableAutoCreation]
    [Obsolete("FirstPassPhysicsBodyConversionSystem is no longer used. (RemovedAfter 2019-08-24)")]
    public class FirstPassPhysicsBodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() => throw new NotImplementedException();
    }
}
