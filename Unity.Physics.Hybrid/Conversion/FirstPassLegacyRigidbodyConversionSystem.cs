using System;
using Unity.Entities;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [UpdateBefore(typeof(FirstPassPhysicsBodyConversionSystem))]
    [DisableAutoCreation]
    [Obsolete("FirstPassLegacyRigidbodyConversionSystem is no longer used. (RemovedAfter 2019-08-24)")]
    public class FirstPassLegacyRigidbodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() => throw new NotImplementedException();
    }
}
