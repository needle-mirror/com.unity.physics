using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using UnityComponent = UnityEngine.Component;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// A system that is updated after all built-in conversion systems that produce <see cref="PhysicsJoint"/>.
    /// </summary>
    [UpdateAfter(typeof(BeginJointBakingSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial class EndJointBakingSystem : SystemBase
    {
        protected override void OnUpdate() {}
    }
}
