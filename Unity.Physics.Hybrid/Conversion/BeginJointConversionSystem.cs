using Unity.Entities;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// A system that is updated before all built-in conversion systems that produce <see cref="PhysicsJoint"/>.
    /// </summary>
    [AlwaysUpdateSystem]
    [UpdateAfter(typeof(PhysicsBodyConversionSystem))]
#if LEGACY_PHYSICS
    [UpdateAfter(typeof(LegacyRigidbodyConversionSystem))]
#endif
    public sealed class BeginJointConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() { }
    }
}
