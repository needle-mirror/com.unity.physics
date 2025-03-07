using Unity.Entities;
using Unity.Physics.Systems;

namespace Unity.Physics.Authoring
{
#if UNITY_EDITOR || ENABLE_UNITY_PHYSICS_RUNTIME_DEBUG_DISPLAY

    /// <summary>
    /// A system which cleans physics debug display data from the previous frame while in play mode.
    /// When using multiple physics worlds, in order for the debug display to work properly, you need to disable
    /// the update of this system in any <see cref="PhysicsSystemGroup">physics system group</see> following the first one.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Default)]
    [UpdateInGroup(typeof(PhysicsInitializeGroup), OrderFirst = true)]
    public partial struct CleanPhysicsDebugDataSystem_Default : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
        }

        public void OnUpdate(ref SystemState state)
        {
            DebugDisplay.DebugDisplay.Clear();
        }
    }

    /// <summary>
    /// A system which cleans physics debug display data from the previous frame while in edit mode.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Editor)]
    [UpdateInGroup(typeof(PhysicsDebugDisplayGroup_Editor), OrderFirst = true)]
    public partial struct CleanPhysicsDebugDataSystem_Editor : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
        }

        public void OnUpdate(ref SystemState state)
        {
            DebugDisplay.DebugDisplay.Clear();
        }
    }
#endif
}
