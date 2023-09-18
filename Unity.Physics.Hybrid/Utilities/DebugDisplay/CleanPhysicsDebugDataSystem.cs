using Unity.Entities;
using Unity.Physics.Systems;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// A system which cleans physics debug display data from the previous frame.
    /// When using multiple physics worlds, in order for the debug display to work properly, you need to disable
    /// the update of this system in either main physics group (<see cref="PhysicsSystemGroup"/>)
    /// or in the custom physics group, whichever updates later in the loop.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.Default)]
    [UpdateInGroup(typeof(PhysicsDebugDisplayGroup), OrderFirst = true)]
    public partial struct CleanPhysicsDebugDataSystem : ISystem
    {
        void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
        }

        void OnUpdate(ref SystemState state)
        {
            DebugDisplay.DebugDisplay.Clear();
        }
    }

    [WorldSystemFilter(WorldSystemFilterFlags.Editor)]
    [UpdateInGroup(typeof(PhysicsDebugDisplayGroup_Editor), OrderFirst = true)]
    public partial struct CleanPhysicsDebugDataSystem_Editor : ISystem
    {
        void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<PhysicsDebugDisplayData>();
        }

        void OnUpdate(ref SystemState state)
        {
            DebugDisplay.DebugDisplay.Clear();
        }
    }
}
