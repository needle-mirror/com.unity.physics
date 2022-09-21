using Unity.Entities;

namespace Unity.Physics.Systems
{
#if !HAVOK_PHYSICS_EXISTS

    /// <summary>
    /// This abstract class can be used to create a system group for a custom physics world.
    /// To create a custom physics group, derive from this class and implement empty constructor
    /// which calls one of two constructors of this class, and potentially implement some of the
    /// other virtual functions.
    /// </summary>
    public abstract partial class CustomPhysicsSystemGroup : CustomPhysicsSystemGroupBase
    {
        private Simulation m_StoredSimulation;

        /// <summary>
        /// Constructor. Your subclass needs to implement an empty constructor which is calling this one to properly set up the world index.
        /// </summary>
        /// <param name="worldIndex"> A world index for a physics world. </param>
        /// <param name="shareStaticColliders"> Should static colliders be shared between main world and this one. </param>
        protected CustomPhysicsSystemGroup(uint worldIndex, bool shareStaticColliders) : base(worldIndex, shareStaticColliders)
        {
            m_StoredSimulation = default;
        }

        protected override void OnCreate()
        {
            base.OnCreate();
            m_StoredSimulation = Simulation.Create();
        }

        protected override void OnDestroy()
        {
            m_StoredSimulation.Dispose();
            base.OnDestroy();
        }

        /// <summary>
        /// Called before the systems in this group are updated. It is useful in cases of needing to store system state (such as NativeArrays, NativeLists etc), before it is ran in a custom group.
        /// If overriding this method, make sure to call base.PreGroupUpdateCallback().
        /// </summary>
        protected override void PreGroupUpdateCallback()
        {
            var broadphaseSystem = World.Unmanaged.GetExistingUnmanagedSystem<BroadphaseSystem>();
            ref var boradphaseData = ref EntityManager.GetComponentDataRW<BroadphaseData>(broadphaseSystem).ValueRW;
            var currentSimulation = boradphaseData.m_UnityPhysicsSimulation;
            boradphaseData.m_UnityPhysicsSimulation = m_StoredSimulation;
            m_StoredSimulation = currentSimulation;

            ref SystemState pickerSystemState = ref World.Unmanaged.GetExistingSystemState<PhysicsSimulationPickerSystem>();
            pickerSystemState.Enabled = false; // disable simulation switching in custom worlds
        }

        /// <summary>
        /// Called after the systems in this group are updated. It is useful in cases of needing to restore system state (such as NativeArrays, NativeLists etc), after it is ran in a custom group.
        /// If overriding this method, make sure to call base.PostGroupUpdateCallback().
        /// </summary>
        protected override void PostGroupUpdateCallback()
        {
            var broadphaseSystem = World.Unmanaged.GetExistingUnmanagedSystem<BroadphaseSystem>();
            ref var boradphaseData = ref EntityManager.GetComponentDataRW<BroadphaseData>(broadphaseSystem).ValueRW;
            var currentSimulation = boradphaseData.m_UnityPhysicsSimulation;
            boradphaseData.m_UnityPhysicsSimulation = m_StoredSimulation;
            m_StoredSimulation = currentSimulation;

            ref SystemState pickerSystemState = ref World.Unmanaged.GetExistingSystemState<PhysicsSimulationPickerSystem>();
            pickerSystemState.Enabled = true; // enable switching back in main world
        }
    }

#endif
}
