using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Unity.Burst;

namespace Unity.Physics.Authoring
{
    /// Job to iterate over all the bodies in a scene, for any
    /// which have a collider, calculate the bounding box and
    /// write it to a debug stream.
    [BurstCompile]
    public unsafe struct DisplayColliderAabbsJob : IJob //<todo.eoin.udebug This can be a parallelfor job
    {
        public DebugStream.Context OutputStream;
        [ReadOnly] public NativeArray<RigidBody> Bodies;

        public void Execute()
        {
            OutputStream.Begin(0);

            for (int b = 0; b < Bodies.Length; b++)
            {
                if (Bodies[b].Collider.IsCreated)
                {
                    Aabb aabb = Bodies[b].Collider.Value.CalculateAabb(Bodies[b].WorldFromBody);

                    float3 center = aabb.Center;
                    OutputStream.Box(aabb.Extents, center, Quaternion.identity, DebugDisplay.ColorIndex.BrightRed);
                }
            }
            OutputStream.End();
        }
    }

    /// Create a DisplayColliderAabbsJob
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(StepPhysicsWorld))]
    public partial class DisplayColliderAabbsSystem : SystemBase
    {
        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        StepPhysicsWorld m_StepPhysicsWorld;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override void OnUpdate()
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawColliderAabbs != 0))
            {
                return;
            }

            if (m_BuildPhysicsWorldSystem.PhysicsWorld.NumBodies == 0)
            {
                return;
            }

            SimulationCallbacks.Callback callback = (ref ISimulation simulation, ref PhysicsWorld world, JobHandle deps) =>
            {
                return new DisplayColliderAabbsJob
                {
                    OutputStream = m_DebugStreamSystem.GetContext(1),
                    Bodies = m_BuildPhysicsWorldSystem.PhysicsWorld.Bodies
                }.Schedule(deps);
            };
            m_StepPhysicsWorld.EnqueueCallback(SimulationCallbacks.Phase.PostCreateDispatchPairs, callback);
        }
    }
}
