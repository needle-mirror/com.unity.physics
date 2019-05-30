using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// Job to iterate over all the bodies in a scene, for any
    /// which have a collider, calculate the bounding box and
    /// write it to a debug stream.
    public unsafe struct DisplayColliderAabbsJob : IJob //<todo.eoin.udebug This can be a parallelfor job
    {
        public DebugStream.Context OutputStream;
        [ReadOnly] public NativeSlice<RigidBody> Bodies;

        public void Execute()
        {
            OutputStream.Begin(0);

            for (int b = 0; b < Bodies.Length; b++)
            {
                if (Bodies[b].Collider != null)
                {
                    Aabb aabb = Bodies[b].Collider->CalculateAabb(Bodies[b].WorldFromBody);

                    float3 center = aabb.Center;
                    OutputStream.Box(aabb.Extents, center, Quaternion.identity, new Color(0.7f, 0.125f, 0.125f));
                }
            }
            OutputStream.End();
        }
    }

    /// Create a DisplayColliderAabbsJob
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DisplayColliderAabbsSystem : JobComponentSystem
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

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawColliderAabbs != 0))
            {
                return inputDeps;
            }

            if (m_BuildPhysicsWorldSystem.PhysicsWorld.NumBodies == 0)
            {
                return inputDeps;
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
            return inputDeps;
        }
    }
}
