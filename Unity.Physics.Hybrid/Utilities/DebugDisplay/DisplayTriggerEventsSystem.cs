#if !HAVOK_PHYSICS_EXISTS

using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Physics.Systems;

namespace Unity.Physics.Authoring
{
    // A system which draws any trigger events produced by the physics step system
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DisplayTriggerEventsSystem : SystemBase
    {
        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        StepPhysicsWorld m_StepPhysicsWorldSystem;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override void OnUpdate()
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawTriggerEvents != 0))
            {
                return;
            }

            unsafe
            {
                // Allocate a block of memory to store our debug output, so it can be shared across the display/finish jobs
                var sharedOutput = (DebugStream.Context*)UnsafeUtility.Malloc(sizeof(DebugStream.Context), 16, Allocator.TempJob);
                *sharedOutput = m_DebugStreamSystem.GetContext(1);
                sharedOutput->Begin(0);

                var job = new DisplayTriggerEventsJob
                {
                    World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                    OutputStreamContext = sharedOutput
                };

                JobHandle handle = new DisplayTriggerEventsJob
                {
                    World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                    OutputStreamContext = sharedOutput
                }.Schedule(m_StepPhysicsWorldSystem.Simulation, ref m_BuildPhysicsWorldSystem.PhysicsWorld, Dependency);

#pragma warning disable 618
                JobHandle finishHandle = new FinishDisplayTriggerEventsJob
                {
                    OutputStreamContext = sharedOutput
                }.Schedule(handle);
#pragma warning restore 618

                m_EndFramePhysicsSystem.AddInputDependency(finishHandle);

                Dependency = handle;
            }
        }

        // Job which iterates over trigger events and writes display info to a DebugStream.
        [BurstCompile]
        private unsafe struct DisplayTriggerEventsJob : ITriggerEventsJobBase
        {
            [ReadOnly] public PhysicsWorld World;
            [NativeDisableUnsafePtrRestriction]
            internal DebugStream.Context* OutputStreamContext;

            public unsafe void Execute(TriggerEvent triggerEvent)
            {
                RigidBody bodyA = World.Bodies[triggerEvent.BodyIndexA];
                RigidBody bodyB = World.Bodies[triggerEvent.BodyIndexB];

                Aabb aabbA = bodyA.Collider.Value.CalculateAabb(bodyA.WorldFromBody);
                Aabb aabbB = bodyB.Collider.Value.CalculateAabb(bodyB.WorldFromBody);
                OutputStreamContext->Line(aabbA.Center, aabbB.Center, Unity.DebugDisplay.ColorIndex.Yellow);
            }
        }

        [BurstCompile]
        private unsafe struct FinishDisplayTriggerEventsJob : IJob
        {
            [NativeDisableUnsafePtrRestriction]
            internal DebugStream.Context* OutputStreamContext;

            public void Execute()
            {
                OutputStreamContext->End();
                UnsafeUtility.Free(OutputStreamContext, Allocator.TempJob);
            }
        }
    }
}

#endif
