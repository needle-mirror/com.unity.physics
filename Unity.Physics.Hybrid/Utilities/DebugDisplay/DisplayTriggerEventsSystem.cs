using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Physics.Systems;

namespace Unity.Physics.Authoring
{
    // A system which draws any trigger events produced by the physics step system
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DisplayTriggerEventsSystem : JobComponentSystem
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

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawTriggerEvents != 0))
            {
                return inputDeps;
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

                JobHandle handle = ScheduleTriggerEventsJob(job, m_StepPhysicsWorldSystem.Simulation, ref m_BuildPhysicsWorldSystem.PhysicsWorld, inputDeps);

#pragma warning disable 618
                JobHandle finishHandle = new FinishDisplayTriggerEventsJob
                {
                    OutputStreamContext = sharedOutput
                }.Schedule(handle);
#pragma warning restore 618

                m_EndFramePhysicsSystem.HandlesToWaitFor.Add(finishHandle);

                return handle;
            }
        }

        protected virtual JobHandle ScheduleTriggerEventsJob(DisplayTriggerEventsJob job, ISimulation simulation, ref PhysicsWorld world, JobHandle inDeps)
        {
            // Explicitly call ScheduleImpl here, to avoid a dependency on Havok.Physics
            return job.ScheduleImpl(simulation, ref world, inDeps);
        }

        // Job which iterates over trigger events and writes display info to a DebugStream.
        [BurstCompile]
        protected unsafe struct DisplayTriggerEventsJob : ITriggerEventsJob
        {
            [ReadOnly] public PhysicsWorld World;
            [NativeDisableUnsafePtrRestriction]
            internal DebugStream.Context* OutputStreamContext;

            public unsafe void Execute(TriggerEvent triggerEvent)
            {
                RigidBody bodyA = World.Bodies[triggerEvent.BodyIndices.BodyAIndex];
                RigidBody bodyB = World.Bodies[triggerEvent.BodyIndices.BodyBIndex];

                Aabb aabbA = bodyA.Collider->CalculateAabb(bodyA.WorldFromBody);
                Aabb aabbB = bodyB.Collider->CalculateAabb(bodyB.WorldFromBody);
                OutputStreamContext->Line(aabbA.Center, aabbB.Center, UnityEngine.Color.yellow);
            }
        }

        [BurstCompile]
        protected unsafe struct FinishDisplayTriggerEventsJob : IJob
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
