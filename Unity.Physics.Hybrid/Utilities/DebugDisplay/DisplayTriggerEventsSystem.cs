using System;
using Unity.Burst;
using Unity.Collections;
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

            DebugStream.Context debugOutput = m_DebugStreamSystem.GetContext(1);
            debugOutput.Begin(0);
            // Allocate a NativeArray to store our debug output, so it can be shared across the display/finish jobs
            var sharedOutput = new NativeArray<DebugStream.Context>(1, Allocator.TempJob);
            sharedOutput[0] = debugOutput;

            var job = new DisplayTriggerEventsJob
            {
                World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                OutputStream = sharedOutput,
            };
            
            JobHandle handle = ScheduleTriggerEventsJob(job, m_StepPhysicsWorldSystem.Simulation, ref m_BuildPhysicsWorldSystem.PhysicsWorld, inputDeps);

#pragma warning disable 618
            JobHandle finishHandle = new FinishDisplayTriggerEventsJob
            {
                OutputStream = sharedOutput
            }.Schedule(handle);
#pragma warning restore 618

            m_EndFramePhysicsSystem.HandlesToWaitFor.Add(finishHandle);

            return handle;
        }

        protected virtual JobHandle ScheduleTriggerEventsJob(DisplayTriggerEventsJob job, ISimulation simulation, ref PhysicsWorld world, JobHandle inDeps)
        {
            // Explicitly call ScheduleImpl here, to avoid a dependency on Havok.Physics
            return job.ScheduleImpl(simulation, ref world, inDeps);
        }

        // Job which iterates over trigger events and writes display info to a DebugStream.
        [BurstCompile]
        protected struct DisplayTriggerEventsJob : ITriggerEventsJob
        {
            [ReadOnly] public PhysicsWorld World;
            public NativeArray<DebugStream.Context> OutputStream;

            public unsafe void Execute(TriggerEvent triggerEvent)
            {
                DebugStream.Context outputContext = OutputStream[0];

                RigidBody bodyA = World.Bodies[triggerEvent.BodyIndices.BodyAIndex];
                RigidBody bodyB = World.Bodies[triggerEvent.BodyIndices.BodyBIndex];

                Aabb aabbA = bodyA.Collider->CalculateAabb(bodyA.WorldFromBody);
                Aabb aabbB = bodyB.Collider->CalculateAabb(bodyB.WorldFromBody);
                outputContext.Line(aabbA.Center, aabbB.Center, UnityEngine.Color.yellow);

                OutputStream[0] = outputContext;
            }
        }

        [BurstCompile]
        [Obsolete("This type will be made protected in a future release. (RemovedAfter 2019-10-15)")]
        public struct FinishDisplayTriggerEventsJob : IJob
        {
            [DeallocateOnJobCompletion]
            public NativeArray<DebugStream.Context> OutputStream;

            public void Execute()
            {
                OutputStream[0].End();
            }
        }
    }
}
