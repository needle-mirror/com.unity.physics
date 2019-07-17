using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics.Systems;

namespace Unity.Physics.Authoring
{
    // A system which draws all contact points produced by the physics step system
    [UpdateBefore(typeof(StepPhysicsWorld))]
    public class DisplayContactsSystem : JobComponentSystem
    {
        StepPhysicsWorld m_StepWorld;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_StepWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawContacts != 0))
            {
                return inputDeps;
            }

            SimulationCallbacks.Callback callback = (ref ISimulation simulation, ref PhysicsWorld world, JobHandle inDeps) =>
            {
                DebugStream.Context debugOutput = m_DebugStreamSystem.GetContext(1);
                debugOutput.Begin(0);
                // Allocate a NativeArray to store our debug output, so it can be shared across the display/finish jobs
                var sharedOutput = new NativeArray<DebugStream.Context>(1, Allocator.TempJob);
                sharedOutput[0] = debugOutput;

                var gatherJob = new DisplayContactsJob
                {
                    OutputStream = sharedOutput
                };

                JobHandle gatherJobHandle = ScheduleContactsJob(gatherJob, simulation, ref world, inDeps);

                var finishJob = new FinishDisplayContactsJob
                {
                    OutputStream = sharedOutput
                };

                return finishJob.Schedule(gatherJobHandle);
            };

            m_StepWorld.EnqueueCallback(SimulationCallbacks.Phase.PostCreateContacts, callback);

            return inputDeps;
        }

        protected virtual JobHandle ScheduleContactsJob(DisplayContactsJob job, ISimulation simulation, ref PhysicsWorld world, JobHandle inDeps)
        {
            // Explicitly call ScheduleImpl here, to avoid a dependency on Havok.Physics
            return job.ScheduleImpl(simulation, ref world, inDeps);
        }

        // Job which iterates over contacts from narrowphase and writes display info to a DebugStream.
        [BurstCompile]
        protected struct DisplayContactsJob : IContactsJob
        {
            public NativeArray<DebugStream.Context> OutputStream;

            public void Execute(ref ModifiableContactHeader header, ref ModifiableContactPoint point)
            {
                DebugStream.Context output = OutputStream[0];
                float3 x0 = point.Position;
                float3 x1 = header.Normal * point.Distance;
                output.Arrow(x0, x1, UnityEngine.Color.green);
                OutputStream[0] = output;
            }
        }

        [BurstCompile]
        protected struct FinishDisplayContactsJob : IJob
        {
            [DeallocateOnJobCompletion]
            public NativeArray<DebugStream.Context> OutputStream;

            public void Execute()
            {
                OutputStream[0].End();
            }
        }
    }

    [Obsolete("DisplayContactsJob has been deprecated. Use DisplayContactsSystem.DisplayContactsJob instead. (RemovedAfter 2019-10-15)")]
    public struct DisplayContactsJob : IContactsJob
    {
        public NativeArray<DebugStream.Context> OutputStream;

        public void Execute(ref ModifiableContactHeader header, ref ModifiableContactPoint point) { }
    }

    [Obsolete("FinishDisplayContactsJob has been deprecated. Use DisplayContactsSystem.FinishDisplayContactsJob instead. (RemovedAfter 2019-10-15)")]
    public struct FinishDisplayContactsJob : IJob
    {
        [DeallocateOnJobCompletion]
        public NativeArray<DebugStream.Context> OutputStream;

        public void Execute() => OutputStream[0].End();
    }
}
