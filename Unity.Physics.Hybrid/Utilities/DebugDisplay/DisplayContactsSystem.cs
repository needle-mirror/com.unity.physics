using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Systems;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    // Job which iterates over contacts from narrowphase and writes display info to a DebugStream.
    [BurstCompile]
    public unsafe struct DisplayContactsJob : IContactsJob
    {
        public NativeArray<DebugStream.Context> OutputStream;

        public void Execute(ref ModifiableContactHeader header, ref ModifiableContactPoint point)
        {
            DebugStream.Context output = OutputStream[0];
            float3 x0 = point.Position;
            float3 x1 = header.Normal * point.Distance;
            Color color = Color.green;
            output.Arrow(x0, x1, color);
            OutputStream[0] = output;
        }
    }

    public unsafe struct FinishDisplayContactsJob : IJob
    {
        [DeallocateOnJobCompletion]
        public NativeArray<DebugStream.Context> OutputStream;

        public void Execute()
        {
            OutputStream[0].End();
        }
    }

    // Create DisplayContactsJob
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

        protected unsafe override JobHandle OnUpdate(JobHandle inputDeps)
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
                NativeArray<DebugStream.Context> sharedOutput = new NativeArray<DebugStream.Context>(1, Allocator.TempJob);
                sharedOutput[0] = debugOutput;

                var gatherJob = new DisplayContactsJob
                {
                    OutputStream = sharedOutput
                };

                // Explicitly call ScheduleImpl here, to avoid a dependency on Havok.Physics
                JobHandle gatherJobHandle = gatherJob.ScheduleImpl(simulation, ref world, inDeps);

                var finishJob = new FinishDisplayContactsJob
                {
                    OutputStream = sharedOutput
                };

                return finishJob.Schedule(gatherJobHandle);
            };

            m_StepWorld.EnqueueCallback(SimulationCallbacks.Phase.PostCreateContacts, callback);


            return inputDeps;
        }
    }
}
