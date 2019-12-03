using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
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
                unsafe
                {
                    // Allocate a block of memory to store our debug output, so it can be shared across the display/finish jobs
                    var sharedOutput = (DebugStream.Context*)UnsafeUtility.Malloc(sizeof(DebugStream.Context), 16, Allocator.TempJob);
                    *sharedOutput = m_DebugStreamSystem.GetContext(1);
                    sharedOutput->Begin(0);

                    var gatherJob = new DisplayContactsJob
                    {
                        DisplayContactIndices = false,
                        OutputStreamContext = sharedOutput
                    };

                    JobHandle gatherJobHandle = ScheduleContactsJob(gatherJob, simulation, ref world, inDeps);

                    var finishJob = new FinishDisplayContactsJob
                    {
                        OutputStreamContext = sharedOutput
                    };

                    return finishJob.Schedule(gatherJobHandle);
                }
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
        //[BurstCompile]
        protected unsafe struct DisplayContactsJob : IContactsJob
        {
            internal bool DisplayContactIndices;
            [NativeDisableUnsafePtrRestriction]
            internal DebugStream.Context* OutputStreamContext;

            public void Execute(ref ModifiableContactHeader header, ref ModifiableContactPoint point)
            {
                float3 x0 = point.Position;
                float3 x1 = header.Normal * point.Distance;
                OutputStreamContext->Arrow(x0, x1, UnityEngine.Color.green);
                if (DisplayContactIndices)
                {
                    OutputStreamContext->Text(point.Index.ToString().ToCharArray(), x0, UnityEngine.Color.red);
                }
            }
        }

        [BurstCompile]
        protected unsafe struct FinishDisplayContactsJob : IJob
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
