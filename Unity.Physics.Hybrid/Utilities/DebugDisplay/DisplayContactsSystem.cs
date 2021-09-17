#if !HAVOK_PHYSICS_EXISTS

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
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(StepPhysicsWorld))]
    public partial class DisplayContactsSystem : SystemBase
    {
        StepPhysicsWorld m_StepWorld;
        DebugStream m_DebugStreamSystem;

        protected override void OnCreate()
        {
            m_StepWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_DebugStreamSystem = World.GetOrCreateSystem<DebugStream>();
        }

        protected override void OnUpdate()
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawContacts != 0))
            {
                return;
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

                    JobHandle gatherJobHandle = new DisplayContactsJob
                    {
                        DisplayContactIndices = false,
                        OutputStreamContext = sharedOutput
                    }.Schedule(simulation, ref world, inDeps);

                    var finishJob = new FinishDisplayContactsJob
                    {
                        OutputStreamContext = sharedOutput
                    };

                    return finishJob.Schedule(gatherJobHandle);
                }
            };

            m_StepWorld.EnqueueCallback(SimulationCallbacks.Phase.PostCreateContacts, callback);
        }

        // Job which iterates over contacts from narrowphase and writes display info to a DebugStream.
        // Cannot be burst-ed since we're converting int to char[] in order to write it to output stream
        //[BurstCompile]
        private unsafe struct DisplayContactsJob : IContactsJobBase
        {
            internal bool DisplayContactIndices;
            [NativeDisableUnsafePtrRestriction]
            internal DebugStream.Context* OutputStreamContext;

            public void Execute(ref ModifiableContactHeader header, ref ModifiableContactPoint point)
            {
                float3 x0 = point.Position;
                float3 x1 = header.Normal * point.Distance;
                OutputStreamContext->Arrow(x0, x1, Unity.DebugDisplay.ColorIndex.Green);
                if (DisplayContactIndices)
                {
                    // The following line is not Burst-compatible
                    OutputStreamContext->Text(point.Index.ToString().ToCharArray(), x0, UnityEngine.Color.red);
                }
            }
        }

        [BurstCompile]
        private unsafe struct FinishDisplayContactsJob : IJob
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
