using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Physics.Systems;

namespace Unity.Physics.Authoring
{
    // A system which draws any collision events produced by the physics step system
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DisplayCollisionEventsSystem : JobComponentSystem
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
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawCollisionEvents != 0))
            {
                return inputDeps;
            }

            unsafe
            {
                // Allocate a block of memory to store our debug output, so it can be shared across the display/finish jobs
                var sharedOutput = (DebugStream.Context*)UnsafeUtility.Malloc(sizeof(DebugStream.Context), 16, Allocator.TempJob);
                *sharedOutput = m_DebugStreamSystem.GetContext(1);
                sharedOutput->Begin(0);

                var job = new DisplayCollisionEventsJob
                {
                    World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                    OutputStreamContext = sharedOutput
                };

                JobHandle handle = ScheduleCollisionEventsJob(job, m_StepPhysicsWorldSystem.Simulation, ref m_BuildPhysicsWorldSystem.PhysicsWorld, inputDeps);

#pragma warning disable 618
                JobHandle finishHandle = new FinishDisplayCollisionEventsJob
                {
                    OutputStreamContext = sharedOutput
                }.Schedule(handle);
#pragma warning restore 618

                m_EndFramePhysicsSystem.HandlesToWaitFor.Add(finishHandle);

                return handle;
            }
        }

        protected virtual JobHandle ScheduleCollisionEventsJob(DisplayCollisionEventsJob job, ISimulation simulation, ref PhysicsWorld world, JobHandle inDeps)
        {
            // Explicitly call ScheduleImpl here, to avoid a dependency on Havok.Physics
            return job.ScheduleImpl(simulation, ref world, inDeps);
        }

        // Job which iterates over collision events and writes display info to a DebugStream.
        [BurstCompile]
        protected unsafe struct DisplayCollisionEventsJob : ICollisionEventsJob
        {
            [ReadOnly] public PhysicsWorld World;
            [NativeDisableUnsafePtrRestriction]
            internal DebugStream.Context* OutputStreamContext;

            public unsafe void Execute(CollisionEvent collisionEvent)
            {
                CollisionEvent.Details details = collisionEvent.CalculateDetails(ref World);

                // Color code the impulse depending on the collision feature
                // vertex - blue
                // edge - cyan
                // face - magenta
                UnityEngine.Color color;
                switch (details.EstimatedContactPointPositions.Length)
                {
                    case 1:
                        color = UnityEngine.Color.blue;
                        break;
                    case 2:
                        color = UnityEngine.Color.cyan;
                        break;
                    default:
                        color = UnityEngine.Color.magenta;
                        break;
                }

                var averageContactPosition = details.AverageContactPointPosition;
                OutputStreamContext->Point(averageContactPosition, 0.01f, color);
                OutputStreamContext->Arrow(averageContactPosition, collisionEvent.Normal * details.EstimatedImpulse, color);
            }
        }

        [BurstCompile]
        protected unsafe struct FinishDisplayCollisionEventsJob : IJob
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
