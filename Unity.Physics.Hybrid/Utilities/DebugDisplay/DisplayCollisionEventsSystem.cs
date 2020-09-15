#if !HAVOK_PHYSICS_EXISTS

using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Physics.Systems;

namespace Unity.Physics.Authoring
{
    // A system which draws any collision events produced by the physics step system
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem))]
    public class DisplayCollisionEventsSystem : SystemBase
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
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawCollisionEvents != 0))
            {
                return;
            }

            unsafe
            {
                // Allocate a block of memory to store our debug output, so it can be shared across the display/finish jobs
                var sharedOutput = (DebugStream.Context*)UnsafeUtility.Malloc(sizeof(DebugStream.Context), 16, Allocator.TempJob);
                *sharedOutput = m_DebugStreamSystem.GetContext(1);
                sharedOutput->Begin(0);

                // This will call the extension method defined in Unity.Physics
                JobHandle handle = new DisplayCollisionEventsJob
                {
                    World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                    OutputStreamContext = sharedOutput
                }.Schedule(m_StepPhysicsWorldSystem.Simulation, ref m_BuildPhysicsWorldSystem.PhysicsWorld, Dependency);

#pragma warning disable 618

                JobHandle finishHandle = new FinishDisplayCollisionEventsJob
                {
                    OutputStreamContext = sharedOutput
                }.Schedule(handle);
#pragma warning restore 618

                m_EndFramePhysicsSystem.AddInputDependency(finishHandle);

                Dependency = handle;
            }
        }

        // Job which iterates over collision events and writes display info to a DebugStream.
        [BurstCompile]
        private unsafe struct DisplayCollisionEventsJob : ICollisionEventsJobBase
        {
            [ReadOnly] public PhysicsWorld World;
            [NativeDisableUnsafePtrRestriction]
            public DebugStream.Context* OutputStreamContext;

            public unsafe void Execute(CollisionEvent collisionEvent)
            {
                CollisionEvent.Details details = collisionEvent.CalculateDetails(ref World);

                // Color code the impulse depending on the collision feature
                // vertex - blue
                // edge - cyan
                // face - magenta
                Unity.DebugDisplay.ColorIndex color;
                switch (details.EstimatedContactPointPositions.Length)
                {
                    case 1:
                        color = Unity.DebugDisplay.ColorIndex.Blue;
                        break;
                    case 2:
                        color = Unity.DebugDisplay.ColorIndex.Cyan;
                        break;
                    default:
                        color = Unity.DebugDisplay.ColorIndex.Magenta;
                        break;
                }

                var averageContactPosition = details.AverageContactPointPosition;
                OutputStreamContext->Point(averageContactPosition, 0.01f, color);
                OutputStreamContext->Arrow(averageContactPosition, collisionEvent.Normal * details.EstimatedImpulse, color);
            }
        }

        [BurstCompile]
        private unsafe struct FinishDisplayCollisionEventsJob : IJob
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
