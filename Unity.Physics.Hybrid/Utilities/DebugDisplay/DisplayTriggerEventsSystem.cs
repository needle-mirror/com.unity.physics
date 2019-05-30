using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using UnityEngine;
using Collider = Unity.Physics.Collider;
using Unity.Mathematics;
using Unity.Burst;

namespace Unity.Physics.Authoring
{
    // A systems which draws any trigger events produced by the physics step system
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

        protected unsafe override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawTriggerEvents != 0))
            {
                return inputDeps;
            }

            inputDeps = JobHandle.CombineDependencies(inputDeps, m_BuildPhysicsWorldSystem.FinalJobHandle, m_StepPhysicsWorldSystem.FinalSimulationJobHandle);

            DebugStream.Context debugOutput = m_DebugStreamSystem.GetContext(1);
            debugOutput.Begin(0);
            // Allocate a NativeArray to store our debug output, so it can be shared across the display/finish jobs
            NativeArray<DebugStream.Context> sharedOutput = new NativeArray<DebugStream.Context>(1, Allocator.TempJob);
            sharedOutput[0] = debugOutput;

            // Explicitly call ScheduleImpl here, to avoid a dependency on Havok.Physics
            JobHandle handle = new DisplayTriggerEventsJob
            {
                World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                OutputStream = sharedOutput,
            }.ScheduleImpl(m_StepPhysicsWorldSystem.Simulation, ref m_BuildPhysicsWorldSystem.PhysicsWorld, inputDeps);

            JobHandle finishHandle = new FinishDisplayTriggerEventsJob
            {
                OutputStream = sharedOutput
            }.Schedule(handle);

            m_EndFramePhysicsSystem.HandlesToWaitFor.Add(finishHandle);

            return handle;
        }

        // Job which iterates over trigger events and writes display info to a DebugStream.
        [BurstCompile]
        unsafe struct DisplayTriggerEventsJob : ITriggerEventsJob
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
                outputContext.Line(aabbA.Center, aabbB.Center, Color.yellow);

                OutputStream[0] = outputContext;
            }
        }

        public unsafe struct FinishDisplayTriggerEventsJob : IJob
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
