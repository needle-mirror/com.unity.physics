using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Collider = Unity.Physics.Collider;

namespace Unity.Physics.Authoring
{
    // A systems which draws any collision events produced by the physics step system
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

        protected unsafe override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawCollisionEvents != 0))
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
            JobHandle handle = new DisplayCollisionEventsJob
            {
                World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                OutputStream = sharedOutput
            }.ScheduleImpl(m_StepPhysicsWorldSystem.Simulation, ref m_BuildPhysicsWorldSystem.PhysicsWorld, inputDeps);

            JobHandle finishHandle = new FinishDisplayCollisionEventsJob
            {
                OutputStream = sharedOutput
            }.Schedule(handle);

            m_EndFramePhysicsSystem.HandlesToWaitFor.Add(finishHandle);

            return handle;
        }

        // Job which iterates over collision events and writes display info to a DebugStream.
        //[BurstCompile]
        unsafe struct DisplayCollisionEventsJob : ICollisionEventsJob
        {
            [ReadOnly] public PhysicsWorld World;
            public NativeArray<DebugStream.Context> OutputStream;

            public void Execute(CollisionEvent collisionEvent)
            {
                DebugStream.Context outputContext = OutputStream[0];

                RigidBody bodyA = World.Bodies[collisionEvent.BodyIndices.BodyAIndex];
                RigidBody bodyB = World.Bodies[collisionEvent.BodyIndices.BodyBIndex];
                float totalImpulse = math.csum(collisionEvent.AccumulatedImpulses);

                bool AreCollisionEventsEnabled(Collider* collider, ColliderKey key)
                {
                    if (collider->CollisionType == CollisionType.Convex)
                    {
                        return ((ConvexColliderHeader*)collider)->Material.EnableCollisionEvents;
                    }
                    else
                    {
                        collider->GetLeaf(key, out ChildCollider child);
                        collider = child.Collider;
                        UnityEngine.Assertions.Assert.IsTrue(collider->CollisionType == CollisionType.Convex);
                        return ((ConvexColliderHeader*)collider)->Material.EnableCollisionEvents;
                    }
                }

                bool areBodyACollisionEventsEnabled = AreCollisionEventsEnabled(bodyA.Collider, collisionEvent.ColliderKeys.ColliderKeyA);
                bool areBodyBCollisionEventsEnabled = AreCollisionEventsEnabled(bodyB.Collider, collisionEvent.ColliderKeys.ColliderKeyB);
                if (areBodyACollisionEventsEnabled || areBodyBCollisionEventsEnabled)
                {
                    outputContext.Text(totalImpulse.ToString().ToCharArray(), 
                        math.lerp(bodyA.WorldFromBody.pos, bodyB.WorldFromBody.pos, 0.5f), Color.blue);
                }

                OutputStream[0] = outputContext;
            }
        }

        public unsafe struct FinishDisplayCollisionEventsJob : IJob
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
