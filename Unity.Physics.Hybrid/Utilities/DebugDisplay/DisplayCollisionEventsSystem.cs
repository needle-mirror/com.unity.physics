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

            DebugStream.Context debugOutput = m_DebugStreamSystem.GetContext(1);
            debugOutput.Begin(0);

            unsafe
            {
                // Allocate a block of memory to store our debug output, so it can be shared across the display/finish jobs
                var sharedOutput = (DebugStream.Context*)UnsafeUtility.Malloc(sizeof(DebugStream.Context), 16, Allocator.TempJob);
                sharedOutput[0] = debugOutput;

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
        //[BurstCompile]
        protected unsafe struct DisplayCollisionEventsJob : ICollisionEventsJob
        {
            [ReadOnly] public PhysicsWorld World;
            [NativeDisableUnsafePtrRestriction]
            internal DebugStream.Context* OutputStreamContext;

            public unsafe void Execute(CollisionEvent collisionEvent)
            {
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
                    OutputStreamContext->Text(totalImpulse.ToString().ToCharArray(),
                        math.lerp(bodyA.WorldFromBody.pos, bodyB.WorldFromBody.pos, 0.5f), UnityEngine.Color.blue);
                }
            }
        }

        [BurstCompile]
        [Obsolete("This type will be made protected in a future release. (RemovedAfter 2019-10-15)")]
        public unsafe struct FinishDisplayCollisionEventsJob : IJob
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
