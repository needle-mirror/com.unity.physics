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

        protected override void OnCreateManager()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateManager<BuildPhysicsWorld>();
            m_StepPhysicsWorldSystem = World.GetOrCreateManager<StepPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateManager<EndFramePhysicsSystem>();
            m_DebugStreamSystem = World.GetOrCreateManager<DebugStream>();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawCollisionEvents != 0))
            {
                return inputDeps;
            }

            inputDeps = JobHandle.CombineDependencies(inputDeps, m_BuildPhysicsWorldSystem.FinalJobHandle, m_StepPhysicsWorldSystem.FinalSimulationJobHandle);

            JobHandle handle = new DisplayCollisionEventsJob
            {
                World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                CollisionEvents = m_StepPhysicsWorldSystem.Simulation.CollisionEvents,
                OutputStream = m_DebugStreamSystem.GetContext(1)
            }.Schedule(1, 1, inputDeps);

            m_EndFramePhysicsSystem.HandlesToWaitFor.Add(handle);

            return handle;
        }

        // Job which iterates over collision events and writes display info to a DebugStream.
        //[BurstCompile]
        struct DisplayCollisionEventsJob : IJobParallelFor
        {
            [ReadOnly] public PhysicsWorld World;
            [ReadOnly] public CollisionEvents CollisionEvents;
            public DebugStream.Context OutputStream;

            public unsafe void Execute(int workItemIndex)
            {
                OutputStream.Begin(workItemIndex);
                foreach (CollisionEvent collisionEvent in CollisionEvents)
                {
                    float3 offset = new float3(0, 1, 0);

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

                    if (AreCollisionEventsEnabled(bodyA.Collider, collisionEvent.ColliderKeys.ColliderKeyA))
                    {
                        OutputStream.Text(totalImpulse.ToString().ToCharArray(), bodyA.WorldFromBody.pos + offset, Color.blue);
                    }
                    if (AreCollisionEventsEnabled(bodyB.Collider, collisionEvent.ColliderKeys.ColliderKeyB))
                    {
                        OutputStream.Text(totalImpulse.ToString().ToCharArray(), bodyB.WorldFromBody.pos + offset, Color.blue);
                    }
                }
                OutputStream.End();
            }
        }
    }
}
