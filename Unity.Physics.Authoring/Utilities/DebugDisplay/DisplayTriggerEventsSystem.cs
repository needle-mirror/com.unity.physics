using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using UnityEngine;
using Collider = Unity.Physics.Collider;

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

        protected override void OnCreateManager()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateManager<BuildPhysicsWorld>();
            m_StepPhysicsWorldSystem = World.GetOrCreateManager<StepPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateManager<EndFramePhysicsSystem>();
            m_DebugStreamSystem = World.GetOrCreateManager<DebugStream>();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!(HasSingleton<PhysicsDebugDisplayData>() && GetSingleton<PhysicsDebugDisplayData>().DrawTriggerEvents != 0))
            {
                return inputDeps;
            }

            inputDeps = JobHandle.CombineDependencies(inputDeps, m_BuildPhysicsWorldSystem.FinalJobHandle, m_StepPhysicsWorldSystem.FinalSimulationJobHandle);

            JobHandle handle = new DisplayTriggerEventsJob
            {
                World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                TriggerEvents = m_StepPhysicsWorldSystem.Simulation.TriggerEvents,
                OutputStream = m_DebugStreamSystem.GetContext(1)
            }.Schedule(1, 1, inputDeps);

            m_EndFramePhysicsSystem.HandlesToWaitFor.Add(handle);

            return handle;
        }

        // Job which iterates over trigger events and writes display info to a DebugStream.
        //[BurstCompile]
        struct DisplayTriggerEventsJob : IJobParallelFor
        {
            [ReadOnly] public PhysicsWorld World;
            [ReadOnly] public TriggerEvents TriggerEvents;
            public DebugStream.Context OutputStream;

            public unsafe void Execute(int workItemIndex)
            {
                OutputStream.Begin(workItemIndex);
                foreach (TriggerEvent triggerEvent in TriggerEvents)
                {
                    RigidBody bodyA = World.Bodies[triggerEvent.BodyIndices.BodyAIndex];
                    RigidBody bodyB = World.Bodies[triggerEvent.BodyIndices.BodyBIndex];

                    bool IsTrigger(Collider* collider, ColliderKey key)
                    {
                        if (collider->CollisionType == CollisionType.Convex)
                        {
                            return ((ConvexColliderHeader*)collider)->Material.IsTrigger;
                        }
                        else
                        {
                            collider->GetLeaf(key, out ChildCollider child);
                            collider = child.Collider;
                            UnityEngine.Assertions.Assert.IsTrue(collider->CollisionType == CollisionType.Convex);
                            return ((ConvexColliderHeader*)collider)->Material.IsTrigger;
                        }
                    }

                    char[] text = "Triggered".ToCharArray();
                    if (IsTrigger(bodyA.Collider, triggerEvent.ColliderKeys.ColliderKeyA))
                    {
                        OutputStream.Text(text, bodyA.WorldFromBody.pos, Color.green);
                    }
                    if (IsTrigger(bodyB.Collider, triggerEvent.ColliderKeys.ColliderKeyB))
                    {
                        OutputStream.Text(text, bodyB.WorldFromBody.pos, Color.green);
                    }
                }
                OutputStream.End();
            }
        }
    }
}
