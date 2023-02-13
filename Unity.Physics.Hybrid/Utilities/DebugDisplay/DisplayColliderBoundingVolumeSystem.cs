using Unity.Physics;
using Unity.Physics.Systems;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using Unity.Burst;

namespace Unity.Physics.Authoring
{
    /// Job to iterate over all the bodies in a scene, for any
    /// which have a collider, calculate the bounding box and
    /// display it.
    [BurstCompile]
    public struct DisplayColliderAabbsJob : IJobParallelFor
    {
        [ReadOnly] public NativeArray<RigidBody> Bodies;

        public void Execute(int b)
        {
            if (Bodies[b].Collider.IsCreated)
            {
                Aabb aabb = Bodies[b].CalculateAabb();
                float3 center = aabb.Center;
                PhysicsDebugDisplaySystem.Box(aabb.Extents, center, Quaternion.identity, DebugDisplay.ColorIndex.BrightRed);
            }
        }
    }

    /// Create a DisplayColliderAabbsJob
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [UpdateAfter(typeof(PhysicsCreateBodyPairsGroup)), UpdateBefore(typeof(PhysicsCreateContactsGroup))]
    [BurstCompile]
    internal partial struct DisplayColliderAabbsSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
#if UNITY_EDITOR
            if (!SystemAPI.TryGetSingleton(out PhysicsDebugDisplayData debugDisplay) || debugDisplay.DrawColliderAabbs == 0)
                return;

            var world = SystemAPI.GetSingleton<PhysicsWorldSingleton>().PhysicsWorld;
            if (world.NumBodies == 0)
            {
                return;
            }

            state.Dependency = new DisplayColliderAabbsJob()
            {
                Bodies = world.Bodies
            }.Schedule(world.NumBodies, 16, state.Dependency);
#endif
        }
    }
}
