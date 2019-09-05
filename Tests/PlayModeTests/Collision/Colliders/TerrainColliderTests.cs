using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Tests.Collision.Colliders
{
    class TerrainColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public TerrainCollider.CollisionMethod CollisionMethod;

            public void Execute()
            {
                var heights = new NativeArray<float>(16, Allocator.Temp);
                TerrainCollider.Create(heights, new int2(4, 4), new float3(1f), CollisionMethod).Release();
            }
        }

        [Test]
        public void TerrainCollider_Create_WhenCalledFromBurstJob_DoesNotThrow(
            [Values] TerrainCollider.CollisionMethod collisionMethod
        ) =>
            new CreateFromBurstJob { CollisionMethod = collisionMethod }.Run();

        #endregion
    }
}
