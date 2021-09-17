using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

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
                TerrainCollider.Create(heights, new int2(4, 4), new float3(1f), CollisionMethod).Dispose();
            }
        }

        [Test]
        public void TerrainCollider_Create_WhenCalledFromBurstJob_DoesNotThrow(
            [Values] TerrainCollider.CollisionMethod collisionMethod
        ) =>
            new CreateFromBurstJob { CollisionMethod = collisionMethod }
            .Run();

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void TerrainCollider_Create_WhenSizeOutOfRange_Throws(
            [Values(0, 1)] int errantDimension
        )
        {
            var size = new int2(2) { [errantDimension] = 1 };
            var ex = Assert.Throws<ArgumentOutOfRangeException>(() => TerrainCollider.Create(default, size, default, default));
            Assert.That(ex.ParamName, Is.EqualTo("size"));
        }

        [Test]
        public void TerrainCollider_Create_WhenScaleOutOfRange_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var ex = Assert.Throws<ArgumentOutOfRangeException>(() => TerrainCollider.Create(default, new int2(2), new float3(errantValue), default));
            Assert.That(ex.ParamName, Is.EqualTo("scale"));
        }

#endif

        #endregion
    }
}
