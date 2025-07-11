using System;
using System.Diagnostics;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Physics.Tests.Utils;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Collision.World
{
    class CollisionWorldTests
    {
        //Tests creating a Zero body world
        [Test]
        public void ZeroBodyInitTest()
        {
            CollisionWorld world = new CollisionWorld(0, 0);
            Assert.IsTrue(world.NumBodies == 0);
            world.Dispose();
        }

        //Tests creating a 10 body world
        [Test]
        public void TenBodyInitTest()
        {
            CollisionWorld world = new CollisionWorld(10, 0);
            Assert.IsTrue(world.NumBodies == 10);
            // The bodies/colliders in this world not not initialized, so they do not need to be disposed.
            world.Dispose();
        }

        //Tests updating an empty world
        [Test]
        public void ScheduleUpdateJobsEmptyWorldTest()
        {
            for (int numThreads = 0; numThreads <= 1; numThreads++)
            {
                Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld();
                Jobs.JobHandle handle = new Jobs.JobHandle();
                Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up(), handle, numThreads == 1);
                worldJobHandle.Complete();
                Assert.IsTrue(worldJobHandle.IsCompleted);
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        //Tests updating an empty world
        [Test]
        public void UpdateEmptyWorldTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld();
            world.CollisionWorld.UpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up());
            TestUtils.DisposeAllColliderBlobs(ref world);
            world.Dispose();
        }

        //Tests updating a static box
        [Test]
        public void ScheduleUpdateJobsOneStaticBoxTest()
        {
            for (int numThreads = 0; numThreads <= 1; numThreads++)
            {
                Unity.Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(1);
                BroadPhaseTests.addStaticBoxToWorld(world, 0, Vector3.zero, quaternion.identity, new Vector3(10, .1f, 10));
                Jobs.JobHandle handle = new Jobs.JobHandle();
                Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up(), handle, numThreads == 1);
                worldJobHandle.Complete();
                Assert.IsTrue(worldJobHandle.IsCompleted);
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        //Tests updating a static box
        [Test]
        public void UpdateWorldOneStaticBoxTest()
        {
            Unity.Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(1);
            BroadPhaseTests.addStaticBoxToWorld(world, 0, Vector3.zero, quaternion.identity, new Vector3(10, .1f, 10));
            world.CollisionWorld.UpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up());
            TestUtils.DisposeAllColliderBlobs(ref world);
            world.Dispose();
        }

        //Tests updating 10 static boxes
        [Test]
        public void ScheduleUpdateJobsTenStaticBoxesTest()
        {
            for (int numThreads = 0; numThreads <= 1; numThreads++)
            {
                Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(10);
                for (int i = 0; i < 10; ++i)
                    BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
                Jobs.JobHandle handle = new Jobs.JobHandle();
                Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up(), handle, numThreads == 1);
                worldJobHandle.Complete();
                Assert.IsTrue(worldJobHandle.IsCompleted);
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        //Tests updating 10 static boxes
        [Test]
        public void UpdateWorldTenStaticBoxesTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(10);
            for (int i = 0; i < 10; ++i)
                BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            world.CollisionWorld.UpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up());
            TestUtils.DisposeAllColliderBlobs(ref world);
            world.Dispose();
        }

        //Tests updating 100 static boxes
        [Test]
        public void ScheduleUpdateJobsOneHundredStaticBoxesTest()
        {
            for (int numThreads = 0; numThreads <= 1; numThreads++)
            {
                Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(100);
                for (int i = 0; i < 100; ++i)
                    BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
                Jobs.JobHandle handle = new Jobs.JobHandle();
                Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up(), handle, numThreads == 1);
                worldJobHandle.Complete();
                Assert.IsTrue(worldJobHandle.IsCompleted);
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        //Tests updating 100 static boxes
        [Test]
        public void UpdateWorldOneHundredStaticBoxesTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(100);
            for (int i = 0; i < 100; ++i)
                BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            world.CollisionWorld.UpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up());
            TestUtils.DisposeAllColliderBlobs(ref world);
            world.Dispose();
        }

        //Tests updating a Dynamic box
        [Test]
        public void ScheduleUpdateJobsOneDynamicBoxTest()
        {
            for (int numThreads = 0; numThreads <= 1; numThreads++)
            {
                Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(0, 1);
                BroadPhaseTests.addDynamicBoxToWorld(world, 0, Vector3.zero, quaternion.identity, new Vector3(10, .1f, 10));
                Jobs.JobHandle handle = new Jobs.JobHandle();
                Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up(), handle, numThreads == 1);
                worldJobHandle.Complete();
                Assert.IsTrue(worldJobHandle.IsCompleted);
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        //Tests updating a Dynamic box
        [Test]
        public void UpdateWorldOneDynamicBoxTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(0, 1);
            BroadPhaseTests.addDynamicBoxToWorld(world, 0, Vector3.zero, quaternion.identity, new Vector3(10, .1f, 10));
            world.CollisionWorld.UpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up());
            TestUtils.DisposeAllColliderBlobs(ref world);
            world.Dispose();
        }

        //Tests updating 10 dynamic boxes
        [Test]
        public void ScheduleUpdateJobsTenDynamicBoxesTest()
        {
            for (int numThreads = 0; numThreads <= 1; numThreads++)
            {
                Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(0, 10);
                for (int i = 0; i < 10; ++i)
                    BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
                Jobs.JobHandle handle = new Jobs.JobHandle();
                Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up(), handle, numThreads == 1);
                worldJobHandle.Complete();
                Assert.IsTrue(worldJobHandle.IsCompleted);
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        //Tests updating 10 dynamic boxes
        [Test]
        public void UpdateWorldTenDynamicBoxesTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(0, 10);
            for (int i = 0; i < 10; ++i)
                BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            world.CollisionWorld.UpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up());
            TestUtils.DisposeAllColliderBlobs(ref world);
            world.Dispose();
        }

        //Tests updating 100 dynamic boxes
        [Test]
        public void ScheduleUpdateJobsOneHundredDynamicBoxesTest()
        {
            for (int numThreads = 0; numThreads <= 1; numThreads++)
            {
                Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(0, 100);
                for (int i = 0; i < 100; ++i)
                    BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
                Jobs.JobHandle handle = new Jobs.JobHandle();
                Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up(), handle, numThreads == 1);
                worldJobHandle.Complete();
                Assert.IsTrue(worldJobHandle.IsCompleted);
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        //Tests updating 100 dynamic boxes
        [Test]
        public void UpdateWorldOneHundredDynamicBoxesTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(0, 100);
            for (int i = 0; i < 100; ++i)
                BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            world.CollisionWorld.UpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up());
            TestUtils.DisposeAllColliderBlobs(ref world);
            world.Dispose();
        }

        //Tests updating 100 static and dynamic boxes
        [Test]
        public void ScheduleUpdateJobsStaticAndDynamicBoxesTest()
        {
            for (int numThreads = 0; numThreads <= 1; numThreads++)
            {
                Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(100, 100);
                for (int i = 0; i < 100; ++i)
                {
                    BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
                    BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
                }

                Jobs.JobHandle handle = new Jobs.JobHandle();
                Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up(), handle, numThreads == 1);
                worldJobHandle.Complete();
                Assert.IsTrue(worldJobHandle.IsCompleted);
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        //Tests updating 100 static and dynamic boxes
        [Test]
        public void UpdateWorldStaticAndDynamicBoxesTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(100, 100);
            for (int i = 0; i < 100; ++i)
            {
                BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
                BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            }

            world.CollisionWorld.UpdateDynamicTree(ref world, 1 / 60, -9.81f * math.up());
            TestUtils.DisposeAllColliderBlobs(ref world);
            world.Dispose();
        }

        static void ValidateBodiesAreEqual(Physics.RigidBody body1, Physics.RigidBody body2, bool expectColliderBlobSharing)
        {
            Assert.AreEqual(body1.WorldFromBody, body2.WorldFromBody);
            Assert.AreEqual(body1.Entity, body2.Entity);
            Assert.AreEqual(body1.CustomTags, body2.CustomTags);
            Assert.AreEqual(body1.Scale, body2.Scale);

            if (expectColliderBlobSharing ||
                !body1.Collider.IsCreated) // if one collider is null, we expect the other also to be null, with our without blob sharing
            {
                Assert.AreEqual(body1.Collider, body2.Collider);
            }
            else
            {
                Assert.AreNotEqual(body1.Collider, body2.Collider);
                // deep compare the collider blobs
                ref var collider1 = ref body1.Collider.Value;
                ref var collider2 = ref body2.Collider.Value;
                Assert.AreEqual(collider1, collider2);
            }
        }

        static void ValidateTreesAreEqual(Broadphase.Tree tree1, Broadphase.Tree tree2)
        {
            Assert.IsTrue(tree1.Nodes.AsArray().SequenceEqual(tree2.Nodes.AsArray()));
            Assert.IsTrue(tree1.NodeFilters.AsArray().SequenceEqual(tree2.NodeFilters.AsArray()));
            Assert.IsTrue(tree1.BodyFilters.AsArray().SequenceEqual(tree2.BodyFilters.AsArray()));
            Assert.IsTrue(tree1.RespondsToCollision.AsArray().SequenceEqual(tree2.RespondsToCollision.AsArray()));
        }

        static void ValidateBroadphasesAreEqual(Broadphase broadphase1, Broadphase broadphase2)
        {
            ValidateTreesAreEqual(broadphase1.DynamicTree, broadphase2.DynamicTree);
            ValidateTreesAreEqual(broadphase1.StaticTree, broadphase2.StaticTree);
        }

        static void ValidateCollisionWorldsAreEqual(CollisionWorld world1, CollisionWorld world2,
            bool expectDynamicColliderBlobSharing, bool expectStaticColliderBlobSharing,
            NativeList<int> expectCollidersNotToBeSharedList = default)
        {
            Assert.AreEqual(world1.NumBodies, world2.NumBodies);
            Assert.AreEqual(world1.NumDynamicBodies, world2.NumDynamicBodies);
            Assert.AreEqual(world1.NumStaticBodies, world2.NumStaticBodies);

            var notSharedColliders = new NativeHashSet<int>(expectCollidersNotToBeSharedList.IsEmpty ? 0 : expectCollidersNotToBeSharedList.Length, Allocator.Temp);
            if (!expectCollidersNotToBeSharedList.IsEmpty)
            {
                foreach (var entry in expectCollidersNotToBeSharedList)
                {
                    notSharedColliders.Add(entry);
                }
            }

            // compare dynamic bodies
            for (var i = 0; i < world1.NumDynamicBodies; ++i)
            {
                var expectColliderBlobSharing = expectDynamicColliderBlobSharing && !notSharedColliders.Contains(i);
                ValidateBodiesAreEqual(world1.Bodies[i], world2.Bodies[i], expectColliderBlobSharing);
            }

            // compare static bodies
            for (var i = world1.NumDynamicBodies; i < world1.NumBodies; ++i)
            {
                var expectColliderBlobSharing = expectStaticColliderBlobSharing && !notSharedColliders.Contains(i);
                ValidateBodiesAreEqual(world1.Bodies[i], world2.Bodies[i], expectColliderBlobSharing);
            }

            // compare entity/body map
            for (var i = 0; i < world1.NumBodies; ++i)
            {
                var body1 = world1.Bodies[i];
                var body2 = world2.Bodies[i];

                Assert.AreEqual(body1.Entity, body2.Entity);
                var bodyIndex1 = world1.GetRigidBodyIndex(body1.Entity);
                var bodyIndex2 = world2.GetRigidBodyIndex(body2.Entity);
                Assert.AreEqual(bodyIndex1, bodyIndex2);
                Assert.AreEqual(bodyIndex1, i);
            }

            ValidateBroadphasesAreEqual(world1.Broadphase, world2.Broadphase);
        }

        // Tests cloning of collision world
        [Test]
        public void CloneCollisionWorld()
        {
            var world = BroadPhaseTests.createPopulatedTestWorld(10, 10);
            try
            {
                // Clone collision world.
                // With the chosen function, colliders are shallow copied. That is, the collider blobs
                // will be shared between the rigid bodies in the original and the cloned world.
                var collisionWorldClone = world.CollisionWorld.Clone();
                try
                {
                    // Compare the cloned world with the original world and expect them to be identical.
                    // Note: we expect the collider blobs to be shared since they are only shallow copied.
                    ValidateCollisionWorldsAreEqual(world.CollisionWorld, collisionWorldClone,
                        expectDynamicColliderBlobSharing: true, expectStaticColliderBlobSharing: true);
                }
                finally
                {
                    collisionWorldClone.Dispose();
                }
            }
            finally
            {
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        // Tests cloning of collision world with valid deep copy options
        [Test]
        public void CloneCollisionWorldValidDeepCopyOptions([Values] bool deepCopyDynamicColliders, [Values] bool deepCopyStaticColliders, [Values] bool deepCopySelectColliders)
        {
            var world = BroadPhaseTests.createPopulatedTestWorld(10, 10);

            // deep copy a few select colliders if requested
            NativeList<int> deepCopyColliders = default;
            if (deepCopySelectColliders)
            {
                deepCopyColliders = new NativeList<int>(20, Allocator.Temp);

                // Note: we only deep copy select dynamic or static colliders if this is not already
                // done through the other options.

                if (!deepCopyDynamicColliders)
                {
                    deepCopyColliders.Add(0);
                    deepCopyColliders.Add(5);
                    deepCopyColliders.Add(3);
                    deepCopyColliders.Add(8);
                }

                if (!deepCopyStaticColliders)
                {
                    deepCopyColliders.Add(14);
                    deepCopyColliders.Add(12);
                    deepCopyColliders.Add(18);
                    deepCopyColliders.Add(19);
                }
            }

            try
            {
                // Clone collision world.
                var collisionWorldClone = world.CollisionWorld.Clone(deepCopyDynamicColliders, deepCopyStaticColliders, deepCopyColliders);
                try
                {
                    // Compare the cloned world with the original world and expect them to be identical.
                    // Note: we only expect collider blob sharing if we don't deep copy the corresponding colliders.
                    ValidateCollisionWorldsAreEqual(world.CollisionWorld, collisionWorldClone,
                        expectDynamicColliderBlobSharing: !deepCopyDynamicColliders,
                        expectStaticColliderBlobSharing: !deepCopyStaticColliders,
                        expectCollidersNotToBeSharedList: deepCopyColliders);
                }
                finally
                {
                    collisionWorldClone.Dispose();
                }
            }
            finally
            {
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        // Tests cloning of collision world with deep copy requests for all colliders and colliders being null
        [Test]
        public void CloneCollisionWorldDeepCopyWithNullColliders([Values] bool deepCopyWithFlags)
        {
            // create test world without colliders
            var numBodiesPerType = 2;
            var world = BroadPhaseTests.createPopulatedTestWorld(numBodiesPerType, numBodiesPerType, false);

            NativeList<int> deepCopyColliders = default;
            if (!deepCopyWithFlags)
            {
                // deep copy all bodies using the deep copy list
                deepCopyColliders = new NativeList<int>(4, Allocator.Temp);
                for (int i = 0; i < numBodiesPerType; ++i)
                {
                    deepCopyColliders.Add(i);
                }
            }

            try
            {
                // Clone collision world.
                var collisionWorldClone = world.CollisionWorld.Clone(deepCopyWithFlags, deepCopyWithFlags, deepCopyColliders);
                try
                {
                    // Compare the cloned world with the original world and expect them to be identical.
                    ValidateCollisionWorldsAreEqual(world.CollisionWorld, collisionWorldClone,
                        expectDynamicColliderBlobSharing: !deepCopyWithFlags,
                        expectStaticColliderBlobSharing: !deepCopyWithFlags,
                        expectCollidersNotToBeSharedList: deepCopyColliders);
                }
                finally
                {
                    collisionWorldClone.Dispose();
                }
            }
            finally
            {
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        static readonly TestCaseData[] k_InvalidDeepCopyOptions =
        {
            new TestCaseData(true, false, new[] {2, 5}).SetName("CloneCollisionWorldInvalidDeepCopyOptions: double copy dynamic collider"),
            new TestCaseData(false, true, new[] {12, 18}).SetName("CloneCollisionWorldInvalidDeepCopyOptions: double copy static collider"),
            new TestCaseData(false, false, new[] {-1}).SetName("CloneCollisionWorldInvalidDeepCopyOptions: deep copy collider indices out of bounds (lower)"),
            new TestCaseData(false, false, new[] {20}).SetName("CloneCollisionWorldInvalidDeepCopyOptions: deep copy collider indices out of bounds (upper)"),
            new TestCaseData(false, false, new[] {14, 14, 4, 2, 1, 12, 12, 4, 5, 7, 17}).SetName("CloneCollisionWorldInvalidDeepCopyOptions: deep copy collider indices not a set"),
        };


        [Conditional(CompilationSymbols.CollectionsChecksSymbol), Conditional(CompilationSymbols.DebugChecksSymbol)]
        void ValidateInvalidDeepCopyOptionThrows(bool deepCopyDynamicColliders, bool deepCopyStaticColliders, int[] deepCopyColliderIndices)
        {
            var world = BroadPhaseTests.createPopulatedTestWorld(10, 10);
            var deepCopyColliders = new NativeList<int>(20, Allocator.Temp);
            foreach (var index in deepCopyColliderIndices)
            {
                deepCopyColliders.Add(index);
            }

            try
            {
                // Clone collision world and expect it to throw.
                NUnit.Framework.Assert.Throws<ArgumentException>(() =>
                {
                    world.CollisionWorld.Clone(deepCopyDynamicColliders, deepCopyStaticColliders,
                        deepCopyColliders);
                });
            }
            finally
            {
                TestUtils.DisposeAllColliderBlobs(ref world);
                world.Dispose();
            }
        }

        // Tests cloning of collision world with invalid deep copy options, expecting the cloning function to throw when safety checks are enabled.
        [TestCaseSource(nameof(k_InvalidDeepCopyOptions))]
        public void CloneCollisionWorldInvalidDeepCopyOptions(bool deepCopyDynamicColliders, bool deepCopyStaticColliders, int[] deepCopyColliderIndices)
        // Use an additional indirection here to exclude this test when safety checks are not enabled via the [Conditional] attribute.
        // Note: we can not use the [Conditional] attribute on the test method itself since NUnit calls the test function
        // via reflection and thus does not respect the [Conditional] attribute.
            => ValidateInvalidDeepCopyOptionThrows(deepCopyDynamicColliders, deepCopyStaticColliders, deepCopyColliderIndices);
    }
}
