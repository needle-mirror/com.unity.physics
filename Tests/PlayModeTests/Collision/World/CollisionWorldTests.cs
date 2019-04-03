using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Collision.PhysicsWorld
{
    public class CollisionWorldTests
    {
        //Tests creating a Zero body world
        [Test]
        public void ZeroBodyInitTest()
        {
            CollisionWorld world = new CollisionWorld(0);
            Assert.IsTrue(world.NumBodies == 0);
            world.Dispose();
        }

        //Tests creating a 10 body world
        [Test]
        public void TenBodyInitTest()
        {
            CollisionWorld world = new CollisionWorld(10);
            Assert.IsTrue(world.NumBodies == 10);
            world.Dispose();
        }

        //Tests updating an empty world
        [Test]
        public void SheduleUpdateJobsEmptyWorldTest()
        {
            //Physics.World world = BroadPhaseTests.createTestWorld();
            //Unity.Jobs.JobHandle handle = new Unity.Jobs.JobHandle();
            //Unity.Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateJobs(ref world, 1 / 60, 1, handle);
            //worldJobHandle.Complete();
            //Assert.IsTrue(worldJobHandle.IsCompleted);
            //world.Dispose();
        }

        //Tests updating a static box
        [Test]
        public void SheduleUpdateJobsOneStaticBoxTest()
        {
            Unity.Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(1);
            BroadPhaseTests.addStaticBoxToWorld(world, 0, Vector3.zero, quaternion.identity, new Vector3(10, .1f, 10));
            Unity.Jobs.JobHandle handle = new Unity.Jobs.JobHandle();
            Unity.Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicLayer(ref world, 1 / 60, 1, handle);
            worldJobHandle.Complete();
            Assert.IsTrue(worldJobHandle.IsCompleted);
            world.Dispose();
        }

        //Tests updating 10 static boxes
        [Test]
        public void SheduleUpdateJobsTenStaticBoxesTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(10);
            for (int i = 0; i < 10; ++i)
                BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            Unity.Jobs.JobHandle handle = new Unity.Jobs.JobHandle();
            Unity.Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicLayer(ref world, 1 / 60, 1, handle);
            worldJobHandle.Complete();
            Assert.IsTrue(worldJobHandle.IsCompleted);
            world.Dispose();
        }

        //Tests updating 100 static boxes
        [Test]
        public void SheduleUpdateJobsOneHundredStaticBoxesTest()
        {
            Physics.PhysicsWorld world = BroadPhaseTests.createTestWorld(100);
            for (int i = 0; i < 100; ++i)
                BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            Unity.Jobs.JobHandle handle = new Unity.Jobs.JobHandle();
            Unity.Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicLayer(ref world, 1 / 60, 1, handle);
            worldJobHandle.Complete();
            Assert.IsTrue(worldJobHandle.IsCompleted);
            world.Dispose();
        }

        //Tests updating a Dynamic box
        [Test]
        public void SheduleUpdateJobsOneDynamicBoxTest()
        {
            //Physics.World world = BroadPhaseTests.createTestWorld(0,1);
            //BroadPhaseTests.addDynamicBoxToWorld(world, 0, Vector3.zero, quaternion.identity, new Vector3(10, .1f, 10));
            //Unity.Jobs.JobHandle handle = new Unity.Jobs.JobHandle();
            //Unity.Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateJobs(ref world, 1 / 60, 1, handle);
            //worldJobHandle.Complete();
            //Assert.IsTrue(worldJobHandle.IsCompleted);
            //world.Dispose();
        }

        //Tests updating 10 dynamic boxes
        [Test]
        public void SheduleUpdateJobsTenDynamicBoxesTest()
        {
            //Physics.World world = BroadPhaseTests.createTestWorld(0,10);
            //for (int i = 0; i < 10; ++i)
            //    BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            //Unity.Jobs.JobHandle handle = new Unity.Jobs.JobHandle();
            //Unity.Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateJobs(ref world, 1 / 60, 1, handle);
            //worldJobHandle.Complete();
            //Assert.IsTrue(worldJobHandle.IsCompleted);
            //world.Dispose();
        }

        //Tests updating 100 dynamic boxes
        [Test]
        public void SheduleUpdateJobsOneHundredDynamicBoxesTest()
        {
            //Physics.World world = BroadPhaseTests.createTestWorld(0,100);
            //for (int i = 0; i < 100; ++i)
            //    BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            //Unity.Jobs.JobHandle handle = new Unity.Jobs.JobHandle();
            //Unity.Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateJobs(ref world, 1 / 60, 1, handle);
            //worldJobHandle.Complete();
            //Assert.IsTrue(worldJobHandle.IsCompleted);
            //world.Dispose();
        }

        //Tests updating 100 static and dynamic boxes
        [Test]
        public void SheduleUpdateJobsStaticAndDynamicBoxesTest()
        {
            //Physics.World world = BroadPhaseTests.createTestWorld(100, 100);
            //for (int i = 0; i < 100; ++i)
            //{
            //    BroadPhaseTests.addDynamicBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            //    BroadPhaseTests.addStaticBoxToWorld(world, i, new Vector3(11 * i, 0, 0), quaternion.identity, new Vector3(10, .1f, 10));
            //}
            //Unity.Jobs.JobHandle handle = new Unity.Jobs.JobHandle();
            //Unity.Jobs.JobHandle worldJobHandle = world.CollisionWorld.ScheduleUpdateDynamicLayer(ref world, 1 / 60, 1, handle);
            //worldJobHandle.Complete();
            //Assert.IsTrue(worldJobHandle.IsCompleted);
            //world.Dispose();
        }
    }
}
