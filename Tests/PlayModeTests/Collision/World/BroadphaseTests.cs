using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Collision.PhysicsWorld
{
    public class BroadPhaseTests
    {
        /// Util functions
        //Creates a world
        static public Physics.PhysicsWorld createTestWorld(int staticBodies = 0, int dynamicBodies = 0, int joints = 0)
        {
            return new Physics.PhysicsWorld(staticBodies, dynamicBodies, joints);
        }

        //Adds a static box to the world
        static public unsafe void addStaticBoxToWorld(Physics.PhysicsWorld world, int index, Vector3 pos, Quaternion orientation, Vector3 size)
        {
            Assert.IsTrue(index < world.NumStaticBodies, "Static body index is out of range in addStaticBoxToWorld");
            Unity.Collections.NativeSlice<Physics.RigidBody> staticBodies = world.StaticBodies;
            Physics.RigidBody rb = staticBodies[index];
            BlobAssetReference<Physics.Collider> collider = Unity.Physics.BoxCollider.Create(pos, orientation, size, .01f);
            rb.Collider = (Collider*)collider.GetUnsafePtr();
            staticBodies[index] = rb;
        }

        //Adds a dynamic box to the world
        static public unsafe void addDynamicBoxToWorld(Physics.PhysicsWorld world, int index, Vector3 pos, Quaternion orientation, Vector3 size)
        {
            Assert.IsTrue(index < world.NumDynamicBodies, "Dynamic body index is out of range in addDynamicBoxToWorld");
            Unity.Collections.NativeSlice<Physics.RigidBody> dynamicBodies = world.DynamicBodies;
            Physics.RigidBody rb = dynamicBodies[index];
            BlobAssetReference<Physics.Collider> collider = Unity.Physics.BoxCollider.Create(pos, orientation, size, .01f);
            rb.Collider = (Collider*)collider.GetUnsafePtr();
            dynamicBodies[index] = rb;
        }

        /// Tests
        //Tests Broadphase Constructor, Init
        [Test]
        public void InitTest()
        {
            Broadphase bf = new Broadphase();

            bf.Init();

            bf.Dispose();
        }

        //Tests ScheduleBuildJobs with one static box in the world
        [Test]
        public void ScheduleBuildJobsOneStaticBoxTest()
        {
            Physics.PhysicsWorld world = createTestWorld(1);
            addStaticBoxToWorld(world, 0, new Vector3(0, 0, 0), Quaternion.identity, new Vector3(10, 0.1f, 10));
            JobHandle handle = new JobHandle();
            StaticLayerChangeInfo staticLayerChangeInfo = new StaticLayerChangeInfo();
            staticLayerChangeInfo.Init(Allocator.TempJob);
            staticLayerChangeInfo.NumStaticBodies = 1;
            staticLayerChangeInfo.HaveStaticBodiesChanged = 1;
            JobHandle result = world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, 1 / 60, 1, ref staticLayerChangeInfo, handle);
            result.Complete();
            Assert.IsTrue(result.IsCompleted);
            world.Dispose();
            staticLayerChangeInfo.Deallocate();
        }

        //Tests ScheduleBuildJobs with 10 static boxes
        [Test]
        public void ScheduleBuildJobsTenStaticBoxesTest()
        {
            Physics.PhysicsWorld world = createTestWorld(10);
            for (int i = 0; i < 10; ++i)
            {
                addStaticBoxToWorld(world, i, new Vector3(i*11, 0, 0), Quaternion.identity, new Vector3(10, 0.1f, 10));
            }
            JobHandle handle = new JobHandle();
            StaticLayerChangeInfo staticLayerChangeInfo = new StaticLayerChangeInfo();
            staticLayerChangeInfo.Init(Allocator.TempJob);
            staticLayerChangeInfo.NumStaticBodies = 10;
            staticLayerChangeInfo.HaveStaticBodiesChanged = 1;
            JobHandle result = world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, 1 / 60, 1, ref staticLayerChangeInfo, handle);
            result.Complete();
            Assert.IsTrue(result.IsCompleted);
            world.Dispose();
            staticLayerChangeInfo.Deallocate();
        }

        //Tests ScheduleBuildJobs with 100 static boxes
        [Test]
        public void ScheduleBuildJobsOneHundredStaticBoxesTest()
        {
            Physics.PhysicsWorld world = createTestWorld(100);
            for (int i = 0; i < 100; ++i)
            {
                addStaticBoxToWorld(world, i, new Vector3(i*11, 0, 0), Quaternion.identity, new Vector3(10, 0.1f, 10));
            }
            JobHandle handle = new JobHandle();
            StaticLayerChangeInfo staticLayerChangeInfo = new StaticLayerChangeInfo();
            staticLayerChangeInfo.Init(Allocator.TempJob);
            staticLayerChangeInfo.NumStaticBodies = 100;
            staticLayerChangeInfo.HaveStaticBodiesChanged = 1;
            JobHandle result = world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, 1 / 60, 1, ref staticLayerChangeInfo, handle);
            result.Complete();
            Assert.IsTrue(result.IsCompleted);
            world.Dispose();
            staticLayerChangeInfo.Deallocate();
        }

        //Tests ScheduleBuildJobs with one Dynamic box in the world
        [Test]
        public void ScheduleBuildJobsOneDynamicBoxTest()
        {
            //Physics.World world = createTestWorld(0,1);
            //addDynamicBoxToWorld(world, 0, new Vector3(0, 0, 0), Quaternion.identity, new Vector3(10, 10, 10));
            //JobHandle handle = new JobHandle();
            //JobHandle result = world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, 1 / 60, 1, handle);
            //result.Complete();
            //Assert.IsTrue(result.IsCompleted);
            //world.Dispose();
        }

        //Tests ScheduleBuildJobs with 10 Dynamic boxes
        [Test]
        public void ScheduleBuildJobsTenDynamicBoxesTest()
        {
            //Physics.World world = createTestWorld(0,10);
            //for (int i = 0; i < 10; ++i)
            //{
            //    addDynamicBoxToWorld(world, i, new Vector3(i*11, 0, 0), Quaternion.identity, new Vector3(10, 10, 10));
            //}
            //JobHandle handle = new JobHandle();
            //JobHandle result = world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, 1 / 60, 1, handle);
            //result.Complete();
            //Assert.IsTrue(result.IsCompleted);
            //world.Dispose();
        }

        //Tests ScheduleBuildJobs with 100 Dynamic boxes
        [Test]
        public void ScheduleBuildJobsOneHundredDynamicBoxesTest()
        {
            //Physics.World world = createTestWorld(0,100);
            //for (int i = 0; i < 100; ++i)
            //{
            //    addDynamicBoxToWorld(world, i, new Vector3(i*11, 0, 0), Quaternion.identity, new Vector3(10, 10, 10));
            //}
            //JobHandle handle = new JobHandle();
            //JobHandle result = world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, 1 / 60, 1, handle);
            //result.Complete();
            //Assert.IsTrue(result.IsCompleted);
            //world.Dispose();
        }

        [Test]
        public void ScheduleBuildJobsStaticAndDynamicBoxesTest()
        {
            //Physics.World world = createTestWorld(100,100);
            //for (int i = 0; i < 100; ++i)
            //{
            //    addStaticBoxToWorld(world, i, new Vector3(i * 11, 0, 0), Quaternion.identity, new Vector3(10, 0.1f, 10));
            //    addDynamicBoxToWorld(world, i, new Vector3(i * 11, 5, 0), Quaternion.identity, new Vector3(1, 1, 1));
            //}
            //JobHandle handle = new JobHandle();
            //JobHandle result = world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, 1 / 60, 1, handle);
            //result.Complete();
            //Assert.IsTrue(result.IsCompleted);
            //world.Dispose();
        }

        //Tests that ScheduleBuildJobs on an empty world returns a completable JobHandle
        [Test]
        public void ScheduleBuildJobsEmptyWorldTest()
        {
            //Physics.World world = createTestWorld();
            //JobHandle handle = new JobHandle();
            //JobHandle result = world.CollisionWorld.Broadphase.ScheduleBuildJobs(ref world, 1 / 60, 1, handle);
            //result.Complete();
            //world.Dispose();
        }
    }
}
