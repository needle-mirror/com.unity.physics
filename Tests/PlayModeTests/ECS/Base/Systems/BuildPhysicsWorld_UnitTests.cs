using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace Unity.Physics.Tests.Systems
{
    class BuildPhysicsWorld_UnitTests
    {
        [Test]
        public void OnUpdate_OneSingleJoint()
        {
            using (var world = new World("Test world"))
            {
                // Create the system
                var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();
                ref var bpwData = ref world.EntityManager.GetComponentDataRW<BuildPhysicsWorldData>(buildPhysicsWorld).ValueRW;

                // Create a body to attach the joint to
                var bodyEntity = world.EntityManager.CreateEntity(typeof(PhysicsCollider), typeof(LocalTransform), typeof(PhysicsWorldIndex), typeof(PhysicsVelocity));

                // Create joint entity
                Entity jointEntity = world.EntityManager.CreateEntity(typeof(PhysicsJoint), typeof(PhysicsConstrainedBodyPair));
                world.EntityManager.AddComponentData(jointEntity, PhysicsJoint.CreateFixed(new RigidTransform(quaternion.identity, float3.zero), new RigidTransform(quaternion.identity, float3.zero)));
                world.EntityManager.AddComponentData(jointEntity, new PhysicsConstrainedBodyPair(Entity.Null, bodyEntity, false));
                world.EntityManager.AddSharedComponent(jointEntity, new PhysicsWorldIndex());

                // Trigger system update
                buildPhysicsWorld.Update(world.Unmanaged);
                var jobHandle = world.Unmanaged.ResolveSystemStateRef(buildPhysicsWorld).Dependency;
                jobHandle.Complete();
                Assert.IsTrue(jobHandle.IsCompleted);

                // Verify results:
                // expect 2 bodies (default static body, plus the extra body we created above) and the above created joint.
                // Note: we always expect static bodies to have changed due to the default static body being created in the
                // first frame.
                VerifySystemData(ref bpwData, 2, 1, 1);
            }
        }

        [Test]
        public void OnUpdate_NoBodiesNoJoints()
        {
            using (var world = new World("Test world"))
            {
                // Create the system
                var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();
                ref var bpwData = ref world.EntityManager.GetComponentDataRW<BuildPhysicsWorldData>(buildPhysicsWorld).ValueRW;

                // Trigger system update
                buildPhysicsWorld.Update(world.Unmanaged);
                var jobHandle = world.Unmanaged.ResolveSystemStateRef(buildPhysicsWorld).Dependency;
                jobHandle.Complete();
                Assert.IsTrue(jobHandle.IsCompleted);

                // Verify results:
                // expect 1 body (default static body) and no joints.
                // Note: we always expect static bodies to have changed due to the default static body being created in the
                // first frame.
                VerifySystemData(ref bpwData, 1, 0, 1);
            }
        }

        [Test]
        public void ModifyCollisionTolerance()
        {
            using (var world = new World("Test world"))
            {
                // create the build physics world system
                var buildPhysicsWorld = world.GetOrCreateSystem<BuildPhysicsWorld>();
                ref var bpwData = ref world.EntityManager.GetComponentDataRW<BuildPhysicsWorldData>(buildPhysicsWorld)
                    .ValueRW;

                // add physics step component and set collision tolerance
                var physicsStepEntity = world.EntityManager.CreateSingleton<PhysicsStep>();
                var collisionTolerance = 0.42f;
                Assert.That(collisionTolerance, Is.Not.EqualTo(CollisionWorld.DefaultCollisionTolerance));
                world.EntityManager.SetComponentData(physicsStepEntity, new PhysicsStep { CollisionTolerance = collisionTolerance});

                buildPhysicsWorld.Update(world.Unmanaged);
                var jobHandle = world.Unmanaged.ResolveSystemStateRef(buildPhysicsWorld).Dependency;
                jobHandle.Complete();
                Assert.IsTrue(jobHandle.IsCompleted);

                // confirm that the collision tolerance was set in the collision world
                var collisionWorld = bpwData.PhysicsData.PhysicsWorld.CollisionWorld;
                Assert.That(collisionWorld.CollisionTolerance, Is.EqualTo(collisionTolerance));
            }
        }

        public static void VerifySystemData(ref BuildPhysicsWorldData bpwData, int expectedNumBodies, int expectedNumJoints, int expectedStaticBodiesChanged)
        {
            Assert.AreEqual(expectedNumBodies, bpwData.PhysicsData.PhysicsWorld.NumBodies);
            Assert.AreEqual(expectedNumJoints, bpwData.PhysicsData.PhysicsWorld.NumJoints);
            Assert.AreEqual(expectedStaticBodiesChanged, bpwData.HaveStaticBodiesChanged.Value);
        }
    }
}
