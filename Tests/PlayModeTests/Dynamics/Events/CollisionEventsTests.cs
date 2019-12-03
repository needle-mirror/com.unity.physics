using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Physics.Tests.Dynamics.CollisionEvents
{
    class CollisionEventsTests
    {
        // Tests the situation where none of the contact points
        // reaches the contact plane after sub-integration using
        // input velocities. This happens due to other bodies actually
        // causing the collision, not the input velocities.
        // In that case, we want to return the closest point,
        // and we verify here that only 1 point is returned.
        [Test]
        public void CalculateDetailsTest()
        {
            // Simple collision event with straight normal and 3 contact points
            LowLevel.CollisionEvent lowLevelEvent = new LowLevel.CollisionEvent();
            lowLevelEvent.BodyIndices.BodyAIndex = 0;
            lowLevelEvent.BodyIndices.BodyBIndex = 1;
            lowLevelEvent.ColliderKeys.ColliderKeyA = ColliderKey.Empty;
            lowLevelEvent.ColliderKeys.ColliderKeyB = ColliderKey.Empty;
            lowLevelEvent.Normal = new float3(0.0f, -1.00000f, 0.0f);
            lowLevelEvent.NumNarrowPhaseContactPoints = 3;
            lowLevelEvent.SolverImpulse = 1.0f;

            // Wrapping collision event
            CollisionEvent collisionEvent = new CollisionEvent();
            collisionEvent.EventData = lowLevelEvent;
            collisionEvent.TimeStep = 1.0f / 60.0f;

            // Input velocity is obviously separating, but high angular velocity still caused an event
            collisionEvent.InputVelocityA = new Velocity
            {
                Angular = new float3(-0.00064f, 11.17604f, 0.02133f),
                Linear = new float3(-3.81205f, -0.56607f, 9.14945f)
            };
            collisionEvent.InputVelocityB = new Velocity
            {
                Angular = new float3(0.00000f, 0.00000f, 0.00000f),
                Linear = new float3(0.00000f, 0.00000f, 0.00000f)
            };

            // Initialize 3 contact points
            collisionEvent.NarrowPhaseContactPoints = new NativeArray<ContactPoint>(3, Allocator.Temp)
            {
                [0] = new ContactPoint
                {
                    Distance = 0.177905f,
                    Position = new float3(-22.744950f, 2.585318f, -50.108990f)
                },
                [1] = new ContactPoint
                {
                    Distance = 0.276652f,
                    Position = new float3(-20.731140f, 2.486506f, -50.322240f)
                },
                [2] = new ContactPoint
                {
                    Distance = 0.278534f,
                    Position = new float3(-20.766140f, 2.484623f, -50.652630f)
                }
            };

            // Allocate a simple world of 1 dynamic and 1 static body
            var simpleWorld = new Physics.PhysicsWorld(1, 1, 0);
            var motionVelocities = simpleWorld.MotionVelocities;
            var motionDatas = simpleWorld.MotionDatas;
            motionDatas[0] = new MotionData
            {
                LinearDamping = 0.0f,
                AngularDamping = 0.0f,
                GravityFactor = 1.0f,
                BodyFromMotion = new RigidTransform(new quaternion(0.0f, 0.0f, 0.0f, 1.0f), new float3(0.0f, 0.0f, 0.0f)),
                WorldFromMotion = new RigidTransform(new quaternion(0.09212853f, 0.1400256f, -0.006776567f, -0.9858292f), new float3(-22.17587f, 0.5172966f, -52.24425f))
            };
            motionVelocities[0] = new MotionVelocity
            {
                LinearVelocity = new float3(-3.81221f, -1.37538f, -15.41893f),
                AngularVelocity = new float3(-7.30913f, -4.78899f, 1.14168f),
                InverseInertiaAndMass = new float4(0.00045f, 0.00045f, 0.00045f, 0.00018f),
                AngularExpansionFactor = 2.05061f
            };

            // Calculate the collision event details and make sure 1 contact point is returned
            var details = collisionEvent.CalculateDetails(ref simpleWorld);
            Assert.AreEqual(details.EstimatedContactPointPositions.Length, 1);

            // Dispose the world data
            simpleWorld.Dispose();
        }
    }
}
