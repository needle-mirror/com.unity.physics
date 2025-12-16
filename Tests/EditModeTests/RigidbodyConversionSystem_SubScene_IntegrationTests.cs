using System;
using System.Collections;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using Unity.Physics.Systems;
using Unity.Scenes;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.TestTools;

namespace Unity.Physics.Tests.Authoring
{
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(PhysicsSystemGroup))]
    internal partial class DeterministicSteppingSystem : SystemBase
    {
        int m_RemainingSimulationStepCount;
        protected override void OnCreate()
        {
            Enabled = false;
        }

        protected override void OnDestroy()
        {
            // make sure we finished the last simulation session
            Assert.IsFalse(m_RemainingSimulationStepCount > 0);
        }

        private void EnableStepping(bool enable)
        {
            var fixedStepSimGroup = World.DefaultGameObjectInjectionWorld.GetExistingSystemManaged<FixedStepSimulationSystemGroup>();
            if (fixedStepSimGroup != null)
            {
                fixedStepSimGroup.Enabled = enable;
            }

            Enabled = enable;
        }

        protected override void OnUpdate()
        {
            if (--m_RemainingSimulationStepCount > 0)
            {
                // still steps to go. Keep stepping.
                return;
            }

            // no more steps to go. Disable stepping.
            EnableStepping(false);
            m_RemainingSimulationStepCount = 0;
        }

        public static void Initialize()
        {
            World.DefaultGameObjectInjectionWorld.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>().Enabled = false;
        }

        public static void Uninitialize()
        {
            World.DefaultGameObjectInjectionWorld.GetOrCreateSystemManaged<FixedStepSimulationSystemGroup>().Enabled = true;
        }

        public void StartStepping(int desiredStepCount)
        {
            Assert.IsFalse(Enabled);
            Assert.IsFalse(IsStepping());

            m_RemainingSimulationStepCount = desiredStepCount;
            EnableStepping(true);
        }

        public bool IsStepping()
        {
            return m_RemainingSimulationStepCount > 0;
        }
    }

    // Conversion system integration tests for rigid bodies
    class RigidbodyConversionSystem_SubScene_IntegrationTests
        : ConversionSystem_SubScene_IntegrationTestsFixture
    {
        private DeterministicSteppingSystem SteppingSystem => World.DefaultGameObjectInjectionWorld?.GetOrCreateSystemManaged<DeterministicSteppingSystem>();

        void CreateSubSceneAndValidate<T>(Action<T> configureSubSceneObject, BodyMotionType expectedMotionType, float expectedMass)
            where T : Component
        {
            CreateAndLoadSubScene(configureSubSceneObject);

            // check result
            var world = World.DefaultGameObjectInjectionWorld;

            using var group = world.EntityManager.CreateEntityQuery(typeof(PhysicsMass));
            using var bodies = group.ToComponentDataArray<PhysicsMass>(Allocator.Temp);
            Assume.That(bodies, Has.Length.EqualTo(1));
            var body = bodies[0];
            Assume.That(body.IsKinematic, Is.EqualTo(expectedMotionType == BodyMotionType.Kinematic));
            if (expectedMotionType == BodyMotionType.Dynamic)
            {
                Assert.IsTrue(math.abs(body.InverseMass - (1 / expectedMass)) < math.EPSILON);
            }
        }

        static IEnumerable GetMotionTypes() => new[] {BodyMotionType.Dynamic, BodyMotionType.Kinematic};

        // Test proper reset of rigid bodies contained in an open sub scene after leaving playmode
        public IEnumerator BasePlayModeAndResetTest<T>(Action<T> bodyCreationAction) where T : Component
        {
            // make sure we are initially in edit mode
            Assert.That(Application.isPlaying, Is.False);

            CreateAndLoadSubScene(bodyCreationAction);

            // wait until sub-scene is loaded by skipping frames
            while (!SceneSystem.IsSceneLoaded(World.DefaultGameObjectInjectionWorld.Unmanaged, SubSceneEntity))
            {
                yield return null;
            }
            Assert.IsNotNull(SubSceneManaged);
            Assert.AreNotEqual(Entity.Null, SubSceneEntity);

            // enable sub-scene for editing
            Scenes.Editor.SubSceneUtility.EditScene(SubSceneManaged);

            // make sure rigid body exists and cache its initial transform
            LocalTransform initialTransform;
            using (var group = World.DefaultGameObjectInjectionWorld.EntityManager.CreateEntityQuery(ComponentType.ReadOnly<LocalTransform>(), ComponentType.ReadOnly<PhysicsMass>()))
            {
                using var bodies = group.ToComponentDataArray<PhysicsMass>(Allocator.Temp);
                Assert.That(bodies, Has.Length.EqualTo(1));
                var body = bodies[0];
                Assert.That(body.IsKinematic, Is.False);
                using var transforms = group.ToComponentDataArray<LocalTransform>(Allocator.Temp);
                Assert.That(transforms, Has.Length.EqualTo(1));
                initialTransform = transforms[0];
            }

            const int kNumStepsPerSession = 20;
            DeterministicSteppingSystem.Initialize();

            // Session 1:
            // enter play mode and simulate a few frames
            yield return new EnterPlayMode();

            while (!Application.isPlaying)
            {
                yield return null;
            }

            Assert.That(SteppingSystem.IsStepping(), Is.False);

            SteppingSystem.StartStepping(kNumStepsPerSession);

            while (SteppingSystem.IsStepping())
            {
                yield return null;
            }
            Assert.That(SteppingSystem.IsStepping(), Is.False);

            // Make sure the body exists and cache its final transformation after simulation
            LocalTransform finalTransform;
            using (var group = World.DefaultGameObjectInjectionWorld.EntityManager.CreateEntityQuery(ComponentType.ReadOnly<LocalTransform>(), ComponentType.ReadOnly<PhysicsMass>()))
            {
                using var transforms = group.ToComponentDataArray<LocalTransform>(Allocator.Temp);
                Assert.That(transforms, Has.Length.EqualTo(1));
                finalTransform = transforms[0];
                // make sure the body moved
                Assert.That(finalTransform, Is.Not.EqualTo(initialTransform));
            }

            // Exit play mode and try playing again, but this time only half the number of steps.
            // Afterwards, confirm that the rigid body restarted from its original location and we approached the same final
            // transformation as in the first play mode session above.
            yield return new ExitPlayMode();

            while (Application.isPlaying)
            {
                yield return null;
            }

            // Session 2:
            // enter play mode and simulate again
            yield return new EnterPlayMode();

            while (!Application.isPlaying)
            {
                yield return null;
            }
            Assert.That(SteppingSystem.IsStepping(), Is.False);

            SteppingSystem.StartStepping(kNumStepsPerSession / 2);
            while (SteppingSystem.IsStepping())
            {
                yield return null;
            }

            // expect the rigid body to have reached the same final transformation
            using (var group = World.DefaultGameObjectInjectionWorld.EntityManager.CreateEntityQuery(ComponentType.ReadOnly<LocalTransform>(), ComponentType.ReadOnly<PhysicsMass>()))
            {
                using var transforms = group.ToComponentDataArray<LocalTransform>(Allocator.Temp);
                Assert.That(transforms, Has.Length.EqualTo(1));
                var intermediateTransform = transforms[0];
                // expect the same orientation since the object should just have fallen under gravity
                Assert.That(intermediateTransform.Rotation, Is.EqualTo(finalTransform.Rotation));
                // expect that the position along gravity (default direction -y) lies somewhere between the initial position and the
                // final position from the first session.
                Assert.That(intermediateTransform.Position.y, Is.LessThan(initialTransform.Position.y));
                Assert.That(intermediateTransform.Position.y, Is.GreaterThan(finalTransform.Position.y));
                // expect the other position components to be unchanged
                Assert.That(new float3(intermediateTransform.Position.x, 0, intermediateTransform.Position.z),
                    Is.PrettyCloseTo(new float3(finalTransform.Position.x, 0, finalTransform.Position.z)));
            }

            DeterministicSteppingSystem.Uninitialize();
        }

        [Test]
        public void RigidbodyConversionSystem_IsInSubScene_DoesNotThrow([ValueSource(nameof(GetMotionTypes))] BodyMotionType motionType)
        {
            float mass = 42f;
            CreateSubSceneAndValidate<Rigidbody>(body =>
            {
                body.mass = mass;
                body.isKinematic = motionType == BodyMotionType.Kinematic;
            }, motionType, mass);
        }

        // Test proper reset of rigid bodies built with Rigidbody components contained in an open sub scene after leaving playmode
        [UnityTest]
        public IEnumerator RigidbodyConversionSystem_IsInSubScene_PlayModeAndReset() => BasePlayModeAndResetTest<Rigidbody>(body =>
        {
            body.transform.position = new Vector3(42f, -4.2f, 0.42f);
            body.transform.rotation = Quaternion.Euler(4.2f, -42f, 42.0f);
            body.mass = 42f;
            body.isKinematic = false;
        });
    }
}
