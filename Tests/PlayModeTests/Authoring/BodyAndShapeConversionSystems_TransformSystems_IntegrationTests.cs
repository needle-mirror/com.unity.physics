using System;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using Unity.Transforms;
using UnityEngine;
#if LEGACY_PHYSICS
using LegacyBox = UnityEngine.BoxCollider;
using LegacyRigidBody = UnityEngine.Rigidbody;
#endif

namespace Unity.Physics.Tests.Authoring
{
    class BodyAndShapeConversionSystems_TransformSystems_IntegrationTests : BaseHierarchyConversionTest
    {
        static void ConvertHierarchyAndUpdateTransformSystemsVerifyEntityExists(GameObject gameObjectHierarchyRoot, EntityQueryDesc query)
        {
            ConvertHierarchyAndUpdateTransformSystems<LocalToWorld>(gameObjectHierarchyRoot, query, false);
        }

        static T ConvertHierarchyAndUpdateTransformSystems<T>(GameObject gameObjectHierarchyRoot)
            where T: struct, IComponentData
        {
            // query with read/write to trigger update of transform system
            var query = new EntityQueryDesc { All = new[] { typeof(PhysicsCollider), ComponentType.ReadWrite<T>() } };
            return ConvertHierarchyAndUpdateTransformSystems<T>(gameObjectHierarchyRoot, query, true);
        }

        static T ConvertHierarchyAndUpdateTransformSystems<T>(
            GameObject gameObjectHierarchyRoot, EntityQueryDesc query, bool returnData
        )
            where T : struct, IComponentData
        {
            if (
                returnData // i.e. value of post-conversion data will be asserted
                && !query.All.Contains(ComponentType.ReadWrite<T>())
                && !query.Any.Contains(ComponentType.ReadWrite<T>())
            )
                Assert.Fail($"{nameof(query)} must contain {ComponentType.ReadWrite<T>()} in order to trigger update of transform system");

            var queryStr = query.ToReadableString();

            using (var world = new World("Test world"))
            using (var blobAssetStore = new BlobAssetStore())
            {
                // convert GameObject hierarchy
                var settings = GameObjectConversionSettings.FromWorld(world, blobAssetStore);
                GameObjectConversionUtility.ConvertGameObjectHierarchy(gameObjectHierarchyRoot, settings);

                // trigger update of transform systems
                using (var group = world.EntityManager.CreateEntityQuery(query))
                {
                    using (var entities = group.ToEntityArray(Allocator.TempJob))
                    {
                        Assert.That(
                            entities.Length, Is.EqualTo(1),
                            $"Conversion systems produced unexpected number of physics entities {queryStr}"
                        );
                    }

                    var localToWorldSystem = world.GetOrCreateSystem<EndFrameTRSToLocalToWorldSystem>();
                    var lastVersion = localToWorldSystem.GlobalSystemVersion;

                    world.GetOrCreateSystem<EndFrameParentSystem>().Update();
                    world.GetOrCreateSystem<EndFrameCompositeRotationSystem>().Update();
                    world.GetOrCreateSystem<EndFrameCompositeScaleSystem>().Update();
                    world.GetOrCreateSystem<EndFrameParentScaleInverseSystem>().Update();
                    world.GetOrCreateSystem<EndFrameTRSToLocalToWorldSystem>().Update();
                    world.GetOrCreateSystem<EndFrameTRSToLocalToParentSystem>().Update();
                    world.GetOrCreateSystem<EndFrameLocalToParentSystem>().Update();
                    world.GetOrCreateSystem<EndFrameWorldToLocalSystem>().Update();
                    world.EntityManager.CompleteAllJobs();

                    using (var entities = group.ToEntityArray(Allocator.TempJob))
                    {
                        Assume.That(
                            entities.Length, Is.EqualTo(1),
                            $"Updating transform systems resulted in unexpected number of physics entities {queryStr}"
                        );
                    }

                    if (!returnData)
                        return default;

                    using (var chunks = group.CreateArchetypeChunkArray(Allocator.TempJob))
                    {
                        // assume transform systems ran if LocalToWorld chunk version has increased
                        var changed = chunks[0].DidChange(
                            localToWorldSystem.GetArchetypeChunkComponentType<LocalToWorld>(), lastVersion
                        );
                        Assume.That(
                            changed, Is.True,
                            $"Transform systems did not run. Is {typeof(T)} an input for any transform system?"
                        );
                    }

                    using (var data = group.ToComponentDataArray<T>(Allocator.TempJob))
                        return data[0];
                }
            }
        }

        static readonly TestCaseData[] k_ExplicitPhysicsBodyHierarchyTestCases =
        {
            new TestCaseData(
                BodyMotionType.Static,
                new EntityQueryDesc { All = new ComponentType[] { typeof(PhysicsCollider), typeof(Parent) } }
            ).SetName("Static has parent"),
            new TestCaseData(
                BodyMotionType.Dynamic,
                new EntityQueryDesc { All = new ComponentType[] { typeof(PhysicsCollider) }, None = new ComponentType[] { typeof(Parent), typeof(PreviousParent), typeof(LocalToParent) } }
            ).SetName("Dynamic is unparented"),
            new TestCaseData(
                BodyMotionType.Kinematic,
                new EntityQueryDesc { All = new ComponentType[] { typeof(PhysicsCollider) }, None = new ComponentType[] { typeof(Parent), typeof(PreviousParent), typeof(LocalToParent) } }
            ).SetName("Kinematic is unparented"),
        };

        [TestCaseSource(nameof(k_ExplicitPhysicsBodyHierarchyTestCases))]
        public void ConversionSystems_WhenChildGOHasExplicitPhysicsBody_EntityIsInExpectedHierarchyLocation(
            BodyMotionType motionType, EntityQueryDesc expectedQuery
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                Array.Empty<Type>(),
                new[] { typeof(PhysicsBodyAuthoring), typeof(PhysicsShapeAuthoring) }
            );
            Child.GetComponent<PhysicsBodyAuthoring>().MotionType = motionType;

            ConvertHierarchyAndUpdateTransformSystemsVerifyEntityExists(Root, expectedQuery);
        }

#if LEGACY_PHYSICS
        static readonly TestCaseData[] k_ExplicitLegacyRigidBodyHierarchyTestCases =
        {
            // no means to produce hierarchy of explicit static bodies with legacy
            k_ExplicitPhysicsBodyHierarchyTestCases[1],
            k_ExplicitPhysicsBodyHierarchyTestCases[2]
        };

        [TestCaseSource(nameof(k_ExplicitLegacyRigidBodyHierarchyTestCases))]
        public void ConversionSystems_WhenChildGOHasExplicitLegacyRigidBody_EntityIsInExpectedHierarchyLocation(
            BodyMotionType motionType, EntityQueryDesc expectedQuery
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                Array.Empty<Type>(),
                new[] { typeof(LegacyRigidBody), typeof(LegacyBox) }
            );
            Child.GetComponent<LegacyRigidBody>().isKinematic = motionType != BodyMotionType.Dynamic;
            Child.gameObject.isStatic = motionType == BodyMotionType.Static;

            ConvertHierarchyAndUpdateTransformSystemsVerifyEntityExists(Root, expectedQuery);
        }
#endif

        [Test]
        public void ConversionSystems_WhenChildGOHasImplicitStaticBody_EntityIsInHierarchy(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type colliderType
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                Array.Empty<Type>(),
                new[] { colliderType }
            );

            var query = new EntityQueryDesc
            {
                All = new ComponentType[] { typeof(PhysicsCollider), typeof(Parent) }
            };
            ConvertHierarchyAndUpdateTransformSystemsVerifyEntityExists(Root, query);
        }

        void TransformHierarchyNodes()
        {
            Root.transform.localPosition = new Vector3(1f, 2f, 3f);
            Root.transform.localRotation = Quaternion.Euler(30f, 60f, 90f);
            Root.transform.localScale = new Vector3(3f, 5f, 7f);
            Parent.transform.localPosition = new Vector3(2f, 4f, 8f);
            Parent.transform.localRotation = Quaternion.Euler(10f, 20f, 30f);
            Parent.transform.localScale = new Vector3(2f, 4f, 8f);
            Child.transform.localPosition = new Vector3(3f, 6f, 9f);
            Child.transform.localRotation = Quaternion.Euler(15f, 30f, 45f);
            Child.transform.localScale = new Vector3(-1f, 2f, -4f);
        }

        [Test]
        public void ConversionSystems_WhenGOHasPhysicsComponents_EntityHasSameLocalToWorldAsGO(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring), null
            )]
            Type bodyType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type colliderType,
            [Values(typeof(StaticOptimizeEntity), null)] Type otherType
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                Array.Empty<Type>(),
                new[] { bodyType, colliderType, otherType }.Where(t => t != null).ToArray()
            );
            TransformHierarchyNodes();

            var localToWorld = ConvertHierarchyAndUpdateTransformSystems<LocalToWorld>(Root);

            Assert.That(localToWorld.Value, Is.PrettyCloseTo(Child.transform.localToWorldMatrix));
        }

        [Test]
        public void ConversionSystems_WhenGOIsImplicitStaticBody_EntityHasSameLocalToWorldAsGO(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type colliderType
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                Array.Empty<Type>(),
                new[] { colliderType }
            );
            TransformHierarchyNodes();

            var localToWorld = ConvertHierarchyAndUpdateTransformSystems<LocalToWorld>(Root);

            Assert.That(localToWorld.Value, Is.PrettyCloseTo(Child.transform.localToWorldMatrix));
        }

        [Test]
        public void ConversionSystems_WhenGOHasNonStaticBody_EntityHasRotationInWorldSpace(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type bodyType,
            [Values(BodyMotionType.Dynamic, BodyMotionType.Kinematic)]
            BodyMotionType motionType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type colliderType
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                Array.Empty<Type>(),
                new[] { bodyType, colliderType }.Where(t => t != null).ToArray()
            );
            TransformHierarchyNodes();
            SetBodyMotionType(Child.GetComponent(bodyType), motionType);

            var rotation = ConvertHierarchyAndUpdateTransformSystems<Rotation>(Root);

            var expectedRotation = Math.DecomposeRigidBodyOrientation(Child.transform.localToWorldMatrix);
            Assert.That(rotation.Value, Is.EqualTo(expectedRotation));
        }

        static void SetBodyMotionType(Component component, BodyMotionType motionType)
        {
#if LEGACY_PHYSICS
            var rigidBody = component as LegacyRigidBody;
            if (rigidBody != null)
            {
                rigidBody.isKinematic = motionType == BodyMotionType.Kinematic;
                return;
            }
#endif
            var physicsBody = component as PhysicsBodyAuthoring;
            physicsBody.MotionType = motionType;
        }

        [Test]
        public void ConversionSystems_WhenGOHasPhysicsComponents_EntityHasTranslationInWorldSpace(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type bodyType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type colliderType,
            [Values(typeof(StaticOptimizeEntity), null)]
            Type otherType
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                Array.Empty<Type>(),
                new[] { bodyType, colliderType, otherType }.Where(t => t != null).ToArray()
            );
            TransformHierarchyNodes();

            var translation = ConvertHierarchyAndUpdateTransformSystems<Translation>(Root);

            Assert.That(translation.Value, Is.PrettyCloseTo(Child.transform.position));
        }
    }
}
