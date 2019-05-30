using System;
using System.Linq;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using Unity.Transforms;
using UnityEngine;

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
            {
                // convert GameObject hierarchy
                GameObjectConversionUtility.ConvertGameObjectHierarchy(gameObjectHierarchyRoot, world);

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

        [Test]
        public void ConversionSystems_WhenChildGOHasPhysicsComponents_EntityIsNewRoot(
            [Values(typeof(PhysicsBody), typeof(Rigidbody), null)]
            Type bodyType,
            [Values(typeof(PhysicsShape), typeof(BoxCollider))]
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

            var query = new EntityQueryDesc
            {
                All = new ComponentType[] { typeof(PhysicsCollider) },
                None = new ComponentType[] { typeof(Parent), typeof(PreviousParent) }
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
            [Values(typeof(PhysicsBody), typeof(Rigidbody), null)] Type bodyType,
            [Values(typeof(PhysicsShape), typeof(BoxCollider))] Type colliderType,
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
        public void ConversionSystems_WhenGOHasPhysicsComponents_EntityHasRotationInWorldSpace(
            [Values(typeof(PhysicsBody), typeof(Rigidbody), null)]
            Type bodyType,
            [Values(typeof(PhysicsShape), typeof(BoxCollider))]
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

            var rotation = ConvertHierarchyAndUpdateTransformSystems<Rotation>(Root);

            Assert.That(rotation.Value, Is.EqualTo((quaternion)Child.transform.rotation));
        }

        [Test]
        public void ConversionSystems_WhenGOHasPhysicsComponents_EntityHasTranslationInWorldSpace(
            [Values(typeof(PhysicsBody), typeof(Rigidbody), null)] Type bodyType,
            [Values(typeof(PhysicsShape), typeof(BoxCollider))] Type colliderType,
            [Values(typeof(StaticOptimizeEntity), null)] Type otherType
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                Array.Empty<Type>(),
                new[] { bodyType, colliderType, otherType }.Where(t => t != null).ToArray()
            );
            TransformHierarchyNodes();

            var translation = ConvertHierarchyAndUpdateTransformSystems<Translation>(Root);

            Assert.That(translation.Value, Is.EqualTo((float3)Child.transform.position));
        }
    }
}
