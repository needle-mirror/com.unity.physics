using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Physics.Authoring;
using Unity.Physics.Extensions;
using UnityEngine;

namespace Unity.Physics.Tests.Authoring
{
    class ColliderConversionSystemIntegrationTests : BaseHierarchyConversionTest
    {
        private UnityEngine.Mesh NonReadableMesh { get; set; }
        private UnityEngine.Mesh ReadableMesh { get; set; }

        [OneTimeSetUp]
        public void OneTimeSetUp()
        {
            ReadableMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Cylinder.fbx");
            Assume.That(ReadableMesh.isReadable, Is.True, $"{ReadableMesh} was not readable.");

            NonReadableMesh = UnityEngine.Mesh.Instantiate(ReadableMesh);
            NonReadableMesh.UploadMeshData(true);
            Assume.That(NonReadableMesh.isReadable, Is.False, $"{NonReadableMesh} was readable.");
        }

        [OneTimeTearDown]
        public void OneTimeTearDown()
        {
#if !UNITY_EDITOR
            UnityEngine.Mesh.Destroy(NonReadableMesh);
#else
            UnityEngine.Mesh.DestroyImmediate(NonReadableMesh);
#endif
        }

        [Test]
        [Ignore("Behavior is inconsistent on some platforms.")]
        public void MeshColliderConversionSystem_WhenMeshColliderHasNonReadableMesh_ThrowsException(
            [Values] bool convex
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(UnityEngine.MeshCollider) });
#if !UNITY_EDITOR
            // legacy components log error messages in the player for non-readable meshes once for each property access
            LogAssert.Expect(LogType.Error, k_NonReadableMeshPattern);
            LogAssert.Expect(LogType.Error, k_NonReadableMeshPattern);
#endif
            Child.GetComponent<UnityEngine.MeshCollider>().sharedMesh = NonReadableMesh;
            Child.GetComponent<UnityEngine.MeshCollider>().convex = convex;

            VerifyLogsException<InvalidOperationException>(k_NonReadableMeshPattern);
        }

        private unsafe void ValidateUniqueColliderCount(NativeArray<PhysicsCollider> colliders, int expectedCount)
        {
            var uniqueColliders = new HashSet<IntPtr>();
            foreach (var c in colliders)
            {
                uniqueColliders.Add((IntPtr)c.ColliderPtr);
            }

            var numUnique = uniqueColliders.Count;
            Assume.That(numUnique, Is.EqualTo(expectedCount), $"Expected {expectedCount} unique collider(s), but found {numUnique} unique collider(s).");
        }

        [Test]
        public void MeshColliderConversionSystem_WhenMultipleShapesShareInputs_CollidersShareTheSameData(
            [Values] bool convex
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                new[] { typeof(UnityEngine.MeshCollider), typeof(Rigidbody) },
                new[] { typeof(UnityEngine.MeshCollider), typeof(Rigidbody) }
            );
            foreach (var shape in Root.GetComponentsInChildren<UnityEngine.MeshCollider>())
            {
                shape.convex = convex;
                shape.sharedMesh = ReadableMesh;
            }
            Child.transform.localPosition = TransformConversionUtils.k_SharedDataChildTransformation.pos;
            Child.transform.localRotation = TransformConversionUtils.k_SharedDataChildTransformation.rot;

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(colliders =>
            {
                // ensure that the colliders indicate that they are shared.
                foreach (var collider in colliders)
                {
                    Assume.That(collider.IsUnique, Is.False);
                }

                // ensure that the collider are effectively shared.
                ValidateUniqueColliderCount(colliders, 1);
            }, 2, k_DefaultWorldIndex);
        }

        public enum MakeUniqueMode
        {
            EntityManager,
            EntityCommandBuffer,
            ParallelWriter
        }

        [Test]
        // Test which ensures that a baked collider is initially shared but can be made unique
        public void ColliderConversionSystem_SharedColliders_MadeUnique([Values] MakeUniqueMode makeUniqueMode)
        {
            // Create two separate rigid bodies with a shared collider.
            // Sharing occurs automatically because the colliders have identical parameters.
            CreateHierarchy(
                Array.Empty<Type>(),
                new[] {typeof(UnityEngine.SphereCollider), typeof(Rigidbody)},
                new[] {typeof(UnityEngine.SphereCollider), typeof(Rigidbody)}
            );

            TestConvertedData<PhysicsCollider>((world, entities, colliders)  =>
            {
                // ensure that the colliders indicate that they are shared.
                foreach (var collider in colliders)
                {
                    Assume.That(collider.IsUnique, Is.False);
                }

                // ensure that the collider are initially shared.
                ValidateUniqueColliderCount(colliders, 1);

                // make one of the two colliders unique
                var c = colliders[0];
                switch (makeUniqueMode)
                {
                    case MakeUniqueMode.EntityManager:
                    {
                        c.MakeUnique(entities[0], world.EntityManager);
                        break;
                    }
                    case MakeUniqueMode.EntityCommandBuffer:
                    {
                        using var ecb = new EntityCommandBuffer(Allocator.Temp);
                        c.MakeUnique(entities[0], ecb);
                        ecb.Playback(world.EntityManager);
                        break;
                    }
                    case MakeUniqueMode.ParallelWriter:
                    {
                        using var ecb = new EntityCommandBuffer(Allocator.TempJob);
                        {
                            c.MakeUnique(entities[0], ecb.AsParallelWriter(), 0);
                            ecb.Playback(world.EntityManager);
                        }
                        break;
                    }
                }
                colliders[0] = c;

                // ensure that the collider is now unique.
                Assert.That(c.IsUnique, Is.True);
                ValidateUniqueColliderCount(colliders, 2);
            }, 2);
        }

        private static readonly TestCaseData[] k_ColliderTypeTestCases =
        {
            new TestCaseData(
                new[] { typeof(UnityEngine.SphereCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // parent components
                new[] { typeof(UnityEngine.SphereCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // child components
                2, // expect unique count
                false // is convex (unused)
            ).SetName("Unique Sphere Collider"),
            new TestCaseData(
                new[] { typeof(UnityEngine.CapsuleCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // parent components
                new[] { typeof(UnityEngine.CapsuleCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // child components
                2, // expect unique count
                false // is convex (unused)
            ).SetName("Unique Capsule Collider"),
            new TestCaseData(
                new[] { typeof(UnityEngine.BoxCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // parent components
                new[] { typeof(UnityEngine.BoxCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // child components
                2, // expect unique count
                false // is convex (unused)
            ).SetName("Unique Box Collider"),
            new TestCaseData(
                new[] { typeof(UnityEngine.MeshCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // parent components
                new[] { typeof(UnityEngine.MeshCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // child components
                2, // expect unique count
                false // is convex
            ).SetName("Unique Mesh Collider (non-convex)"),
            new TestCaseData(
                new[] { typeof(UnityEngine.MeshCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // parent components
                new[] { typeof(UnityEngine.MeshCollider), typeof(Rigidbody), typeof(ForceUniqueColliderAuthoring)}, // child components
                2, // expect unique count
                true // is convex
            ).SetName("Unique Mesh Collider (convex)"),
        };

        // Test which ensures that a baked built-in collider is unique when the ForceUniqueColliderAuthoring component is present
        [TestCaseSource(nameof(k_ColliderTypeTestCases))]
        public void ColliderConversionSystem_BuiltInSharedColliders_MadeUnique(Type[] parentComponentTypes,
            Type[] childComponentTypes, int expectedCount, bool isConvex)
        {
            // Create two separate rigid bodies with the same collider type but with the force unique component
            CreateHierarchy(Array.Empty<Type>(), parentComponentTypes, childComponentTypes);

            // Must ensure that the mesh isn't null:
            foreach (var parent in parentComponentTypes)
            {
                if (parent == typeof(UnityEngine.MeshCollider))
                {
                    Parent.GetComponent<UnityEngine.MeshCollider>().sharedMesh = ReadableMesh;
                    Parent.GetComponent<UnityEngine.MeshCollider>().convex = isConvex;
                }
            }
            foreach (var child in childComponentTypes)
            {
                if (child == typeof(UnityEngine.MeshCollider))
                {
                    Child.GetComponent<UnityEngine.MeshCollider>().sharedMesh = ReadableMesh;
                    Child.GetComponent<UnityEngine.MeshCollider>().convex = isConvex;
                }
            }

            TestConvertedData<PhysicsCollider>((world, entities, colliders)  =>
            {
                int numUnique = 0;
                foreach (var collider in colliders)
                {
                    if (collider.IsUnique) numUnique++;
                    Assume.That(collider.IsUnique, Is.True);
                }

                Assume.That(numUnique, Is.EqualTo(expectedCount), $"Expected {expectedCount} unique collider(s), but found {numUnique} unique collider(s).");
            }, 2);
        }
    }
}
