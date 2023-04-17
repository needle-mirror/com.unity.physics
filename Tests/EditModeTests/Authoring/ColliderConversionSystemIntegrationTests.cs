using System;
using System.Collections.Generic;
using NUnit.Framework;
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
            //if (NonReadableMesh != null)
            //    UnityMesh.DestroyImmediate(NonReadableMesh);
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

        [Test]
        public unsafe void MeshColliderConversionSystem_WhenMultipleShapesShareInputs_CollidersShareTheSameData(
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
                var uniqueColliders = new HashSet<IntPtr>();
                foreach (var c in colliders)
                    uniqueColliders.Add((IntPtr)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(1), $"Expected colliders to reference the same data, but found {numUnique} different colliders.");
            }, 2, k_DefaultWorldIndex);
        }
    }
}
