using System;
using System.IO;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Physics.Authoring;
using Unity.Scenes;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.LowLevel;
using UnityEngine.SceneManagement;
using UnityMesh = UnityEngine.Mesh;
#if LEGACY_PHYSICS
using LegacyMeshCollider = UnityEngine.MeshCollider;
#endif

namespace Unity.Physics.Tests.Authoring
{
    // Conversion system integration tests for physics shapes, including legacy physics
    class PhysicsShapeConversionSystem_SubScene_IntegrationTests
        : ConversionSystem_SubScene_IntegrationTestsFixture
    {
        UnityMesh NonReadableMesh { get; set; }

        [OneTimeSetUp]
        protected new void OneTimeSetUp()
        {
            base.OneTimeSetUp();

            // create non-readable mesh asset
            NonReadableMesh = UnityMesh.Instantiate(Resources.GetBuiltinResource<UnityMesh>("New-Cylinder.fbx"));
            NonReadableMesh.UploadMeshData(true);
            Assume.That(NonReadableMesh.isReadable, Is.False, $"{NonReadableMesh} was readable.");
            AssetDatabase.CreateAsset(NonReadableMesh, $"{TemporaryAssetsPath}/NonReadableMesh.asset");
        }

        void CreateSubSceneAndValidate<T>(Action<T> configureSubSceneObject, ColliderType expectedColliderType)
            where T : Component
        {
            CreateAndLoadSubScene(configureSubSceneObject);

            // check result
            var world = World.DefaultGameObjectInjectionWorld;
            using (var group = world.EntityManager.CreateEntityQuery(typeof(PhysicsCollider)))
            using (var bodies = group.ToComponentDataArray<PhysicsCollider>(Allocator.Persistent))
            {
                Assume.That(bodies, Has.Length.EqualTo(1));
                Assume.That(bodies[0].IsValid, Is.True);
                Assert.That(bodies[0].Value.Value.Type, Is.EqualTo(expectedColliderType));
            }
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenShapeIsConvexWithNonReadableMesh_IsInSubScene_DoesNotThrow() =>
            CreateSubSceneAndValidate<PhysicsShapeAuthoring>(shape => shape.SetConvexHull(default, NonReadableMesh), ColliderType.Convex);

        [Test]
        public void PhysicsShapeConversionSystem_WhenShapeIsMeshWithNonReadableMesh_IsInSubScene_DoesNotThrow() =>
            CreateSubSceneAndValidate<PhysicsShapeAuthoring>(shape => shape.SetMesh(NonReadableMesh), ColliderType.Mesh);

#if LEGACY_PHYSICS
        [Test]
        public void LegacyShapeConversionSystem_WhenShapeIsConvexWithNonReadableMesh_IsInSubScene_DoesNotThrow() =>
            CreateSubSceneAndValidate<LegacyMeshCollider>(
                shape =>
                {
                    shape.sharedMesh = NonReadableMesh;
                    shape.convex = true;
                },
                ColliderType.Convex
            );

        [Test]
        public void LegacyShapeConversionSystem_WhenShapeIsMeshWithNonReadableMesh_IsInSubScene_DoesNotThrow() =>
            CreateSubSceneAndValidate<LegacyMeshCollider>(
                shape =>
                {
                    shape.sharedMesh = NonReadableMesh;
                    shape.convex = false;
                },
                ColliderType.Mesh
            );
#endif
    }
}
