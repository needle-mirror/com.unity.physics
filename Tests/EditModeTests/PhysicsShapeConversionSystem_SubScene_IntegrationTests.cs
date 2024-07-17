using System;
using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Scenes;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;
using Unity.Physics.Extensions;
using Unity.Scenes.Editor;
using Unity.Transforms;
using Object = UnityEngine.Object;

namespace Unity.Physics.Tests.Authoring
{
    // Conversion system integration tests for built-in physics colliders
    class ColliderConversionSystem_SubScene_IntegrationTests
        : ConversionSystem_SubScene_IntegrationTestsFixture
    {
        UnityEngine.Mesh NonReadableMesh { get; set; }

        [OneTimeSetUp]
        protected new void OneTimeSetUp()
        {
            // create non-readable mesh asset
            NonReadableMesh = UnityEngine.Mesh.Instantiate(Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Cylinder.fbx"));
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
        public void ColliderConversionSystem_WhenShapeIsConvexWithNonReadableMesh_IsInSubScene_DoesNotThrow() =>
            CreateSubSceneAndValidate<UnityEngine.MeshCollider>(
                shape =>
                {
                    shape.sharedMesh = NonReadableMesh;
                    shape.convex = true;
                },
                ColliderType.Convex
            );

        [Test]
        public void ColliderConversionSystem_WhenShapeIsMeshWithNonReadableMesh_IsInSubScene_DoesNotThrow() =>
            CreateSubSceneAndValidate<UnityEngine.MeshCollider>(
                shape =>
                {
                    shape.sharedMesh = NonReadableMesh;
                    shape.convex = false;
                },
                ColliderType.Mesh
            );

        void ValidateExpectedBoxCompoundCollider(List<UnityEngine.BoxCollider> expectedBoxes)
        {
            unsafe
            {
                using (var group = World.DefaultGameObjectInjectionWorld.EntityManager.CreateEntityQuery(ComponentType.ReadOnly<PhysicsCollider>(), ComponentType.ReadOnly<LocalTransform>()))
                {
                    using var colliderComponents = group.ToComponentDataArray<PhysicsCollider>(Allocator.Temp);
                    using var localTransforms = group.ToComponentDataArray<LocalTransform>(Allocator.Temp);
                    Assert.That(colliderComponents, Has.Length.EqualTo(1));
                    Assert.That(localTransforms, Has.Length.EqualTo(1));
                    var colliderComponent = colliderComponents[0];
                    var colliderTransform = localTransforms[0];
                    Assert.That(colliderComponent.IsValid, Is.True);
                    Assert.That(colliderComponent.Value.Value.Type, Is.EqualTo(ColliderType.Compound));
                    var compound = colliderComponent.Value.AsPtr<CompoundCollider>();

                    // make sure that all colliders are present
                    Assert.That(compound->Children.Length, Is.EqualTo(expectedBoxes.Count));
                    for (int i = 0; i < compound->Children.Length; ++i)
                    {
                        ref var child = ref compound->Children[i];
                        var collider = child.Collider;
                        Assert.That(collider->Type, Is.EqualTo(ColliderType.Box));
                        var box = (BoxCollider*)collider;
                        var localChildTM = child.CompoundFromChild;
                        // find the expected box collider
                        const float kEps = 0.0001f;
                        var expectedBoxIndex = expectedBoxes.FindIndex(expectedBoxCollider =>
                        {
                            float3 expectedChildPosition = expectedBoxCollider.transform.position;
                            quaternion expectedChildRotation = expectedBoxCollider.transform.rotation;

                            var worldTM = colliderTransform.TransformTransform(new LocalTransform { Position = localChildTM.pos, Rotation = localChildTM.rot });
                            // check position
                            if (math.lengthsq(expectedChildPosition - worldTM.Position) < kEps)
                            {
                                // check orientation
                                var expectedRotMatrix = new float3x3(expectedChildRotation);
                                var actualRotMatrix = new float3x3(worldTM.Rotation);
                                var c0 = math.lengthsq(actualRotMatrix.c0 - expectedRotMatrix.c0);
                                var c1 = math.lengthsq(actualRotMatrix.c1 - expectedRotMatrix.c1);
                                var c2 = math.lengthsq(actualRotMatrix.c2 - expectedRotMatrix.c2);

                                if (c0 < kEps && c1 < kEps && c2 < kEps)
                                {
                                    // check box parameters
                                    return math.lengthsq((float3)expectedBoxCollider.center - box->Center) < kEps &&
                                    math.lengthsq((float3)expectedBoxCollider.size - box->Size) < kEps;
                                }
                            }

                            return false;
                        });

                        Assert.That(expectedBoxIndex, Is.Not.EqualTo(-1));
                    }
                }
            }
        }

        IEnumerator UpdateEditorAndWorld()
        {
            yield return null;

            // Note: Editor doesn't update if it doesn't have focus. So we must explicitly update the world.
            World.DefaultGameObjectInjectionWorld.Update();
        }

        // Tests that incremental baking of compound colliders works when modifications to the children are done that don't
        // trigger re-baking of the children but of the containing compound collider. This includes translating or rotating the children
        // without changing their collider geometry.
        [UnityTest]
        public IEnumerator ColliderConversionSystem_IncrementalCompoundColliderBaking_IsInSubScene()
        {
            // make sure we are in EditMode
            Assume.That(!Application.isPlaying);

            var wasLiveConversionEnabled = LiveConversionEditorSettings.LiveConversionEnabled;
            try
            {
                // enable live conversion for incremental baking
                LiveConversionEditorSettings.LiveConversionEnabled = true;

                var expectedBoxes = new List<UnityEngine.BoxCollider>();

                CreateAndLoadSubScene(() =>
                {
                    var parent = new GameObject("Parent");
                    var child = new GameObject("Child");
                    child.transform.parent = parent.transform;

                    parent.AddComponent<UnityEngine.BoxCollider>();
                    child.AddComponent<UnityEngine.BoxCollider>();

                    child.transform.localPosition = Vector3.one;
                });

                // wait until sub-scene is loaded by skipping frames
                while (!SceneSystem.IsSceneLoaded(World.DefaultGameObjectInjectionWorld.Unmanaged, SubSceneEntity))
                {
                    yield return null;
                }

                Assume.That(SubSceneManaged, Is.Not.Null);

                // enable sub-scene for editing
                SubSceneUtility.EditScene(SubSceneManaged);

                expectedBoxes.InsertRange(0,
                    Object.FindObjectsByType<UnityEngine.BoxCollider>(FindObjectsSortMode.None));

                yield return UpdateEditorAndWorld();

                ValidateExpectedBoxCompoundCollider(expectedBoxes);

                // make a modification to each colliders' transform to trigger incremental compound collider re-baking and validate again
                var boxColliders = Object.FindObjectsByType<UnityEngine.BoxCollider>(FindObjectsSortMode.None);
                Assert.That(boxColliders.Length, Is.EqualTo(expectedBoxes.Count));
                foreach (var box in boxColliders)
                {
                    // make sure expected box is present
                    Assert.That(expectedBoxes.Contains(box), Is.True);

                    // Use Undo system for the modifications since incremental baking is using it to track changes
                    Undo.RecordObject(box.gameObject.transform, "Modify local position");
                    box.gameObject.transform.localPosition += Vector3.one;

                    // Note: it takes an extra frame to establish that something has changed when using RecordObject unless Flush is called
                    Undo.FlushUndoRecordObjects();

                    yield return UpdateEditorAndWorld();

                    ValidateExpectedBoxCompoundCollider(expectedBoxes);
                }
            }
            finally
            {
                LiveConversionEditorSettings.LiveConversionEnabled = wasLiveConversionEnabled;
            }
        }
    }
}
