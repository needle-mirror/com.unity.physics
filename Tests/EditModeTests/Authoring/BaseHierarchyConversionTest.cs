using System;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Physics.Authoring;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.TestTools;

namespace Unity.Physics.Tests.Authoring
{
    abstract class BaseHierarchyConversionTest
    {
        protected readonly PhysicsWorldIndex k_DefaultWorldIndex = new PhysicsWorldIndex();

        protected static readonly Regex k_NonReadableMeshPattern = new Regex(@"\b((un)?readable|Read\/Write|(non-)?accessible)\b");

        public static readonly TestCaseData[] k_ExplicitPhysicsBodyHierarchyTestCases =
        {
            new TestCaseData(
                BodyMotionType.Static,
                new EntityQueryDesc { All = new ComponentType[] { typeof(PhysicsCollider), typeof(Parent) } }
            ).SetName("Static has parent"),
            new TestCaseData(
                BodyMotionType.Dynamic,
                new EntityQueryDesc { All = new ComponentType[] { typeof(PhysicsCollider) }, None = new ComponentType[] { typeof(Parent), typeof(PreviousParent) } }
            ).SetName("Dynamic is unparented"),
            new TestCaseData(
                BodyMotionType.Kinematic,
                new EntityQueryDesc { All = new ComponentType[] { typeof(PhysicsCollider) }, None = new ComponentType[] { typeof(Parent), typeof(PreviousParent) } }
            ).SetName("Kinematic is unparented"),
        };

        protected static readonly TestCaseData[] k_ExplicitRigidbodyHierarchyTestCases =
        {
            // no means to produce hierarchy of explicit static bodies with legacy
            k_ExplicitPhysicsBodyHierarchyTestCases[1],
            k_ExplicitPhysicsBodyHierarchyTestCases[2]
        };

        protected void CreateHierarchy(
            Type[] rootComponentTypes, Type[] parentComponentTypes, Type[] childComponentTypes
        )
        {
            Root = new GameObject("Root", rootComponentTypes);
            Parent = new GameObject("Parent", parentComponentTypes);
            Child = new GameObject("Child", childComponentTypes);
            Child.transform.parent = Parent.transform;
            Parent.transform.parent = Root.transform;
        }

        protected void CreateHierarchy(
            bool rootStatic,
            Type[] rootComponentTypes, Type[] parentComponentTypes, Type[] childComponentTypes
        )
        {
            Root = new GameObject("Root", rootComponentTypes);
            Parent = new GameObject("Parent", parentComponentTypes);
            Child = new GameObject("Child", childComponentTypes);
            Child.transform.parent = Parent.transform;
            Parent.transform.parent = Root.transform;
            Root.isStatic = rootStatic;
        }

        protected GameObject Root { get; private set; }
        protected GameObject Parent { get; private set; }
        protected GameObject Child { get; private set; }

        internal enum Node { Root, Parent, Child }

        protected GameObject GetNode(Node node)
        {
            switch (node)
            {
                case Node.Root: return Root;
                case Node.Parent: return Parent;
                case Node.Child: return Child;
                default: throw new NotImplementedException($"Unknown node {node}");
            }
        }

        [TearDown]
        public void TearDown()
        {
            if (Child != null)
                GameObject.DestroyImmediate(Child);
            if (Parent != null)
                GameObject.DestroyImmediate(Parent);
            if (Root != null)
                GameObject.DestroyImmediate(Root);
        }

        protected void TestConvertedData<T>(Action<T> checkValue) where T : unmanaged, IComponentData =>
            TestConvertedData<T>((world, entities, components) => { checkValue(components[0]); }, 1);

        protected void TestConvertedData<T>(Action<NativeArray<T>> checkValue, int assumeCount) where T : unmanaged, IComponentData =>
            TestConvertedData<T>((world, entities, components) => { checkValue(components); }, assumeCount);

        public static Entity ConvertBakeGameObject(GameObject go, World world, BlobAssetStore blobAssetStore)
        {
#if UNITY_EDITOR
            // We need to use an intermediate world as BakingUtility.BakeGameObjects cleans up previously baked
            // entities. This means that we need to move the entities from the intermediate world into the final
            // world. As ConvertBakeGameObject returns the main baked entity, we use the EntityGUID to find that
            // entity in the final world
            using var intermediateWorld = new World("BakingWorld");
            BakingUtility.BakeGameObjects(intermediateWorld, new GameObject[] {go}, new BakingSettings(BakingUtility.BakingFlags.AddEntityGUID, blobAssetStore));
            var bakingSystem = intermediateWorld.GetExistingSystemManaged<BakingSystem>();
            var intermediateEntity = bakingSystem.GetEntity(go);
            var intermediateEntityGuid = intermediateWorld.EntityManager.GetComponentData<EntityGuid>(intermediateEntity);
            // Copy the world
            world.EntityManager.MoveEntitiesFrom(intermediateWorld.EntityManager);
            // Search for the entity in the final world by comparing the EntityGuid from entity in the intermediate world
            var query = world.EntityManager.CreateEntityQuery(new ComponentType[] {typeof(EntityGuid)});
            using var entityArray = query.ToEntityArray(Allocator.TempJob);
            using var entityGUIDs = query.ToComponentDataArray<EntityGuid>(Allocator.TempJob);
            for (int index = 0; index < entityGUIDs.Length; ++index)
            {
                if (entityGUIDs[index] == intermediateEntityGuid)
                {
                    return entityArray[index];
                }
            }
            return Entity.Null;
#endif
        }

        protected void TestConvertedData<T>(Action<World, NativeArray<Entity>, NativeArray<T>> checkValues, int assumeCount) where T : unmanaged, IComponentData
        {
            var world = new World("Test world");

            try
            {
                using (var blobAssetStore = new BlobAssetStore(128))
                {
                    ConvertBakeGameObject(Root, world, blobAssetStore);

                    using var group = world.EntityManager.CreateEntityQuery(typeof(T));
                    using var components = group.ToComponentDataArray<T>(Allocator.Temp);
                    using var entities = group.ToEntityArray(Allocator.Temp);
                    Assume.That(components, Has.Length.EqualTo(assumeCount));
                    checkValues(world, entities, components);
                }
            }
            finally
            {
                world.Dispose();
            }
        }

        protected void TestConvertedSharedData<T, S>(S sharedComponent)
            where T : unmanaged, IComponentData
            where S : unmanaged, ISharedComponentData =>
            TestConvertedSharedData<T, S>((world, entities, components) => {}, 1, sharedComponent);

        protected void TestConvertedSharedData<T, S>(Action<T> checkValue, S sharedComponent)
            where T : unmanaged, IComponentData
            where S : unmanaged, ISharedComponentData =>
            TestConvertedSharedData<T, S>((world, entities, components) =>
            {
                checkValue?.Invoke(components[0]);
            }, 1, sharedComponent);

        protected void TestConvertedSharedData<T, S>(Action<World, Entity, T> checkValue, S sharedComponent)
            where T : unmanaged, IComponentData
            where S : unmanaged, ISharedComponentData =>
            TestConvertedSharedData<T, S>((world, entities, components) =>
            {
                checkValue?.Invoke(world, entities[0], components[0]);
            }, 1, sharedComponent);

        protected void TestConvertedSharedData<T, S>(Action<NativeArray<T>> checkValue, int assumeCount, S sharedComponent)
            where T : unmanaged, IComponentData
            where S : unmanaged, ISharedComponentData =>
            TestConvertedSharedData<T, S>((world, entities, components) =>
            {
                checkValue?.Invoke(components);
            }, assumeCount, sharedComponent);

        protected void TestConvertedSharedData<T, S>(Action<World, NativeArray<Entity>, NativeArray<T>> checkValues, int assumeCount, S sharedComponent)
            where T : unmanaged, IComponentData
            where S : unmanaged, ISharedComponentData
        {
            var world = new World("Test world");

            try
            {
                using (var blobAssetStore = new BlobAssetStore(128))
                {
                    ConvertBakeGameObject(Root, world, blobAssetStore);

                    using (var group = world.EntityManager.CreateEntityQuery(new ComponentType[] { typeof(T), typeof(S) }))
                    {
                        group.AddSharedComponentFilter(sharedComponent);
                        using var components = group.ToComponentDataArray<T>(Allocator.Temp);
                        using var entities = group.ToEntityArray(Allocator.Temp);
                        Assume.That(components, Has.Length.EqualTo(assumeCount));
                        checkValues(world, entities, components);
                    }
                }
            }
            finally
            {
                world.Dispose();
            }
        }

        protected void TestMeshData(int numExpectedMeshSections, int[] numExpectedPrimitivesPerSection, bool[][] quadPrimitiveExpectedFlags)
        {
            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(meshCollider =>
            {
                unsafe
                {
                    ref var mesh = ref ((MeshCollider*)meshCollider.ColliderPtr)->Mesh;
                    Assume.That(mesh.Sections.Length, Is.EqualTo(numExpectedMeshSections), $"Expected {numExpectedMeshSections} section(s) on mesh collider.");
                    for (int i = 0; i < numExpectedMeshSections; ++i)
                    {
                        ref var section = ref mesh.Sections[i];
                        var numPrimitives = numExpectedPrimitivesPerSection[i];
                        Assume.That(section.PrimitiveFlags.Length, Is.EqualTo(numPrimitives), $"Expected {numPrimitives} primitive(s) in {i}'th mesh section.");
                        for (int j = 0; j < numPrimitives; ++j)
                        {
                            var isQuad = quadPrimitiveExpectedFlags[i][j];
                            Assert.That(section.PrimitiveFlags[j] & Mesh.PrimitiveFlags.IsQuad, isQuad ? Is.EqualTo(Mesh.PrimitiveFlags.IsQuad) : Is.Not.EqualTo(Mesh.PrimitiveFlags.IsQuad), $"Expected primitive {j} in section {i} to be " + (isQuad ? "a quad" : "not a quad") + " on mesh collider.");
                        }
                    }
                }
            }, k_DefaultWorldIndex);
        }

        protected void VerifyLogsException<T>(Regex message = null) where T : Exception
        {
            var world = new World("Test world");
            try
            {
                using (var blobAssetStore = new BlobAssetStore(128))
                {
                    LogAssert.Expect(LogType.Exception, message ?? new Regex($"\b{typeof(T).Name}\b"));
                    ConvertBakeGameObject(Root, world, blobAssetStore);
                }
            }
            finally
            {
                world.Dispose();
            }
        }

        protected void VerifyNoDataProduced<T>() where T : unmanaged, IComponentData
        {
            var world = new World("Test world");

            try
            {
                using (var blobAssetStore = new BlobAssetStore(128))
                {
                    ConvertBakeGameObject(Root, world, blobAssetStore);

                    using var group = world.EntityManager.CreateEntityQuery(typeof(T));
                    using var components = group.ToComponentDataArray<T>(Allocator.Temp);

                    Assert.That(components.Length, Is.EqualTo(0), $"Conversion pipeline produced {typeof(T).Name}");
                }
            }
            finally
            {
                world.Dispose();
            }
        }
    }
}
