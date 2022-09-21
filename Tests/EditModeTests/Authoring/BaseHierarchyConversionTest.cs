using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Entities.Conversion;
using UnityEngine;
using UnityEngine.TestTools;

namespace Unity.Physics.Tests.Authoring
{
    abstract class BaseHierarchyConversionTest
    {
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
            TestConvertedData((Action<NativeArray<T>>)(components => { checkValue(components[0]); }), 1);

        protected static Entity ConvertBakeGameObject(GameObject go, World world, BlobAssetStore blobAssetStore)
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

        protected void TestConvertedData<T>(Action<NativeArray<T>> checkValues, int assumeCount) where T : unmanaged, IComponentData
        {
            var world = new World("Test world");

            try
            {
                using (var blobAssetStore = new BlobAssetStore(128))
                {
                    ConvertBakeGameObject(Root, world, blobAssetStore);

                    using (var group = world.EntityManager.CreateEntityQuery(typeof(T)))
                    {
                        using (var components = group.ToComponentDataArray<T>(Allocator.Persistent))
                        {
                            Assume.That(components, Has.Length.EqualTo(assumeCount));
                            checkValues(components);
                        }
                    }
                }
            }
            finally
            {
                world.Dispose();
            }
        }

        protected void TestConvertedSharedData<T, S>(Action<T> checkValue, S sharedComponent)
            where T : unmanaged, IComponentData
            where S : unmanaged, ISharedComponentData =>
            TestConvertedSharedData((Action<NativeArray<T>>)(components => { checkValue(components[0]); }), 1, sharedComponent);

        protected void TestConvertedSharedData<T, S>(Action<NativeArray<T>> checkValues, int assumeCount, S sharedComponent)
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
                        using (var components = group.ToComponentDataArray<T>(Allocator.Persistent))
                        {
                            Assume.That(components, Has.Length.EqualTo(assumeCount));
                            checkValues(components);
                        }
                    }
                }
            }
            finally
            {
                world.Dispose();
            }
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

                    using (var group = world.EntityManager.CreateEntityQuery(typeof(T)))
                    using (var bodies = group.ToComponentDataArray<T>(Allocator.Persistent))
                        Assert.That(bodies.Length, Is.EqualTo(0), $"Conversion pipeline produced {typeof(T).Name}");
                }
            }
            finally
            {
                world.Dispose();
            }
        }
    }
}
