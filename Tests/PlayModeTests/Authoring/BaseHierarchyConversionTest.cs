using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
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

        protected void TestConvertedData<T>(Action<T> checkValue) where T : struct, IComponentData =>
            TestConvertedData((Action<NativeArray<T>>)(components => { checkValue(components[0]); }), 1);

        protected void TestConvertedData<T>(Action<NativeArray<T>> checkValues, int assumeCount) where T : struct, IComponentData
        {
            var world = new World("Test world");

            try
            {
                using (var blobAssetStore = new BlobAssetStore())
                {
                    var settings = GameObjectConversionSettings.FromWorld(world, blobAssetStore);
                    GameObjectConversionUtility.ConvertGameObjectHierarchy(Root, settings);

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
            where T : struct, IComponentData
            where S : struct, ISharedComponentData =>
            TestConvertedSharedData((Action<NativeArray<T>>)(components => { checkValue(components[0]); }), 1, sharedComponent);

        protected void TestConvertedSharedData<T, S>(Action<NativeArray<T>> checkValues, int assumeCount, S sharedComponent)
            where T : struct, IComponentData
            where S : struct, ISharedComponentData
        {
            var world = new World("Test world");

            try
            {
                using (var blobAssetStore = new BlobAssetStore())
                {
                    var settings = GameObjectConversionSettings.FromWorld(world, blobAssetStore);
                    GameObjectConversionUtility.ConvertGameObjectHierarchy(Root, settings);

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
                using (var blobAssetStore = new BlobAssetStore())
                {
                    LogAssert.Expect(LogType.Exception, message ?? new Regex($"\b{typeof(T).Name}\b"));
                    var settings = GameObjectConversionSettings.FromWorld(world, blobAssetStore);
                    GameObjectConversionUtility.ConvertGameObjectHierarchy(Root, settings);
                }
            }
            finally
            {
                world.Dispose();
            }
        }

        protected void VerifyNoDataProduced<T>() where T : struct, IComponentData
        {
            var world = new World("Test world");

            try
            {
                using (var blobAssetStore = new BlobAssetStore())
                {
                    var settings = GameObjectConversionSettings.FromWorld(world, blobAssetStore);
                    GameObjectConversionUtility.ConvertGameObjectHierarchy(Root, settings);

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
