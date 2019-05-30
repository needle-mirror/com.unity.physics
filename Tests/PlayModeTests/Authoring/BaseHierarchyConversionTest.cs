using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using UnityEngine;

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


        protected void TestConvertedData<T>(Action<T> checkValue) where T : struct, IComponentData
        {
            var world = new World("Test world");

            try
            {
                GameObjectConversionUtility.ConvertGameObjectHierarchy(Root, world);

                using (var group = world.EntityManager.CreateEntityQuery(typeof(T)))
                {
                    using (var bodies = group.ToComponentDataArray<T>(Allocator.Persistent))
                    {
                        Assume.That(bodies, Has.Length.EqualTo(1));
                        var componentData = bodies[0];

                        checkValue(componentData);
                    }
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
                GameObjectConversionUtility.ConvertGameObjectHierarchy(Root, world);

                using (var group = world.EntityManager.CreateEntityQuery(typeof(T)))
                using (var bodies = group.ToComponentDataArray<T>(Allocator.Persistent))
                    Assert.That(bodies.Length, Is.EqualTo(0), $"Conversion pipeline produced {typeof(T).Name}");
            }
            finally
            {
                world.Dispose();
            }
        }
    }
}
