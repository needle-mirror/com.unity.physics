using System.Runtime.InteropServices;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Base.Containers
{
    public class ElementPoolTests
    {
        // must be blittable, so we cannot use bool as member,
        //see https://docs.microsoft.com/en-us/dotnet/framework/interop/blittable-and-non-blittable-types
        public struct PoolTestElement : IPoolElement
        {
            private int _allocated;

            public int TestIndex { get; set; }

            public bool IsAllocated
            {
                get => _allocated == 0;
                set => _allocated = value ? 1 : 0;
            }

            void IPoolElement.MarkFree(int nextFree)
            {
                IsAllocated = false;
                NextFree = nextFree;
            }

            public int NextFree { get; set; }
        }

        [Test]
        public void CreateEmpty([Values(1, 100, 200)] int count)
        {
            var pool = new ElementPool<PoolTestElement>(count, Allocator.Persistent);

            var numElems = 0;
            foreach (var elem in pool.Elements)
            {
                numElems++;
            }

            Assert.IsTrue(numElems == 0);
            Assert.IsTrue(pool.Capacity == count);
            Assert.IsTrue(pool.GetFirstIndex() == -1);

            pool.Dispose();
        }

        [Test]
        public void InsertAndClear([Values(1, 100, 200)] int count)
        {
            var pool = new ElementPool<PoolTestElement>(count, Allocator.Persistent);


            for (var i = 0; i < count; ++i)
            {
                pool.Allocate(new PoolTestElement{ TestIndex  = i});
                Assert.IsTrue(pool[i].IsAllocated);
            }

            Assert.IsTrue(pool.Capacity == count);

            var numElems = 0;
            foreach (var elem in pool.Elements)
            {
                Assert.IsTrue(pool[numElems].TestIndex == numElems);
                numElems++;
            }

            Assert.IsTrue(numElems == count);
            Assert.IsTrue(pool.PeakCount == count);
            Assert.IsTrue(pool.GetFirstIndex() == 0);
            Assert.IsTrue(pool.GetNextIndex(0) == (count == 1 ? -1 : 1));
            Assert.IsTrue(pool.GetNextIndex(count) == -1);

            pool.Clear();

            numElems = 0;
            foreach (var elem in pool.Elements)
            {
                numElems++;
            }

            Assert.IsTrue(numElems == 0);
            Assert.IsTrue(pool.PeakCount == 0);
            Assert.IsTrue(pool.GetFirstIndex() == -1);

            pool.Dispose();
        }

        [Test]
        public void Copy([Values(1, 100, 200)] int count)
        {
            var pool = new ElementPool<PoolTestElement>(count, Allocator.Persistent);
            var anotherPool = new ElementPool<PoolTestElement>(count, Allocator.Persistent);

            Assert.IsTrue(pool.Capacity == anotherPool.Capacity);

            for (var i = 0; i < count; ++i)
            {
                pool.Allocate(new PoolTestElement { TestIndex = i });
                Assert.IsTrue(pool[i].IsAllocated);
            }

            anotherPool.CopyFrom(pool);

            Assert.IsTrue(pool.PeakCount == anotherPool.PeakCount);
            Assert.IsTrue(pool.GetFirstIndex() == anotherPool.GetFirstIndex());

            for (var i = 0; i < count; ++i)
            {
                Assert.IsTrue(pool[i].TestIndex == i);
                Assert.IsTrue(anotherPool[i].TestIndex==i);
                Assert.IsTrue(anotherPool[i].IsAllocated);
            }

            pool.Dispose();
            anotherPool.Dispose();
        }
    }
}
