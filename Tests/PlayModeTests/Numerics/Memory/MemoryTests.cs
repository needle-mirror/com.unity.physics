using Unity.Numerics.Linear.Dense.Primitives;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using System;
using NUnit.Framework;
using Unity.Numerics.Memory;

namespace Unity.Numerics.Tests
{
    public unsafe partial class TestMemory
    {
        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct ConcurrentAllocFree : IJobFor
        {
            public MemoryManager heap;
            public int maxAllocSize;
            public int numAllocs;

            public void Execute(int index)
            {
                using var allocs = new NativeList<IntPtr>(numAllocs, Allocator.Temp);
                var r = new Random.Random((uint)index + 1);

                for (int i = 0; i < numAllocs; i++)
                {
                    if (allocs.IsEmpty || r.NextGaussian() > 0)
                    {
                        // allocate
                        var ptr = heap.Allocate<char>(r.NextUniformInt(8, maxAllocSize + 1));
                        allocs.Add((IntPtr)ptr);
                    }
                    else
                    {
                        // free
                        var idx = r.NextUniformInt(0, allocs.Length);
                        var ptr = (char*)allocs[idx];
                        heap.Free(ptr);
                        allocs.RemoveAt(idx);
                    }
                }

                for (int i = 0; i < allocs.Length; ++i)
                {
                    var ptr = (char*)allocs[i];
                    heap.Free(ptr);
                }
            }
        }

        [Test]
        [Ignore("MemoryManager is not thread safe")]
        public void TestMemoryManager_ConcurrentAllocFree()
        {
            int numAllocs = 256;
            int maxAllocSize = 512;
            int numAllocBatches = 16;
            Assert.DoesNotThrow(
                () =>
                {
                    using var heap = MemoryManager.Create(numAllocBatches * numAllocs * maxAllocSize, Allocator.Temp);
                    var job = new ConcurrentAllocFree()
                    {
                        heap = heap,
                        numAllocs = numAllocs,
                        maxAllocSize = maxAllocSize
                    };
                    job.ScheduleParallel(numAllocBatches, 1, default).Complete();

                    Assert.IsTrue(heap.info->activeAllocations == 0);
                });
        }

        [BurstCompile(CompileSynchronously = true)]
        [GenerateTestsForBurstCompatibility]
        struct ConcurrentAlloc : IJobFor
        {
            public MemoryManager heap;
            public int numAllocs;
            public int maxAllocSize;

            public void Execute(int index)
            {
                //UnityEngine.Debug.Log($"Execute({index}): " + string.Format("heapStart=0x{0,8:X}", (long)heap.info));
                using var allocs = new NativeList<IntPtr>(numAllocs, Allocator.Temp);
                var r = new Random.Random((uint)index + 1);

                for (int i = 0; i < numAllocs; i++)
                {
                    // allocate
                    var size = r.NextUniformInt(8, maxAllocSize + 1);
                    //UnityEngine.Debug.Log($"allocating {size} on thread {index}");
                    var ptr = heap.Allocate<char>(size);
                    allocs.Add((IntPtr)ptr);
                }

                for (int i = 0; i < allocs.Length; ++i)
                {
                    var ptr = (char*)allocs[i];
                    heap.Free(ptr);
                }
            }
        }

        [Test]
        [Ignore("MemoryManager is not thread safe")]
        public void TestMemoryManager_ConcurrentAlloc()
        {
            int numAllocs = 1024;
            int maxAllocSize = 128;
            int numAllocBatches = 16;
            Assert.DoesNotThrow(
                () =>
                {
                    using var heap = MemoryManager.Create(numAllocBatches * numAllocs * maxAllocSize, Allocator.Temp);
                    var job = new ConcurrentAlloc
                    {
                        heap = heap,
                        numAllocs = numAllocs,
                        maxAllocSize = maxAllocSize
                    };
                    job.ScheduleParallel(numAllocBatches, 1, default).Complete();

                    Assert.IsTrue(heap.info->activeAllocations == 0);
                });
        }
    }
}
