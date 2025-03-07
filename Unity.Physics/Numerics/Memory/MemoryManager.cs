#define TRACKED_ALLOCATIONS
#define USE_MALLOC
#define REPORT_STATS

using System;
using System.Collections.Generic;
using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Numerics.Memory
{
    [GenerateTestsForBurstCompatibility]
    [BurstCompile]
    internal unsafe partial struct MemoryManager : IDisposable
    {
        public MemoryManager(long size, Collections.Allocator allocator)
        {
#if !USE_MALLOC
            var obj = new Allocator(size, allocator);
            info = (Allocator*)UnsafeUtility.Malloc(UnsafeUtility.SizeOf<Allocator>(), 1, allocator);
            UnsafeUtility.CopyStructureToPtr(ref obj, info);
#else
            info = null;
            this.allocator = allocator;
#endif
            Valid = true;
        }

        public static MemoryManager Create(long size, Collections.Allocator alloc)
        {
            return new MemoryManager(size, alloc);
        }

        [return : NoAlias]
        public T* Allocate<T>(long count) where T : unmanaged
        {
            return (T*)Allocate(UnsafeUtility.SizeOf<T>() * count);
        }

        [return : NoAlias]
        private void* Allocate(long size)
        {
            //var threadId = Thread.CurrentThread.ManagedThreadId;

            if (size == 0)
                return null;

#if USE_MALLOC
            return UnsafeUtility.Malloc(size, 16, allocator);
#else
            var ptr = info->Allocate(size);
            if (ptr == null)
            {
                throw new OutOfMemoryException("Insufficient memory in Unity.Numerics.MemoryPool.Allocate()!");
            }
#if TRACKED_ALLOCATIONS
            LeakTrace.AddTrace((IntPtr)ptr);
#endif
            return ptr;
#endif
        }

        public void Free(void* ptr)
        {
            if (ptr == null)
                return;

#if USE_MALLOC
            UnsafeUtility.Free(ptr, allocator);
#else
            info->Free(ptr);

#if TRACKED_ALLOCATIONS
            LeakTrace.RemoveTrace((IntPtr)ptr);
#endif
#endif
        }

        public void Dispose()
        {
#if USE_MALLOC
            return;
#else

#if REPORT_STATS
            string statsLog = $"Blocks remaining: {info->heap->UnsafeCount}\n" +
                $"Memory committed: {(long)info->HeapTop - (long)info->heap}";
            UnityEngine.Debug.Log(statsLog);
#endif
            if (info->activeAllocations != 0)
            {
                UnsafeUtility.Free(info->memory, info->allocator);
                UnsafeUtility.Free(info, info->allocator);

                string msg = $"Leak detected: Unity.Numerics.MemoryPool disposed with {info->activeAllocations} objects still allocated!";
#if TRACKED_ALLOCATIONS
                var it = LeakTrace.stackTrace.Values.GetEnumerator();
                it.MoveNext();
                msg += it.Current.ToString();
#endif
                throw new System.InvalidOperationException(msg);
            }

            UnsafeUtility.Free(info->memory, info->allocator);
            UnsafeUtility.Free(info, info->allocator);
            Valid = false;
#endif
        }

        [DebuggerTypeProxy(typeof(MemoryManagerDebugView))]
        public partial struct Allocator
        {
        }

        [NativeDisableUnsafePtrRestriction]
        public readonly Allocator* info;

        public bool Valid { get; private set; }

#if USE_MALLOC
        readonly Collections.Allocator allocator;
#endif

#if TRACKED_ALLOCATIONS
        public struct LeakTrace
        {
            public static Dictionary<IntPtr, StackTrace> stackTrace = new Dictionary<IntPtr, StackTrace>();
            public static void AddTrace(IntPtr ptr)
            {
                stackTrace[ptr] = new StackTrace(2, true);
            }

            public static void RemoveTrace(IntPtr ptr)
            {
                stackTrace.Remove(ptr);
            }
        }
#endif
    }

    internal unsafe class BlockDebugView
    {
        private MemoryManager.Block block;

        public BlockDebugView(MemoryManager.Block block)
        {
            this.block = block;
        }

        public MemoryManager.Block* This => block.Self;

        public long Size => block.Size;

        public bool Free
        {
            get => block.IsFree;
        }

        public bool Locked
        {
            get => block.IsLocked;
        }
        public MemoryManager.Block* Next => block.Next;
    }

    internal class MemoryManagerDebugView
    {
        private MemoryManager.Allocator heap;

        public MemoryManagerDebugView(MemoryManager.Allocator heap)
        {
            this.heap = heap;
        }

        public MemoryManager.Block[] Free
        {
            get
            {
                return GetBlocksOfType(true);
            }
        }

        public MemoryManager.Block[] Allocated
        {
            get
            {
                return GetBlocksOfType(false);
            }
        }

        public MemoryManager.Block[] Locked
        {
            get
            {
                List<MemoryManager.Block> blocks = new List<MemoryManager.Block>();
                unsafe
                {
                    var block = heap.heap;
                    while (block->IsValid)
                    {
                        if (block->IsLocked)
                            blocks.Add(*block);
                        block = block->Next;
                    }
                }
                return blocks.ToArray();
            }
        }


        private MemoryManager.Block[] GetBlocksOfType(bool free)
        {
            List<MemoryManager.Block> blocks = new List<MemoryManager.Block>();
            unsafe
            {
                var block = heap.heap;
                while (block->IsValid)
                {
                    if (block->IsFree == free)
                        blocks.Add(*block);
                    block = block->Next;
                }
            }
            return blocks.ToArray();
        }
    }
}
