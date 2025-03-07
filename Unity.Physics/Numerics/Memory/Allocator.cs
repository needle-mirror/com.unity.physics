using System;
using System.Threading;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Numerics.Memory
{
    [GenerateTestsForBurstCompatibility]
    internal unsafe partial struct MemoryManager : IDisposable
    {
        public partial struct Allocator
        {
            /// <summary>
            /// Heap start.  All blocks are reachable from this one.
            /// </summary>
            public readonly Block* heap;

            /// <summary>
            /// Heap end.  The last valid block precedes this one.
            /// </summary>
            private IntPtr topAddr;

            /// <summary>
            /// The block of memory reserved by this allocator.
            /// </summary>
            [NativeDisableUnsafePtrRestriction]
            public readonly void* memory;

            /// <summary>
            /// Total memory reserved by this allocator.
            /// </summary>
            public readonly long reservedSize;

            /// <summary>
            /// Amount of memory in bytes that has been allocated.
            /// </summary>
            public long allocatedSize;

            /// <summary>
            /// Total number of outstanding used blocks (number of allocs minus number of frees)
            /// </summary>
            public int activeAllocations;

            /// <summary>
            /// Total number of allocs and frees executed so far
            /// </summary>
            public long operationCount;

            /// <summary>
            /// Dots allocator type
            /// </summary>
            public readonly Collections.Allocator allocator;


            public Allocator(long size, Collections.Allocator allocator)
            {
                var guardSize = UnsafeUtility.SizeOf<Block>();
                memory = UnsafeUtility.Malloc(size + guardSize, 32, allocator);
                UnsafeUtility.MemClear((byte*)memory, size + guardSize);

                this.allocator = allocator;
                activeAllocations = 0;
                reservedSize = size;
                allocatedSize = 0;
                operationCount = 0;

                heap = Block.Construct(memory);
                heap->Alloc(0);

                topAddr = (IntPtr)heap;
                lastSearch = heap;
            }

            [return : NoAlias]
            public byte* Allocate(long size)
            {
                size = Align(size);

                Interlocked.Increment(ref activeAllocations);
                Interlocked.Increment(ref operationCount);

                Block* block = FindFreeBlock(size);

                if (block != null)
                {
                    // see if there's room for another header, and don't split if the resulting block
                    // is too small.
                    long minSize = 32;

                    var newSize = block->Size - size - Block.HeaderSize;
                    if (newSize >= minSize)
                    {
                        var newBlock = Block.Construct(block->Data + size);
                        newBlock->Alloc(newSize, Usage.Free | Usage.Available);
                        block->content = size | (long)Usage.Available;

                        Interlocked.CompareExchange(ref topAddr, (IntPtr)newBlock, (IntPtr)block);
                    }
                    else
                    {
                        block->Flags = Usage.Available;
                    }
                }

                if (block != null)
                {
                    return block->Data;
                }

                // could not find a suitable free block, allocate from remaining memory
                var ptr = ReserveMemory(size);
                if (ptr == null)
                {
                    // out of memory
                    Interlocked.Decrement(ref activeAllocations);
                    return null;
                }

                block = Block.Construct(ptr);
                block->Alloc(size, Usage.Available);

                topAddr = (IntPtr)block;

                return block->Data;
            }

            [return : NoAlias]
            private byte* ReserveMemory(long size)
            {
                size = AllocSize(size);

                var reserved = Interlocked.Add(ref allocatedSize, size);
                if (reserved >= reservedSize)
                {
                    return null;
                }
                return (byte*)memory + reserved - size;
            }

            public void Free([NoAlias] void* ptr)
            {
                Interlocked.Decrement(ref activeAllocations);
                Interlocked.Increment(ref operationCount);

                var block = GetHeader(ptr);
                var size = block->Size;
                var last = block->content | (long)Usage.Available;
                if (Interlocked.CompareExchange(ref block->content, size | (long)(Usage.Free | Usage.Available), last) != last)
                {
                    throw new Exception("Bad block in Allocator.Free()");
                }
            }

            [return : NoAlias]
            Block* FindFreeBlock(long size)
            {
                if (!heap->IsValid)
                    return null;

                Block* leftNode = null, rightNode;

                Block* leftNext = heap;

            searchAgain:
                while (true)
                {
                    Block* t = heap;
                    Block* tnext = t->Next;

                    long free = -Block.HeaderSize;
                    long avail = -Block.HeaderSize;

                    while (t->IsValid && (t->IsAllocatable || free < size))
                    {
                        if (!t->Mark(Usage.Free))
                        {
                            // the run is over but doesn't contain enough free space; unmark and keep going
                            avail = Unmark(leftNext, t);
                            free = -Block.HeaderSize;
                            leftNode = t;
                            leftNext = tnext;
                        }
                        else
                        {
                            free += t->Size + Block.HeaderSize;
                            if (free >= size)
                                break;
                        }

                        if (!tnext->IsValid)
                        {
                            break;
                        }

                        t = tnext;
                        tnext = t->Next;
                    }

                    rightNode = t;

                    if (leftNext > rightNode)
                    {
                        if (avail >= size)
                            goto searchAgain;

                        return null;
                    }

                    if (free < size)
                    {
                        avail = Coalesce(leftNext, rightNode->Next, Usage.Free);
                        if (avail >= size)
                            goto searchAgain;
                        else
                            return null;
                    }

                    // check if the nodes are adjacent
                    if (leftNext == rightNode)
                    {
                        if (rightNode->IsValid && !rightNode->IsFree)
                            goto searchAgain;

                        return (free < size) ? null : rightNode;
                    }

                    // coalesce the nodes
                    if (Coalesce(leftNext, rightNode->Next, Usage.Free) > 0)
                    {
                        return leftNext;
                    }
                    throw new InvalidOperationException("Fatal error in Allocator.FindFreeBlock()");
                }
            }

            long Coalesce(Block* from, Block* to, Usage flags = Usage.Nothing)
            {
                if (from == to)
                    return -Block.HeaderSize;

                var old = (from->content & ~3) | (long)Usage.Free;
                var newSize = ((long)to - (long)from->Data);
                if (Interlocked.CompareExchange(ref from->content, newSize | (long)flags, old) == old)
                {
                    Interlocked.CompareExchange(ref topAddr, (IntPtr)from, (IntPtr)to);
                    return newSize;
                }
                return 0;
            }

            private static long Unmark(Block* from, Block* to)
            {
                var size = -Block.HeaderSize;
                for (var block = from; block != to; block = block->Next)
                {
                    if (block->IsFree)
                        size += block->Size + Block.HeaderSize;

                    if (!block->Unmark())
                    {
                        throw new Exception("Fatal error in Allocator.Unmark()");
                    }
                }

                return size;
            }

            Block* lastSearch;

            public Block* HeapTop { get => (Block*)topAddr; }

            static Block* GetHeader(void* ptr)
            {
                return (Block *)((byte*)ptr - Block.HeaderSize);
            }

            static long Align(long size)
            {
                //return (size + 0xf) & ~0xf;
                return (size + 0x7) & ~0x7;
//                var wordSize = UnsafeUtility.SizeOf<IntPtr>();
//                return (size + wordSize - 1) & ~(wordSize - 1);
            }

            static long AllocSize(long size)
            {
                return size + Block.HeaderSize;
            }
        }
    }
}
