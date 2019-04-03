//#define BLOCK_STREAM_DEBUG

using System;
using System.Diagnostics;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using UnityEngine.Assertions;

namespace Unity.Collections
{
    // A linked list of fixed size memory blocks for writing arbitrary data to.
    [NativeContainer]
    public unsafe struct BlockStream : IDisposable
    {
        [NativeDisableUnsafePtrRestriction] BlockStreamData* m_Block;
        private readonly Allocator m_AllocatorLabel;

        // Unique "guid" style int used to identify instance of BlockStream for debugging purposes.
        private readonly uint m_UniqueBlockStreamId;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        private AtomicSafetyHandle m_Safety;
        [NativeSetClassTypeToNullOnSchedule]
        private DisposeSentinel m_DisposeSentinel;
#endif

        public BlockStream(int foreachCount, uint uniqueBlockStreamId, Allocator allocator = Allocator.TempJob)
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            if (foreachCount <= 0)
                throw new ArgumentException("foreachCount must be > 0", "foreachCount");
            if (allocator <= Allocator.None)
                throw new ArgumentException("Allocator must be Temp, TempJob or Persistent", "allocator");
#endif

            m_Block = null;
            m_AllocatorLabel = allocator;
            m_UniqueBlockStreamId = uniqueBlockStreamId;

            int blockCount = JobsUtility.MaxJobThreadCount;

            int allocationSize = sizeof(BlockStreamData) + sizeof(Block*) * blockCount + sizeof(Range) * foreachCount;
            byte* buffer = (byte*)UnsafeUtility.Malloc(allocationSize, 16, m_AllocatorLabel);
            UnsafeUtility.MemClear(buffer, allocationSize);

            m_Block = (BlockStreamData*)buffer;
            m_Block->Allocator = m_AllocatorLabel;
            m_Block->BlockCount = blockCount;
            m_Block->Blocks = (Block**)(buffer + sizeof(BlockStreamData));

            m_Block->Ranges = (Range*)((byte*)m_Block->Blocks + sizeof(Block*) * blockCount);
            m_Block->RangeCount = foreachCount;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            DisposeSentinel.Create(out m_Safety, out m_DisposeSentinel, 0, m_AllocatorLabel);
#endif
        }

        public bool IsCreated => m_Block != null;

        public int ForEachCount
        {
            get
            {
                CheckReadAccess();
                return m_Block->RangeCount;
            }
        }

        [Conditional("BLOCK_STREAM_DEBUG")]
        void CheckReadAccess()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
        }

        [Conditional("BLOCK_STREAM_DEBUG")]
        void CheckWriteAccess()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
#endif
        }

        public int ComputeItemCount()
        {
            CheckReadAccess();
            
            int itemCount = 0;

            for (int i = 0; i != m_Block->RangeCount; i++)
            {
                itemCount += m_Block->Ranges[i].Count;
            }

            return itemCount;
        }

        public NativeArray<T> ToNativeArray<T>(Allocator allocator = Allocator.Temp) where T : struct
        {
            CheckReadAccess();
            
            var array = new NativeArray<T>(ComputeItemCount(), allocator, NativeArrayOptions.UninitializedMemory);
            Reader reader = this;

            int offset = 0;
            for (int i = 0; i != reader.ForEachCount; i++)
            {
                reader.BeginForEachIndex(i);
                int rangeItemCount = reader.RemainingItemCount;
                for (int j = 0; j < rangeItemCount; ++j)
                {
                    array[offset] = reader.Read<T>();
                    offset++;
                }
            }

            return array;
        }

        private void _Dispose()
        {
            if (m_Block == null)
            {
                return;
            }

            for (int i = 0; i != m_Block->BlockCount; i++)
            {
                Block* block = m_Block->Blocks[i];
                while (block != null)
                {
                    Block* next = block->Next;
                    UnsafeUtility.Free(block, m_AllocatorLabel);
                    block = next;
                }
            }

            UnsafeUtility.Free(m_Block, m_AllocatorLabel);
            m_Block = null;
        }

        public void Dispose()
        {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            DisposeSentinel.Dispose(ref m_Safety, ref m_DisposeSentinel);
#endif
            _Dispose();
        }

        public JobHandle ScheduleDispose(JobHandle inputDeps)
        {
            // [DeallocateOnJobCompletion] is not supported, but we want the deallocation to happen in a thread.
            // DisposeSentinel needs to be cleared on main thread.
            // AtomicSafetyHandle can be destroyed after the job was scheduled
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            DisposeSentinel.Clear(ref m_DisposeSentinel);
#endif
            var jobHandle = new DisposeJob { BlockStream = this }.Schedule(inputDeps);
            
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle.Release(m_Safety);            
#endif

            m_Block = null;

            return jobHandle;
        }

        public struct Range
        {
            public Block* Block;
            public int Offset;
            public int Count;

            /// One byte past the end of the last byte written
            public int LastOffset;
        }

        public Range* GetRangeRW(int forEachIndex)
        {
            CheckWriteAccess();

            return m_Block->Ranges + forEachIndex;
        }

        public struct BlockStreamData
        {
            public const int AllocationSize = 4 * 1024;
            public Allocator Allocator;

            public Block** Blocks;
            public int BlockCount;

            public Range* Ranges;
            public int RangeCount;

            public Block* Allocate(Block* oldBlock, int threadIndex)
            {
                Assert.IsTrue(threadIndex < BlockCount && threadIndex >= 0);

                Block* block = (Block*)UnsafeUtility.Malloc(AllocationSize, 16, Allocator);
                block->Next = null;

                if (oldBlock == null)
                {
                    if (Blocks[threadIndex] == null)
                        Blocks[threadIndex] = block;
                    else
                    {
                        // Walk the linked list and append our new block to the end.
                        // Otherwise, we leak memory.
                        Block* head = Blocks[threadIndex];
                        while (head->Next != null)
                        {
                            head = head->Next;
                        }
                        head->Next = block;
                    }
                }
                else
                {
                    oldBlock->Next = block;
                }

                return block;
            }
        }

        public struct Block
        {
            public Block* Next;

            public fixed byte Data[1];
        }

        [NativeContainer]
        //@TODO: Try to use min / max check instead...
        [NativeContainerSupportsMinMaxWriteRestriction]
        public struct Writer
        {
            [NativeDisableUnsafePtrRestriction]
            BlockStreamData* m_BlockStream;
            [NativeDisableUnsafePtrRestriction]
            Block* m_CurrentBlock;
            [NativeDisableUnsafePtrRestriction]
            byte* m_CurrentPtr;
            [NativeDisableUnsafePtrRestriction]
            byte* m_CurrentBlockEnd;

            int m_ForeachIndex;
            int m_Count;

            [NativeDisableUnsafePtrRestriction]
            Block* m_FirstBlock;
            int m_FirstOffset;

#pragma warning disable CS0649
            [NativeSetThreadIndex]
            int m_ThreadIndex;
#pragma warning restore CS0649

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            private AtomicSafetyHandle m_Safety;
            int m_Length;
            int m_MinIndex;
            int m_MaxIndex;
#endif
            
            
            public Writer(BlockStream stream)
            {
                this = stream;
            }

            public static implicit operator Writer(BlockStream stream)
            {
                var writer = new Writer();
                writer.m_BlockStream = stream.m_Block;
                writer.m_ForeachIndex = -1;
                writer.m_Count = -1;
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                writer.m_Safety = stream.m_Safety;
                writer.m_Length = stream.ForEachCount;
                writer.m_MinIndex = 0;
                writer.m_MaxIndex = stream.ForEachCount - 1;
#endif
                return writer;
            }

            public int ForEachCount
            {
                get
                {
                    CheckAccess();

                    return m_BlockStream->RangeCount;
                }
            }

            [Conditional("BLOCK_STREAM_DEBUG")]
            void CheckAccess()
            {
                #if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckWriteAndThrow(m_Safety);
                #endif
            }

            public void BeginForEachIndex(int foreachIndex)
            {
                
#if BLOCK_STREAM_DEBUG
                if (foreachIndex < m_MinIndex || foreachIndex > m_MaxIndex)
                    throw new ArgumentException($"Index {foreachIndex} is out of restricted IJobParallelFor range [{m_MinIndex}...{m_MaxIndex}] in BlockStream.");
                Assert.IsTrue(foreachIndex >= 0 && foreachIndex < m_BlockStream->RangeCount);
                Assert.AreEqual(-1, m_ForeachIndex);
                Assert.AreEqual(0, m_BlockStream->Ranges[foreachIndex].Count);
#endif
                               
                m_ForeachIndex = foreachIndex;
                m_Count = 0;
                m_FirstBlock = m_CurrentBlock;
                m_FirstOffset = (int)(m_CurrentPtr - (byte*)m_CurrentBlock);
            }

            public void AppendForEachIndex(int foreachIndex)
            {
                CheckAccess();
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                if (foreachIndex < m_MinIndex || foreachIndex > m_MaxIndex)
                    throw new System.ArgumentException($"forEachIndex {foreachIndex} is not in the allowed range");
#endif

#if BLOCK_STREAM_DEBUG
                Assert.IsTrue(foreachIndex >= 0 && foreachIndex < m_BlockStream->RangeCount);
                Assert.AreEqual(-1, m_ForeachIndex);
#endif

                m_ForeachIndex = foreachIndex;
                m_Count = m_BlockStream->Ranges[foreachIndex].Count;
                m_FirstOffset = m_BlockStream->Ranges[foreachIndex].Offset;
                m_FirstBlock = m_BlockStream->Ranges[foreachIndex].Block;

                if (m_Count > 0)
                {
                    m_CurrentBlock = m_FirstBlock;
                    while (m_CurrentBlock->Next != null)
                    {
                        m_CurrentBlock = m_CurrentBlock->Next;
                    }

                    m_CurrentPtr = (byte*)m_CurrentBlock + m_BlockStream->Ranges[foreachIndex].LastOffset;
                    m_CurrentBlockEnd = (byte*)m_CurrentBlock + BlockStreamData.AllocationSize;
                }
                else
                {
                    Block* allocatedBlock = m_BlockStream->Blocks[m_ThreadIndex];
                    if (allocatedBlock != null)
                    {
                        // This can happen if a job was generated for this work item but didn't produce any
                        // contacts - the range will be empty, but a block will have been preemptively allocated
                        m_FirstBlock = allocatedBlock;
                        m_CurrentBlock = m_FirstBlock;
                        m_CurrentPtr = m_CurrentBlock->Data;
                        m_FirstOffset = (int)(m_CurrentPtr - (byte*)m_CurrentBlock);
                        m_CurrentBlockEnd = (byte*)m_CurrentBlock + BlockStreamData.AllocationSize;
                    }
                }
            }

            public void EndForEachIndex()
            {
#if BLOCK_STREAM_DEBUG
                Assert.AreNotEqual(-1, m_ForeachIndex);
#endif
                CheckAccess();

                m_BlockStream->Ranges[m_ForeachIndex].Count = m_Count;
                m_BlockStream->Ranges[m_ForeachIndex].Offset = m_FirstOffset;
                m_BlockStream->Ranges[m_ForeachIndex].Block = m_FirstBlock;

                m_BlockStream->Ranges[m_ForeachIndex].LastOffset = (int)(m_CurrentPtr - (byte*)m_CurrentBlock);

#if BLOCK_STREAM_DEBUG
                m_ForeachIndex = -1;
#endif
            }

            public void Write<T>(T value) where T : struct
            {
                ref T dst = ref Allocate<T>();
                dst = value;
            }

            //@TODO: Array allocate

            public ref T Allocate<T>() where T : struct
            {
                int size = UnsafeUtility.SizeOf<T>();
                return ref UnsafeUtilityEx.AsRef<T>(Allocate(size));
            }

            //@TODO: Make private or remove it.
            public byte* Allocate(int size)
            {
                CheckAccess();
#if BLOCK_STREAM_DEBUG
                if( m_ForeachIndex == -1 )
                    throw new InvalidOperationException("Allocate must be called within BeginForEachIndex / EndForEachIndex");
                if(size > BlockStreamData.AllocationSize - sizeof(void*))
                    throw new InvalidOperationException("Allocation size is too large");
#endif

                byte* ptr = m_CurrentPtr;
                m_CurrentPtr += size;

                if (m_CurrentPtr > m_CurrentBlockEnd)
                {
                    Block* oldBlock = m_CurrentBlock;

                    m_CurrentBlock = m_BlockStream->Allocate(oldBlock, m_ThreadIndex);
                    m_CurrentPtr = m_CurrentBlock->Data;

                    if (m_FirstBlock == null)
                    {
                        m_FirstOffset = (int)(m_CurrentPtr - (byte*)m_CurrentBlock);
                        m_FirstBlock = m_CurrentBlock;
                    }

                    m_CurrentBlockEnd = (byte*)m_CurrentBlock + BlockStreamData.AllocationSize;
                    ptr = m_CurrentPtr;
                    m_CurrentPtr += size;
                }

                m_Count++;

                return ptr;
            }
        }

        [NativeContainer]
        [NativeContainerIsReadOnly]
        public struct Reader
        {
            [NativeDisableUnsafePtrRestriction]
            BlockStreamData* m_BlockStream;
            [NativeDisableUnsafePtrRestriction]
            Block* m_CurrentBlock;
            [NativeDisableUnsafePtrRestriction]
            byte* m_CurrentPtr;
            [NativeDisableUnsafePtrRestriction]
            byte* m_CurrentBlockEnd;
            [NativeDisableUnsafePtrRestriction]
            byte* m_LastPtr; // The memory returned by the previous Read(). Used by Write() //<todo.eoin Remove this

            int m_RemainingItemCount;

#if ENABLE_UNITY_COLLECTIONS_CHECKS
            AtomicSafetyHandle m_Safety;
#endif
            
            public Reader(BlockStream stream)
            {
                this = stream;
            }

            public static implicit operator Reader(BlockStream stream)
            {
                var reader = new Reader();
                reader.m_BlockStream = stream.m_Block;
                
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                reader.m_Safety = stream.m_Safety;
#endif
                return reader;
            }

            void CheckAccess()
            {
#if ENABLE_UNITY_COLLECTIONS_CHECKS
                AtomicSafetyHandle.CheckReadAndThrow(m_Safety);
#endif
            }

            
            /// <summary>
            /// 
            /// </summary>
            /// <param name="foreachIndex"></param>
            /// <returns> The number of elements at this index</returns>
            public int BeginForEachIndex(int foreachIndex)
            {
                CheckAccess();
                
                m_CurrentBlock = m_BlockStream->Ranges[foreachIndex].Block;
                m_CurrentPtr = (byte*)m_CurrentBlock + m_BlockStream->Ranges[foreachIndex].Offset;
                m_CurrentBlockEnd = (byte*)m_CurrentBlock + BlockStreamData.AllocationSize;

                m_RemainingItemCount = m_BlockStream->Ranges[foreachIndex].Count;

                return m_RemainingItemCount;
            }

            public int ForEachCount => m_BlockStream->RangeCount;

            public int RemainingItemCount => m_RemainingItemCount;

            public byte* Read(int size)
            {
                CheckAccess();
                
                m_RemainingItemCount--;

                byte* ptr = m_CurrentPtr;
                m_CurrentPtr += size;

                if (m_CurrentPtr > m_CurrentBlockEnd)
                {
                    m_CurrentBlock = m_CurrentBlock->Next;
                    m_CurrentPtr = m_CurrentBlock->Data;
                    m_CurrentBlockEnd = (byte*)m_CurrentBlock + BlockStreamData.AllocationSize;

                    ptr = m_CurrentPtr;
                    m_CurrentPtr += size;
                }

                m_LastPtr = ptr;
                return ptr;
            }

            public ref T Read<T>() where T : struct
            {
                int size = UnsafeUtility.SizeOf<T>();

#if BLOCK_STREAM_DEBUG
                Assert.IsTrue(size <= BlockStreamData.AllocationSize - (sizeof(void*)));
                Assert.IsTrue(RemainingItemCount >= 1);
#endif

                return ref UnsafeUtilityEx.AsRef<T>(Read(size));
            }

            public ref T Peek<T>() where T : struct
            {
                CheckAccess();
                
                int size = UnsafeUtility.SizeOf<T>();

#if BLOCK_STREAM_DEBUG
                size += sizeof(int);
                Assert.IsTrue(size <= BlockStreamData.AllocationSize - (sizeof(void*)));
                Assert.IsTrue(m_RemainingItemCount >= 1);
#endif

                byte* ptr = m_CurrentPtr;

                if (ptr + size > m_CurrentBlockEnd)
                    ptr = m_CurrentBlock->Next->Data;

#if BLOCK_STREAM_DEBUG
                ptr += sizeof(int);
#endif

                return ref UnsafeUtilityEx.AsRef<T>(ptr);
            }

            
            
            //LETS FIX THIS!!!!!
            
            //<todo.eoin This does not belong here
            public void Write<T>(T d) where T : struct
            {
                ref T prev = ref UnsafeUtilityEx.AsRef<T>(m_LastPtr);
                prev = d;
            }

        }

        [BurstCompile]
        private struct DisposeJob : IJob
        {
            public BlockStream BlockStream;

            public void Execute()
            {
                BlockStream._Dispose();
            }
        }
    }
}
