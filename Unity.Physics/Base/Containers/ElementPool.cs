using System;
using System.Runtime.CompilerServices;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine.Assertions;

namespace Unity.Collections
{
    public interface IPoolElement
    {
        bool IsAllocated { get; }
        void MarkFree(int nextFree);
        int NextFree { get; }
    }

    // A fixed capacity array acting as a pool of allocated/free structs referenced by indices
    public struct ElementPool<T> : IDisposable where T : struct, IPoolElement
    {
        private NativeArray<T> m_Elements;    // storage for all elements (allocated and free)
        private readonly bool m_DisposeArray; // whether to dispose the storage on destruction
        private int m_FirstFreeIndex;         // the index of the first free element (or -1 if none free)

        public int Capacity => m_Elements.Length;     // the maximum number of elements that can be allocated
        public int PeakCount { get; private set; }  // the maximum number of elements allocated so far

        public ElementPool(int capacity, Allocator allocator)
        {
            m_Elements = new NativeArray<T>(capacity, allocator);
            m_DisposeArray = true;
            m_FirstFreeIndex = -1;
            PeakCount = 0;
        }

        public unsafe ElementPool(void* userBuffer, int capacity)
        {
            m_Elements = NativeArrayUnsafeUtility.ConvertExistingDataToNativeArray<T>(userBuffer, capacity, Allocator.None);
#if ENABLE_UNITY_COLLECTIONS_CHECKS
            NativeArrayUnsafeUtility.SetAtomicSafetyHandle(ref m_Elements, AtomicSafetyHandle.GetTempUnsafePtrSliceHandle());
#endif
            m_DisposeArray = false;
            m_FirstFreeIndex = -1;
            PeakCount = 0;
        }

        // Add an element to the pool
        public int Allocate(T element)
        {
            Assert.IsTrue(element.IsAllocated);
            if (m_FirstFreeIndex != -1)
            {
                int index = m_FirstFreeIndex;
                m_FirstFreeIndex = m_Elements[index].NextFree;
                m_Elements[index] = element;
                return index;
            }

            Assert.IsTrue(PeakCount < m_Elements.Length);
            m_Elements[PeakCount++] = element;
            return PeakCount - 1;
        }

        // Remove an element from the pool
        public void Release(int index)
        {
            T element = m_Elements[index];
            element.MarkFree(m_FirstFreeIndex);
            m_Elements[index] = element;
            m_FirstFreeIndex = index;
        }

        // Empty the pool
        public void Clear()
        {
            PeakCount = 0;
            m_FirstFreeIndex = -1;
        }

        // Get/set an element
        public T this[int index]
        {
            get { T element = m_Elements[index]; Assert.IsTrue(element.IsAllocated); return element; }
            set { m_Elements[index] = value; }
        }

        // Get the first allocated index
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetFirstIndex()
        {
            return GetNextIndex(-1);
        }

        // Get the next allocated index
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int GetNextIndex(int index)
        {
            for (++index; index < PeakCount; ++index)
            {
                if (m_Elements[index].IsAllocated)
                {
                    return index;
                }
            }
            return -1;
        }

        public unsafe void CopyFrom(ElementPool<T> other)
        {
            Assert.IsTrue(other.m_Elements.Length <= Capacity);
            PeakCount = other.PeakCount;
            m_FirstFreeIndex = other.m_FirstFreeIndex;
            UnsafeUtility.MemCpy(m_Elements.GetUnsafePtr(), other.m_Elements.GetUnsafePtr(), PeakCount * UnsafeUtility.SizeOf<T>());
        }

        public unsafe void CopyFrom(void* buffer, int length)
        {
            Assert.IsTrue(length <= Capacity);
            PeakCount = length;
            m_FirstFreeIndex = -1;
            UnsafeUtility.MemCpy(m_Elements.GetUnsafePtr(), buffer, PeakCount * UnsafeUtility.SizeOf<T>());
        }

        public void Dispose()
        {
            if (m_DisposeArray)
            {
                m_Elements.Dispose();
            }
        }

        #region Enumerables

        public IndexEnumerable Indices => new IndexEnumerable { Slice = new NativeSlice<T>(m_Elements, 0, PeakCount) };
        public ElementEnumerable Elements => new ElementEnumerable { Slice = new NativeSlice<T>(m_Elements, 0, PeakCount) };

        public struct IndexEnumerable
        {
            internal NativeSlice<T> Slice;

            public IndexEnumerator GetEnumerator() => new IndexEnumerator(ref Slice);
        }

        public struct ElementEnumerable
        {
            internal NativeSlice<T> Slice;

            public ElementEnumerator GetEnumerator() => new ElementEnumerator(ref Slice);
        }

        // An enumerator for iterating over the indices
        public struct IndexEnumerator
        {
            internal NativeSlice<T> Slice;
            internal int Index;

            public int Current => Index;

            internal IndexEnumerator(ref NativeSlice<T> slice)
            {
                Slice = slice;
                Index = -1;
            }

            public bool MoveNext()
            {
                while (true)
                {
                    if (++Index >= Slice.Length)
                    {
                        return false;
                    }
                    if (Slice[Index].IsAllocated)
                    {
                        return true;
                    }
                }
            }
        }

        // An enumerator for iterating over the allocated elements
        public struct ElementEnumerator
        {
            internal NativeSlice<T> Slice;
            internal IndexEnumerator IndexEnumerator;

            public T Current => Slice[IndexEnumerator.Current];

            internal ElementEnumerator(ref NativeSlice<T> slice)
            {
                Slice = slice;
                IndexEnumerator = new IndexEnumerator(ref slice);
            }

            public bool MoveNext() => IndexEnumerator.MoveNext();
        }

        #endregion
    }
}
